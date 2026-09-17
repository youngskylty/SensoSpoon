function T=benchmark_rcflex_control(armFilter)
% Frozen gains, unseen sensor seeds and held-out amplitudes/loads/scenarios.
%
% armFilter (optional) is a string or string array of arm TAGS. When given,
% only those arms are run and their rows REPLACE the matching rows of the
% existing control_benchmark.csv, with every other arm's rows carried through
% unchanged. A full sweep is six arms x twenty cases and takes over an hour, so
% re-running one arm after a change that provably cannot affect the others is
% worth supporting -- but only as a merge, never as a partial file, or the CSV
% would silently become a different experiment. Omit it to run everything.
% Every arm is run over identical cases and identical sensor seeds (9100+k), so
% the comparison is a controlled experiment: the only difference between the
% blocks of rows is the arm definition below.
%
% arms: {algorithm, ekf_intent_model, ekf_osc2, semg_fusion, tag}. The EKF
% fields are inert for simple_pid (no EKF is constructed) and semg_fusion is
% inert unless the EKF is running, so each arm pins the fields it does not use
% at the defaults.
%
% The last three arms are the controlled comparison the sEMG claim rests on.
% They differ ONLY in C.semg_fusion:
%   ekf_cv       IMU-only, nominal intent process noise. The control group.
%   ekf_cv_qfix  IMU-ONLY ABLATION. Same Q-scaling mechanism, constant
%                confidence, no sEMG stream at all. Isolates what the MECHANISM
%                does without the SENSOR -- i.e. the classic "just lower
%                q_intent" fix.
%   ekf_cv_semg  the shipped sEMG+IMU system: the same mechanism driven by the
%                measured voluntary muscle drive.
% Any gain ekf_cv_semg shows over BOTH of the other two is attributable to the
% sEMG stream and to nothing else.
arms={'simple_pid','cv_legacy',false,'off','simple_pid'
      'ekf_predictive','cv_legacy',false,'off','ekf_cv_legacy'
      'ekf_predictive','cv',false,'off','ekf_cv'
      'ekf_predictive','cv',true,'off','ekf_cv_osc2'
      'ekf_predictive','cv',false,'intent_gate_fixed','ekf_cv_qfix'
      'ekf_predictive','cv',false,'intent_gate','ekf_cv_semg'};
if nargin<1;armFilter=[];end
rows=struct([]);
% Carry through the arms this call is not re-running, so the CSV always holds a
% complete sweep.
kept=table();
if ~isempty(armFilter)
 armFilter=string(armFilter);
 assert(all(ismember(armFilter,string(arms(:,5)))),...
  'SensoSpoon:Benchmark','Unknown arm tag in armFilter: %s',strjoin(setdiff(armFilter,string(arms(:,5))),', '));
 f=project_path('results','control_benchmark.csv');
 assert(isfile(f),'SensoSpoon:Benchmark','armFilter needs an existing control_benchmark.csv to merge into.');
 prev=readtable(f,'TextType','string');
 kept=prev(~ismember(prev.arm,armFilter),:);
 arms=arms(ismember(string(arms(:,5)),armFilter),:);
 fprintf('Re-running %d arm(s): %s. Carrying through %d existing rows.\n',...
  size(arms,1),strjoin(armFilter,', '),height(kept));
end
names={'sine','sine','sine','sine','sine','mixed','chirp','irregular','combined','intent','sine','sine','sine','step','sine','sine','sine','sine','sine','sine'};
freq=[4 8 12 6 10 8 8 8 8 8 8 8 8 8 8 5 7 9 11 20];
amp=[1 1 1 3 3 2 2 2 1 0 1 1 1 5 1 1 1 1 1 1];
for a=1:size(arms,1)
 C=rcflex_control_parameters(arms{a,1});C.ekf_intent_model=arms{a,2};C.ekf_osc2=arms{a,3};
 C.semg_fusion=arms{a,4};
 % The sEMG stream costs real time to generate, so it is switched on only for
 % the arm that reads it. Every other arm therefore runs exactly as it ran
 % before the sEMG subsystem existed.
 C.semg_enabled=strcmp(arms{a,4},'intent_gate');
 C=rcflex_prepare_control(C);
 for k=1:numel(names)
  H=rcflex_parameters;C.seed=9100+k;duration=4;
  if k==10;duration=8;end
  if k==11;H.food_mass_kg=0;end
  if k==12;H.food_mass_kg=.015;end
  if k==13;H.bus_voltage_V=5.8;end
  if k==15;H.coil_R_ohm=1.2*H.coil_R_ohm;H.vca_force_constant_N_A=.85*H.vca_force_constant_N_A;H.guide_friction_N=2*H.guide_friction_N;end
  % C is passed so the sEMG arm gets its stream. rcflex_control_case reads C
  % only inside its sEMG branch, so the mechanical scenario every other arm
  % sees is identical with or without it.
  D=rcflex_control_case(names{k},duration,freq(k),amp(k),C.seed,C);
  step=.0005;if k==14||k==20;step=.0001;end
  S=run_rcflex_control_fast(D,C,H,step);m=rcflex_control_metrics(S);
  row=struct('arm',string(arms{a,5}),'algorithm',string(arms{a,1}),'intent_model',string(arms{a,2}),'osc2',double(arms{a,3}),'semg_fusion',string(arms{a,4}),'case_id',k,'scenario',string(names{k}),'frequency_Hz',freq(k),'amplitude_deg',amp(k),'payload_g',H.food_mass_kg*1000,'bus_V',H.bus_voltage_V,'mechanical_step_s',step);
  fields=fieldnames(m);for j=1:numel(fields);row.(fields{j})=m.(fields{j});end
  row.within_limits=m.joint_peak_deg<=15&&m.stroke_peak_mm<=H.vca_stroke_mm&&m.current_peak_A<=H.current_limit_A+1e-9&&m.side_load_peak_N<=H.shaft_side_load_N&&m.power_mean_W<=H.coil_continuous_power_W;
  row.meets_90_percent_target=row.within_limits&&m.suppression_percent>=90;
  % Independent of any controller: the inverse-dynamics feasibility bound in
  % evaluate_rcflex_envelope.m says whether the hardware could serve this input
  % AT ALL under perfect knowledge and exact tracking. Cases outside it are
  % excluded from the headline mean rather than quietly dragging it down -- a
  % controller cannot be scored on an input no controller could serve. The flag
  % is recorded per row so the exclusion is auditable and never applied by hand.
  row.hardware_feasible=rcflex_case_feasible(freq(k),amp(k),H);
  rows=[rows;row]; %#ok<AGROW>
  save(project_path('results',sprintf('control_case_%s_%02d.mat',arms{a,5},k)),'S','m');
  fprintf('%-14s case %02d %s: %.2f%%, %.3f deg, %.2f A, within limits %d, feasible %d\n',arms{a,5},k,names{k},m.suppression_percent,m.attitude_rms_deg,m.current_peak_A,row.within_limits,row.hardware_feasible);
  T=struct2table(rows);
  if ~isempty(kept);T=[kept;T];end
  writetable(T,project_path('results','control_benchmark.csv'));
 end
end
end
