function T=run_joint_travel_experiment(writeOutputs)
%RUN_JOINT_TRAVEL_EXPERIMENT What the repaired joint-travel check actually does.
%
% BACKGROUND. rcflex_authority_manager derates control authority when the
% tremor correction it predicts over the next C.auth_horizon_s would exceed
% either the coil current limit or the mechanism's joint travel. The travel
% branch was INOPERATIVE from the day it was written: the predicted angle was
% clipped to C.joint_limit before being compared against C.joint_limit, so the
% comparison could never fail. V2 of the progress report diagnosed it, V3 left
% it open, and it is repaired now (see the ORDERING note in
% rcflex_authority_manager.m and tests/test_rcflex_authority_manager.m).
%
% THE QUESTION THIS ANSWERS is the one the repair does not answer by itself:
% does making the check operative change how the controller behaves? Repairing
% a check that never fires changes nothing, and that is a result, not a
% non-result -- but it can only be claimed by measuring.
%
% THE CONTROL GROUP is C.auth_joint_check=false, which reproduces the
% pre-repair behaviour exactly (gammaJoint pinned at 1) without needing the
% defective code to be kept in the tree. One switch, one file, as with every
% other alternative in this project.
%
% THE SWEEP deliberately runs past anything physiological. The benchmark's
% largest tremor is 5 deg and the travel limit is 14 deg, so if the branch only
% ever fires outside the benchmark, the sweep has to go outside the benchmark
% to find the point where it starts firing. Rows beyond the hardware envelope
% are flagged: a suppression percentage from a run that is hammering its end
% stops describes the end-stop model, not the controller.
if nargin<1;writeOutputs=true;end

% 8 Hz is the centre of the tremor band and the frequency the benchmark uses
% for every non-sine scenario, so amplitude is the only thing varying here.
amps=[1 2 3 5 8 11 14 18 24];
extra={'step',4,8,5,'benchmark case 14 (bus 7.4 V, the worst envelope breach)'
       'sine',4,20,1,'benchmark case 20 (20 Hz, above the modelled tremor band)'};

rows={};
H=rcflex_parameters;
for k=1:numel(amps)
 rows=[rows;sweep_point('sine',4,8,amps(k),sprintf('%g deg tremor at 8 Hz',amps(k)),H)]; %#ok<AGROW>
end
for k=1:size(extra,1)
 rows=[rows;sweep_point(extra{k,1},extra{k,2},extra{k,3},extra{k,4},extra{k,5},H)]; %#ok<AGROW>
end

T=cell2table(rows,'VariableNames',{'scenario','frequency_Hz','amplitude_deg','note',...
 'joint_check','suppression_percent','attitude_rms_deg','joint_peak_deg',...
 'current_peak_A','stroke_peak_mm','power_mean_W','side_load_peak_N',...
 'within_limits','hardware_feasible','gamma_auth_min','gamma_auth_mean'});

fprintf('\n');disp(T);
report(T);

if writeOutputs
 out=project_path('results','joint_travel_authority.csv');
 writetable(T,out);fprintf('Wrote %s\n',out);
 plot_joint_travel_experiment(T);
end
end

% -------------------------------------------------------------------------
function rows=sweep_point(name,dur,fHz,amp,note,H)
%SWEEP_POINT One scenario, run twice: joint check off (pre-repair) and on.
% Identical seed, identical everything else, so the pair is a controlled
% comparison of exactly one boolean.
rows={};
for jc=[false true]
 C=rcflex_control_parameters('ekf_predictive');
 C.ekf_intent_model='cv';
 C.auth_joint_check=jc;
 C.seed=9300+round(10*amp)+fHz;   % seeds outside the benchmark's 9100+k block
 C=rcflex_prepare_control(C);
 D=rcflex_control_case(name,dur,fHz,amp,C.seed,C);
 step=.0005;if fHz>=20;step=.0001;end
 S=run_rcflex_control_fast(D,C,H,step);
 m=rcflex_control_metrics(S);
 % Diagnostic slot 20 is gamma_auth (rcflex_attitude_step: ekfDiag starts at
 % 17, gamma_auth is its 4th element). Scored after the opening ramp, like
 % every other metric in this project.
 w=S.time>=1.5;
 g=S.control(w,20);
 within=m.joint_peak_deg<=15&&m.stroke_peak_mm<=H.vca_stroke_mm&&...
  m.current_peak_A<=H.current_limit_A+1e-9&&m.side_load_peak_N<=H.shaft_side_load_N&&...
  m.power_mean_W<=H.coil_continuous_power_W;
 rows(end+1,:)={string(name),fHz,amp,string(note),jc,m.suppression_percent,...
  m.attitude_rms_deg,m.joint_peak_deg,m.current_peak_A,m.stroke_peak_mm,...
  m.power_mean_W,m.side_load_peak_N,double(within),...
  double(rcflex_case_feasible(fHz,amp,H)),min(g),mean(g)}; %#ok<AGROW>
 fprintf('  %-6s %5.1f deg %4.0f Hz  joint_check=%d : suppression %8.2f%%, joint peak %6.2f deg, gamma_auth min %.4f\n',...
  name,amp,fHz,jc,m.suppression_percent,m.joint_peak_deg,min(g));
end
end

% -------------------------------------------------------------------------
function report(T)
%REPORT State plainly whether the repair changed anything, and where.
off=T(T.joint_check==false,:);on=T(T.joint_check==true,:);
d=abs(on.gamma_auth_min-off.gamma_auth_min);
bind=d>1e-9;
fprintf('\n---------------- joint-travel repair: measured effect ----------------\n');
if ~any(bind)
 fprintf(['The repaired branch did NOT bind anywhere in this sweep, up to %g deg of\n' ...
  'tremor. Authority is identical with the check on and off, so control\n' ...
  'performance is unchanged by the repair over this range.\n'],max(T.amplitude_deg));
else
 first=find(bind,1);
 fprintf('First amplitude at which the branch binds: %g deg (%s at %g Hz).\n',...
  on.amplitude_deg(first),on.scenario(first),on.frequency_Hz(first));
 fprintf('%d of %d sweep points are affected.\n',nnz(bind),numel(bind));
 fprintf('\n%-28s %10s %10s %10s %10s\n','point','gamma off','gamma on','supp off','supp on');
 for k=find(bind)'
  fprintf('%-28s %10.4f %10.4f %10.2f %10.2f\n',...
   sprintf('%s %g deg %g Hz',on.scenario(k),on.amplitude_deg(k),on.frequency_Hz(k)),...
   off.gamma_auth_min(k),on.gamma_auth_min(k),...
   off.suppression_percent(k),on.suppression_percent(k));
 end
end
% The envelope caveat applies to every row, whether or not the branch bound.
nb=sum(T.within_limits==0);
if nb>0
 fprintf(['\n%d of %d rows breach the hardware envelope. Their suppression numbers describe\n' ...
  'the end-stop model rather than the controller and must not be compared as if they\n' ...
  'were control results -- the within_limits and hardware_feasible columns carry this.\n'],...
  nb,height(T));
end
fprintf('----------------------------------------------------------------------\n');
end
