function T=run_intent_authority_experiment(writeOutputs)
%RUN_INTENT_AUTHORITY_EXPERIMENT The ceiling test behind the sEMG intent gate.
%
% Run this BEFORE reading any sEMG result. It answers two questions with the
% IMU-only estimator alone, and the answers are what motivate the fusion
% architecture and bound what it can possibly achieve:
%
%   1. Is the intent process noise q_intent actually the binding constraint on
%      the remaining EKF failures? Scale it and see.
%   2. Can a single fixed value of it serve every case? If one could, no sensor
%      would be needed and the whole fusion is unnecessary.
%
% The answer to (2) is no, and the experiment shows the dilemma quantitatively
% rather than asserting it: the scaling that most helps the off-harmonic tremor
% cases is the scaling that destroys voluntary motion. That is the trade a
% SENSOR has to break, because it cannot be broken by tuning. Everything the
% sEMG gate does is choose a point on this curve per sample.
%
% No sEMG is generated or read anywhere in this script.
if nargin<1;writeOutputs=true;end

scales=[1 1/2 1/8 1/64 1/512];
cases={ 6,'mixed',    4,8,2,'off-harmonic tremor, no voluntary motion'
        8,'irregular',4,8,2,'off-harmonic tremor, no voluntary motion'
        3,'sine',     4,12,1,'in-band tremor'
        2,'sine',     4,8,1,'in-band tremor, nominal'
        9,'combined', 4,8,1,'intent and tremor ramp together'
       10,'intent',   8,8,0,'voluntary motion only, no tremor'};
rows={};
fprintf('\nIntent-authority ceiling test (IMU-only; no sEMG anywhere)\n');
fprintf('%3s %-10s |','k','scenario');
for s=scales;fprintf('%18s',sprintf('q x 1/%g',1/s));end
fprintf('\n');
for ci=1:size(cases,1)
 k=cases{ci,1};nm=cases{ci,2};dur=cases{ci,3};fHz=cases{ci,4};amp=cases{ci,5};
 fprintf('%3d %-10s |',k,nm);
 for s=scales
  C=rcflex_control_parameters('ekf_predictive');
  C.ekf_intent_model='cv';
  C.ekf_q_intent=C.ekf_q_intent*s;
  C=rcflex_prepare_control(C);
  C.seed=9100+k;                       % the benchmark's own sensor seeds
  D=rcflex_control_case(nm,dur,fHz,amp,C.seed);
  S=run_rcflex_control_fast(D,C,rcflex_parameters,.0005);
  M=rcflex_control_metrics(S);Q=rcflex_intent_split_metrics(S);
  rows(end+1,:)={k,nm,s,M.suppression_percent,M.attitude_rms_deg,...
   Q.tremor_leak_deg,Q.intent_track_deg}; %#ok<AGROW>
  if isnan(M.suppression_percent)
   fprintf('%18s',sprintf('rms %.4f deg',M.attitude_rms_deg));
  else
   fprintf('%18s',sprintf('%7.2f%% %.4fd',M.suppression_percent,M.attitude_rms_deg));
  end
 end
 fprintf('\n');
end
T=cell2table(rows,'VariableNames',{'case_id','scenario','q_intent_scale',...
 'suppression_percent','attitude_rms_deg','tremor_leak_deg','intent_track_deg'});
fprintf('\n');disp(T);

if writeOutputs
 out=project_path('results','intent_authority_ceiling.csv');
 writetable(T,out);fprintf('Wrote %s\n',out);
 % Plotting lives in its own file and reads the CSV back, so a plotting fault
 % cannot cost the 30 closed-loop runs above. It did once.
 plot_rcflex_intent_ceiling();
end
end
