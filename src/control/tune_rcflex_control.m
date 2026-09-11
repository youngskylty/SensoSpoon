function T=tune_rcflex_control()
% Small declared training grid; independent benchmarks use different seeds.
rows=[];
for cutoff=[.8 1 1.2]
 for bw=[25 35]
  C=rcflex_control_parameters;C.intent_filter='lowpass';C.intent_hz=cutoff;C.natural_hz=bw;
  loss=zeros(1,3);freq=[4 8 12];
  for j=1:3
   D=rcflex_control_case('sine',3,freq(j),1); %#ok<NBRAK>
   S=run_rcflex_control_fast(D,C);m=rcflex_control_metrics(S);loss(j)=m.attitude_rms_deg;
  end
  S=run_rcflex_control_fast(rcflex_control_case('intent',5,8,0),C);m=rcflex_control_metrics(S);
  rows=[rows;cutoff bw loss m.attitude_rms_deg]; %#ok<AGROW>
  fprintf('cutoff %.1f bandwidth %g: tremor %.3f %.3f %.3f; intent %.3f deg\n',rows(end,:));
 end
end
T=array2table(rows,'VariableNames',{'intent_hz','bandwidth_hz','rms_4Hz','rms_8Hz','rms_12Hz','intent_error_deg'});writetable(T,project_path('results','control_training.csv'));
end
