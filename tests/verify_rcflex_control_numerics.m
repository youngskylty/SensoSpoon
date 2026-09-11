function T=verify_rcflex_control_numerics()
d=load(project_path('results','control_native.mat'),'S');N=d.S;
D=rcflex_control_case('sine',N.time(end),8,1,N.C.seed);rows=zeros(3,5);
steps=[.0005 .00025 .0001];mn=rcflex_control_metrics(N);
for j=1:numel(steps)
 S=run_rcflex_control_fast(D,N.C,rcflex_parameters,steps(j));m=rcflex_control_metrics(S);
 delta=rad2deg(S.Y(:,1:2)-N.Y(:,1:2));err=sqrt(mean(sum(delta.^2,2)));
 rows(j,:)=[steps(j) err max(abs(delta),[],'all') m.attitude_rms_deg abs(m.suppression_percent-mn.suppression_percent)];
end
T=array2table(rows,'VariableNames',{'mechanical_step_s','joint_difference_rms_deg','joint_difference_peak_deg','attitude_rms_deg','suppression_difference_points'});
writetable(T,project_path('results','control_numerical_verification.csv'));disp(T);
V=struct('duration_s',N.time(end),'C',N.C,'table',T);save(project_path('results','control_numerical_verification.mat'),'V');
assert(rows(1,2)<.02&&rows(1,5)<2,'Fast benchmark differs materially from native continuous simulation');
assert(rows(end,2)<.02,'Fine-step reference mismatch');
end
