function T=benchmark_rcflex_control()
% Frozen gains, unseen sensor seeds and held-out amplitudes/loads/scenarios.
C=rcflex_control_parameters;rows=struct([]);
names={'sine','sine','sine','sine','sine','mixed','chirp','irregular','combined','intent','sine','sine','sine','step','sine','sine','sine','sine','sine','sine'};
freq=[4 8 12 6 10 8 8 8 8 8 8 8 8 8 8 5 7 9 11 20];
amp=[1 1 1 3 3 2 2 2 1 0 1 1 1 5 1 1 1 1 1 1];
for k=1:numel(names)
 H=rcflex_parameters;C.seed=9100+k;duration=4;
 if k==10;duration=8;end
 if k==11;H.food_mass_kg=0;end
 if k==12;H.food_mass_kg=.015;end
 if k==13;H.bus_voltage_V=5.8;end
 if k==15;H.coil_R_ohm=1.2*H.coil_R_ohm;H.vca_force_constant_N_A=.85*H.vca_force_constant_N_A;H.guide_friction_N=2*H.guide_friction_N;end
 D=rcflex_control_case(names{k},duration,freq(k),amp(k),C.seed);
 step=.0005;if k==14||k==20;step=.0001;end
 S=run_rcflex_control_fast(D,C,H,step);m=rcflex_control_metrics(S);
 row=struct('case_id',k,'scenario',string(names{k}),'frequency_Hz',freq(k),'amplitude_deg',amp(k),'payload_g',H.food_mass_kg*1000,'bus_V',H.bus_voltage_V,'mechanical_step_s',step);
 fields=fieldnames(m);for j=1:numel(fields);row.(fields{j})=m.(fields{j});end
 row.within_limits=m.joint_peak_deg<=15&&m.stroke_peak_mm<=H.vca_stroke_mm&&m.current_peak_A<=H.current_limit_A+1e-9&&m.side_load_peak_N<=H.shaft_side_load_N&&m.power_mean_W<=H.coil_continuous_power_W;
 row.meets_90_percent_target=row.within_limits&&m.suppression_percent>=90;
 rows=[rows;row]; %#ok<AGROW>
 save(project_path('results',sprintf('control_case_%02d.mat',k)),'S','m');
 fprintf('Case %02d %s: %.2f%%, %.3f deg, %.2f A, within limits %d\n',k,names{k},m.suppression_percent,m.attitude_rms_deg,m.current_peak_A,row.within_limits);
 T=struct2table(rows);writetable(T,project_path('results','control_benchmark.csv'));
end
end
