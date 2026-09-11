function T=screen_rcflex_strong_soup()
% Fixed controller/fluid parameters, identical 8 mL on both sides.
C=rcflex_control_parameters;H=rcflex_parameters;H.food_mass_kg=.008;
rows=struct([]);
for amplitude=[3 4 5]
 row=struct('frequency_Hz',4,'amplitude_deg',amplitude,'duration_s',4,'liquid_mL',8,'status',"",'without_spill_mL',NaN,'with_spill_mL',NaN,'attitude_rms_deg',NaN,'current_peak_A',NaN,'stroke_peak_mm',NaN,'joint_peak_deg',NaN,'power_mean_W',NaN,'within_limits',false);
 try
  D=rcflex_control_case('sine',4,4,amplitude);S=run_rcflex_control_fast(D,C,H,.00025);m=rcflex_control_metrics(S);
  row.attitude_rms_deg=m.attitude_rms_deg;row.current_peak_A=m.current_peak_A;row.stroke_peak_mm=m.stroke_peak_mm;row.joint_peak_deg=m.joint_peak_deg;row.power_mean_W=m.power_mean_W;
  row.within_limits=m.current_peak_A<=H.current_limit_A&&m.stroke_peak_mm<=H.vca_stroke_mm&&m.joint_peak_deg<=15&&m.power_mean_W<=H.coil_continuous_power_W&&m.side_load_peak_N<=H.shaft_side_load_N;
  F=simulate_rcflex_liquid(S,rcflex_liquid_parameters,false);row.without_spill_mL=F.sides(1).spilled_mL(end);row.with_spill_mL=F.sides(2).spilled_mL(end);row.status="computed";
  save(project_path('results',sprintf('strong_soup_screen_%gdeg.mat',amplitude)),'S','F','m');
 catch err
  row.status=string(err.identifier)+": "+string(err.message);
 end
 rows=[rows;row]; %#ok<AGROW>
 T=struct2table(rows);writetable(T,project_path('results','strong_soup_screen.csv'));disp(T(end,:));
end
end
