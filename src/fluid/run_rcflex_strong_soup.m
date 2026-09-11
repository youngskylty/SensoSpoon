function [S,F]=run_rcflex_strong_soup()
% Identical 4 Hz / 5 deg inputs and 8 mL fill; unchanged control and fluid parameters.
H=rcflex_parameters;H.food_mass_kg=.008;
D=rcflex_control_case('sine',4,4,5);
S=run_rcflex_control(D,rcflex_control_parameters,H,'strong_soup_native.mat');
S.scenario=struct('frequency_Hz',4,'amplitude_deg',5,'duration_s',4);
m=rcflex_control_metrics(S);disp(m);
assert(m.current_peak_A<=H.current_limit_A && m.stroke_peak_mm<=H.vca_stroke_mm && m.joint_peak_deg<=15 && m.power_mean_W<=H.coil_continuous_power_W && m.side_load_peak_N<=H.shaft_side_load_N,'Strong case exceeds a checked hardware limit');
F=simulate_rcflex_liquid(S,rcflex_liquid_parameters,false);
save(project_path('results','strong_soup_native.mat'),'S','F','m');
P=rcflex_liquid_parameters;P.grid_mm=.5;P.step_s=.000125;
refined=simulate_rcflex_liquid(S,P,false);
save(project_path('results','strong_soup_refinement.mat'),'refined');
T=table([1;.5],[.00025;.000125],[F.sides(1).spilled_mL(end);refined.sides(1).spilled_mL(end)],[F.sides(2).spilled_mL(end);refined.sides(2).spilled_mL(end)],'VariableNames',{'grid_mm','step_s','without_spill_mL','with_spill_mL'});
writetable(T,project_path('results','strong_soup_refinement.csv'));disp(T);
f=animate_rcflex_comparison(S,project_path('figures','soup_strong_tremor_comparison.mp4'),F);close(f);
end
