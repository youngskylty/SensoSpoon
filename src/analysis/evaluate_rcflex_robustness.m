function T=evaluate_rcflex_robustness()
% Deliberately adverse engineering assumptions, not vendor tolerance guarantees.
H=rcflex_parameters();nominal=evaluate_rcflex_envelope(H,true);
H.vca_force_constant_N_A=.85*H.vca_force_constant_N_A;
H.coil_R_ohm=1.15*H.coil_R_ohm;H.bus_voltage_V=6.4;
H.flexure_stiffness_Nm_rad=1.1*H.flexure_stiffness_Nm_rad;
H.guide_friction_N=2*H.guide_friction_N;
adverse=evaluate_rcflex_envelope(H,false);
nominal.scenario=repmat("nominal",height(nominal),1);
adverse.scenario=repmat("adverse_assumptions",height(adverse),1);
T=[nominal;adverse];writetable(T,project_path('results','tremor_robustness.csv'));
fprintf('Inverse feasibility only: nominal %d/%d, adverse %d/%d cases within modelled limits.\n',sum(nominal.nominal_feasible),height(nominal),sum(adverse.nominal_feasible),height(adverse));
end
