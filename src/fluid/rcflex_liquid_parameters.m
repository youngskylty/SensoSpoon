function P=rcflex_liquid_parameters()
% Reduced free-surface model; damping/outflow coefficients need experiments.
P.density_kg_m3=1000;P.damping_ratio=.12;
P.grid_mm=1;P.step_s=.00025;P.output_hz=120;
P.weir_coefficient=.6;P.dry_depth_m=1e-5;
P.name='two_mode_slosh_one_way';
end
