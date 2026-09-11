function F=run_rcflex_soup_demo(volume_mL)
% Recompute mechanics with matching liquid payload, then one-way fluid response.
if nargin<1;volume_mL=5;end
H=rcflex_parameters;P=rcflex_liquid_parameters;G=rcflex_liquid_geometry(H,P);
assert(volume_mL>0&&volume_mL<G.capacity_m3*1e6,'Volume exceeds bowl capacity');
H.food_mass_kg=volume_mL*1e-6*P.density_kg_m3;
D=rcflex_control_case('sine',2,8,1);
S=run_rcflex_control(D,rcflex_control_parameters,H,sprintf('soup_%gml_native.mat',volume_mL));
F=simulate_rcflex_liquid(S,P,false);
save(project_path('results',sprintf('liquid_%gml.mat',volume_mL)),'S','F');
animate_rcflex_comparison(S,project_path('figures',sprintf('soup_%gml_comparison.mp4',volume_mL)),F);
end
