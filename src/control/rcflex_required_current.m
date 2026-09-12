function [i_req,info]=rcflex_required_current(q,qd,qdd,hand,hd,hdd,translation,H,B)
% Shared inverse-dynamics feasibility computation: given a prescribed joint
% trajectory, return the coil current that would be required to produce it
% exactly. Used by both the offline envelope analysis
% (src/analysis/evaluate_rcflex_envelope.m) and the online predictive
% authority manager (rcflex_authority_manager.m) so the two cannot drift
% apart -- extracted from evaluate_rcflex_envelope.m, do not duplicate.
L=rcflex_linkage(q,H);
[M,bias]=rcflex_mass_bias(q,qd,hand,hd,hdd,H,B,translation);
torque=M*qdd+bias+H.flexure_stiffness_Nm_rad.*q+H.flexure_damping_Nms_rad*qd;
sd=L.J*qd;
friction=H.guide_friction_N*tanh(sd/.001)+H.guide_viscous_Ns_m*sd;
force=L.J'\torque+friction;
K=H.vca_force_constant_N_A*max(1-.27*(L.s/(H.vca_stroke_mm/1000)).^2,.5);
i_req=force./K;
d=L.slider-L.attach;
side_load_N=abs(force).*sqrt(sum(d(2:3,:).^2,1))'./abs(d(1,:))';
info=struct('L',L,'K',K,'stroke',L.s,'side_load_N',side_load_N);
end
