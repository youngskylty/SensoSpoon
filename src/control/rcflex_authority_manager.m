function [gamma_auth,i_pred]=rcflex_authority_manager(gamma_prev,hand,x,H,B,C)
% Predictive current/travel feasibility check. The EKF's oscillator state
% gives the tremor trajectory ahead of time, so the required cancellation
% current is known BEFORE it is demanded: propagate the two-harmonic
% reconstruction forward over C.auth_horizon_s, evaluate the same
% inverse-dynamics helper the offline envelope analysis uses
% (rcflex_required_current, factored out so the two cannot drift apart),
% and derate authority before the allocator saturates rather than after.
%
% gamma_prev: previous authority scale in [0,1] (persisted in controller state)
% hand      : current [roll;pitch] handle attitude (rad) -- held fixed over
%             the short horizon; gravity/inertial coupling terms vary slowly
%             compared to the tremor-frequency current demand being checked.
% x         : 13-state EKF vector, or [] before the first EKF update
%             (returns full authority, ramping at the normal rate limit).
%
% Deviation from docs/CONTROL_IMPROVEMENT_PLAN.md's signature (which also
% takes q, qd, tremor_states separately): simplified here to (hand, x)
% because the predicted joint trajectory is reconstructed analytically
% from the EKF's own quadrature state rather than passed in numerically.
dt=1/C.outer_hz;
if isempty(x)
 gamma_auth=gamma_prev+max(min(1-gamma_prev,C.auth_rate_per_s*dt),-C.auth_rate_per_s*dt);
 i_pred=0;return
end
idx=C.ekf.idx;omega=x(idx.omega);
tau=linspace(0,C.auth_horizon_s,5);
iPeak=0;qPeak=0;
for k=1:numel(tau)
 t=tau(k);c1=cos(omega*t);s1=sin(omega*t);c2=cos(2*omega*t);s2=sin(2*omega*t);
 % Two-harmonic reconstruction and its analytic derivatives (no finite differencing).
 tremX  =      x(idx.a1x)*c1+x(idx.b1x)*s1        +x(idx.a2x)*c2+x(idx.b2x)*s2;
 tremY  =      x(idx.a1y)*c1+x(idx.b1y)*s1        +x(idx.a2y)*c2+x(idx.b2y)*s2;
 tremXd = omega*(-x(idx.a1x)*s1+x(idx.b1x)*c1)+2*omega*(-x(idx.a2x)*s2+x(idx.b2x)*c2);
 tremYd = omega*(-x(idx.a1y)*s1+x(idx.b1y)*c1)+2*omega*(-x(idx.a2y)*s2+x(idx.b2y)*c2);
 tremXdd=-omega^2*(x(idx.a1x)*c1+x(idx.b1x)*s1)-4*omega^2*(x(idx.a2x)*c2+x(idx.b2x)*s2);
 tremYdd=-omega^2*(x(idx.a1y)*c1+x(idx.b1y)*s1)-4*omega^2*(x(idx.a2y)*c2+x(idx.b2y)*s2);
 % Correction joint opposes tremor; axis map: q=[pitch;roll] <-> EKF [y;x].
 q  =max(min(-[tremY;tremX],C.joint_limit),-C.joint_limit);
 qd =-[tremYd;tremXd];qdd=-[tremYdd;tremXdd];
 [iReq,~]=rcflex_required_current(q,qd,qdd,hand,zeros(2,1),zeros(2,1),zeros(9,1),H,B);
 iPeak=max(iPeak,max(abs(iReq)));qPeak=max(qPeak,max(abs(q)));
end
gammaCurrent=C.auth_margin*H.current_limit_A/max(iPeak,eps);
gammaJoint=C.joint_limit/max(qPeak,eps);
gammaRaw=min([1,gammaCurrent,gammaJoint]);
gamma_auth=gamma_prev+max(min(gammaRaw-gamma_prev,C.auth_rate_per_s*dt),-C.auth_rate_per_s*dt);
i_pred=iPeak;
end
