function test_rcflex_authority_manager()
%TEST_RCFLEX_AUTHORITY_MANAGER Regression cover for the joint-travel limiter.
%
% HISTORY. V2 of the progress report diagnosed the joint branch of
% rcflex_authority_manager as INOPERATIVE: the predicted correction angle was
% clipped to C.joint_limit before being compared against C.joint_limit, so
% gammaJoint could never fall below 1 and the branch derated authority in zero
% of 3,000 randomised probes. This test exists so that defect cannot return
% silently. It asserts:
%   (1) the rate limiter still limits the rate (the check below bypasses it, so
%       that bypass must not be hiding a second defect);
%   (2) the branch does NOT fire on a demand that fits inside the travel;
%   (3) the branch DOES fire on a demand that does not, and the derate is not
%       explainable by the current branch alone -- attributed using the i_pred
%       the function itself returns, so the test does not re-implement it;
%   (4) over a randomised sweep it fires on over-travel inputs and never on
%       in-travel ones. This is the direct counterpart of the V2 probe that
%       returned zero.
H=rcflex_parameters;B=rcflex_body_properties(H);
C=rcflex_prepare_control(rcflex_control_parameters('ekf_predictive'));
idx=C.ekf.idx;hand=zeros(2,1);

% Probing copy with the rate limiter opened right up, so ONE call reports the
% converged gammaRaw. gammaRaw does not depend on gamma_prev, so this changes
% nothing about the feasibility check itself -- it only removes the ramp.
P=C;P.auth_rate_per_s=1e9;

% ---- (1) the rate limiter still limits ---------------------------------
x=zeros(C.ekf.s_len,1);x(idx.omega)=2*pi*8;x(idx.a1y)=deg2rad(40);
g1=rcflex_authority_manager(1,hand,x(1:13),H,B,C);
assert(abs((1-g1)-C.auth_rate_per_s/C.outer_hz)<1e-9,...
 'Authority must ramp at exactly C.auth_rate_per_s (moved %.5f in one tick).',1-g1);

% ---- (2) an in-travel demand must leave full authority ------------------
x=zeros(C.ekf.s_len,1);x(idx.omega)=2*pi*6;x(idx.a1y)=deg2rad(1);
gSmall=rcflex_authority_manager(1,hand,x(1:13),H,B,P);
assert(abs(gSmall-1)<1e-9,'Authority must stay at 1 for an in-travel demand (got %.4f).',gSmall);

% ---- (3) an over-travel demand must derate, attributably ----------------
% 40 deg of predicted pitch correction against a 14 deg travel limit, at the
% low end of the tremor band so the current branch is not the binding one.
x=zeros(C.ekf.s_len,1);x(idx.omega)=2*pi*C.ekf_omega_min_Hz;x(idx.a1y)=deg2rad(40);
[gBig,iPred]=rcflex_authority_manager(1,hand,x(1:13),H,B,P);
assert(gBig<1-1e-6,'Joint-travel branch did not trigger on a 40 deg demand (gamma=%.4f).',gBig);
gammaCurrentOnly=C.auth_margin*H.current_limit_A/max(iPred,eps);
assert(gBig<gammaCurrentOnly-1e-6,...
 ['Derate is fully explained by the current branch (gamma=%.4f, current branch alone ' ...
  'allows %.4f), so the joint branch is still inoperative.'],gBig,gammaCurrentOnly);
expected=C.joint_limit/peak_demand(x,C);
assert(abs(gBig-min([1 expected gammaCurrentOnly]))<1e-9,...
 'Joint derate %.4f does not equal the expected travel ratio %.4f.',gBig,expected);

% ---- (4) randomised sweep ----------------------------------------------
rng(20260917);n=400;fired=0;overTravel=0;
for k=1:n
 x=zeros(C.ekf.s_len,1);
 x(idx.omega)=2*pi*(C.ekf_omega_min_Hz+rand*(C.ekf_omega_max_Hz-C.ekf_omega_min_Hz));
 amp=deg2rad(30*rand);   % 0-30 deg, straddling the 14 deg travel limit
 x([idx.a1x idx.b1x idx.a1y idx.b1y])=amp*(2*rand(4,1)-1);
 x([idx.a2x idx.b2x idx.a2y idx.b2y])=.3*amp*(2*rand(4,1)-1);
 handK=deg2rad(20)*(2*rand(2,1)-1);
 [g,iP]=rcflex_authority_manager(1,handK,x(1:13),H,B,P);
 qPk=peak_demand(x,C);
 over=qPk>C.joint_limit;overTravel=overTravel+over;
 jointBound=g<min(1,C.auth_margin*H.current_limit_A/max(iP,eps))-1e-9;
 fired=fired+jointBound;
 if jointBound
  assert(over,'Joint branch derated an in-travel demand (peak %.2f deg vs limit %.2f deg).',...
   rad2deg(qPk),rad2deg(C.joint_limit));
 end
end
assert(fired>0,'Joint-travel branch never triggered across %d randomised probes.',n);
fprintf(['Authority manager: rate limit intact; joint-travel branch stays open in travel, ' ...
 'fires on %d of %d over-travel probes (%d probes total), and matches the travel ratio.\n'],...
 fired,overTravel,n);
end

function qPk=peak_demand(x,C)
% Independent recomputation of the RAW (unclipped) predicted correction peak,
% written from the design intent rather than copied from the implementation.
idx=C.ekf.idx;omega=x(idx.omega);qPk=0;
for t=linspace(0,C.auth_horizon_s,5)
 c1=cos(omega*t);s1=sin(omega*t);c2=cos(2*omega*t);s2=sin(2*omega*t);
 tx=x(idx.a1x)*c1+x(idx.b1x)*s1+x(idx.a2x)*c2+x(idx.b2x)*s2;
 ty=x(idx.a1y)*c1+x(idx.b1y)*s1+x(idx.a2y)*c2+x(idx.b2y)*s2;
 qPk=max(qPk,max(abs([ty;tx])));
end
end
