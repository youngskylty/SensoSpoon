function [s,ref,info]=rcflex_intent_ekf_step(s,hand,C,semgOmega,intentConf)
% Pure function, no persistent variables (determinism required by
% tests/test_rcflex_intent_ekf.m and tests/test_rcflex_control.m).
%
% s    : packed [x(13); P(:)(169); nis_bar(1)] = C.ekf.s_len (183) elements,
%        plus the osc2 presence-gate state when C.ekf_osc2 is set. Pass [] to
%        initialise from C.ekf.s0.
% hand : [roll;pitch] raw handle attitude (rad), same convention as the
%        `hand` local in rcflex_attitude_step.m -- NOT axis-swapped.
% ref  : [pitch;roll] lead-predicted voluntary intent (rad), matching the
%        convention the notch-chain `ref` used so it drops straight into
%        Rd=rx(ref(2))*ry(ref(1)) unchanged.
% intentConf : OPTIONAL 2x1 voluntary-drive confidence in [0,1], [roll; pitch],
%        from rcflex_semg_intent_gate. Scales the intent process noise per
%        axis. Omitted or [] leaves Q exactly as it was, so every IMU-only
%        result is bit-identical.
%
% info : [omega_hat_Hz; nis_bar; gamma_trust; 0; 0]. The last two slots are
%        reserved (they carried an independent frequency observable in an
%        experiment that has since been reverted); the diagnostic vector width
%        depends on them, so they stay present and read zero.
%
% Deviation from docs/03_Control_Algorithm_Selection.md Sec 4.5 (which
% recommends BMFLC-KF over EKF): kept per docs/CONTROL_IMPROVEMENT_PLAN.md.
% ω is estimated rather than gridded, which is what lets an out-of-band
% 20 Hz case be tracked at all (a 4-12 Hz BMFLC basis cannot represent it).
E=C.ekf;idx=E.idx;n=E.n;

if isempty(s);s=E.s0;end
x=s(1:n);P=reshape(s(n+1:n+n*n),n,n);nisBar=s(E.ix_nis);

% Trust from the PREVIOUS step's smoothed NIS scales this step's intent
% process noise, so the estimator re-converges within a few samples of a
% transient instead of being dragged along by the oscillator model.
gammaTrustPrev=min(1,max(0,1-(nisBar-C.ekf_nis_lo)/(C.ekf_nis_hi-C.ekf_nis_lo)));

dt=1/C.outer_hz;omega=x(idx.omega);
omega2=[0;0];if E.osc2;omega2=x(E.om2_ix);end

% Presence gate for the second oscillator (C.ekf_osc2). omega2x/y are informed
% only by the joint fit, so nothing stops them railing against a bound and
% explaining voluntary intent as a low-frequency "tremor" when no such tremor
% exists -- measured at 1.1-2.5 deg on 'combined'.
%
% The gate asks "how much does the BASE model fail to explain, per axis?" and
% is driven by the filter residual rather than the raw measurement. Driving it
% from the raw signal does not work: the tracked fundamental leaks through any
% practical lowpass and supplies ~95% of the measured band power, so the gate
% reads the same on a scenario with real off-harmonic content and one without.
% The residual has every modelled component subtracted out by construction, so
% the fundamental disappears from it and only unmodelled content survives.
%
% Two details matter. The residual is only available after this step's update,
% so the gate uses the PREVIOUS step's value (the same one-step-late pattern as
% gammaTrustPrev above). And the power is measured on the residual with the
% second oscillator's own contribution ADDED BACK -- otherwise a working
% oscillator would shrink the very signal keeping its gate open, and the gate
% would chatter.
conf2=[0;0];
if E.osc2
 gatePowPrev=s(E.ix_gate_pow);
 conf2=gatePowPrev./(gatePowPrev+E.gate_floor);
end
% ---------------------------------------------------------------------------

F=eye(n);F(idx.thx,idx.thdx)=dt;F(idx.thy,idx.thdy)=dt;
if E.ca
 F(idx.thx,idx.accx)=.5*dt^2;F(idx.thdx,idx.accx)=dt;
 F(idx.thy,idx.accy)=.5*dt^2;F(idx.thdy,idx.accy)=dt;
end
% [axis-a-index axis-b-index harmonic-number frequency-state-index]
% The frequency-state column lets the second oscillator (states 14:19, enabled
% by C.ekf_osc2) rotate at its OWN omega2 rather than a harmonic of omega,
% which is what gives off-harmonic tremor content somewhere to live.
pairs=[idx.a1x idx.b1x 1 idx.omega;idx.a2x idx.b2x 2 idx.omega;...
       idx.a1y idx.b1y 1 idx.omega;idx.a2y idx.b2y 2 idx.omega];
if E.osc2
 pairs=[pairs;idx.a3x idx.b3x 1 idx.omega2x;idx.a3y idx.b3y 1 idx.omega2y];
end
xpred=x;
% Intent mean propagation. Under 'cv_legacy' this is deliberately SKIPPED, so
% the angle mean stays put while F above advances the covariance as though it
% had moved -- the original behaviour, kept bit-identical by default. 'cv'
% repairs that inconsistency; 'ca' additionally carries the acceleration state.
switch E.intent_model
 case 'cv'
  xpred(idx.thx)=x(idx.thx)+dt*x(idx.thdx);
  xpred(idx.thy)=x(idx.thy)+dt*x(idx.thdy);
 case 'ca'
  xpred(idx.thx)=x(idx.thx)+dt*x(idx.thdx)+.5*dt^2*x(idx.accx);
  xpred(idx.thdx)=x(idx.thdx)+dt*x(idx.accx);
  xpred(idx.thy)=x(idx.thy)+dt*x(idx.thdy)+.5*dt^2*x(idx.accy);
  xpred(idx.thdy)=x(idx.thdy)+dt*x(idx.accy);
end
for p=1:size(pairs,1)
 ai=pairs(p,1);bi=pairs(p,2);h=pairs(p,3);wi=pairs(p,4);
 phi=h*x(wi)*dt;c=cos(phi);sn=sin(phi);
 a=x(ai);b=x(bi);
 xpred(ai)=c*a-sn*b;xpred(bi)=sn*a+c*b;
 F(ai,ai)=c;F(ai,bi)=-sn;F(bi,ai)=sn;F(bi,bi)=c;
 % d/domega of R(phi)*[a;b], phi=h*omega*dt: dR/dphi=[-sin -cos; cos -sin]
 F(ai,wi)=h*dt*(-sn*a-c*b);F(bi,wi)=h*dt*(c*a-sn*b);
end

Qeff=E.Q;boost=1+C.ekf_q_boost*(1-gammaTrustPrev);
% The trust boost must act on whichever state actually drives the intent chain,
% or it does nothing: under 'ca' the velocity entries are 1e-10 placeholders.
ivel=[idx.thdx idx.thdy];
if E.ca;ivel=[idx.accx idx.accy];end
Qeff(ivel,ivel)=Qeff(ivel,ivel)*boost;
% --- sEMG intent gate (C.semg_fusion='intent_gate*') ------------------------
% The one place sEMG changes the estimator. It scales the intent process noise
% per axis by the measured voluntary muscle drive: with no voluntary drive the
% intent states lose the freedom to oscillate, so off-harmonic tremor the
% {omega,2*omega} basis cannot represent is left in the residual (where the
% controller rejects it as a disturbance) instead of being absorbed into the
% reference as if it were voluntary motion.
%
% Applied AFTER the NIS trust boost and multiplying it, so the gate scales the
% boost down as well as the base process noise.
%
% THE OBVIOUS OBJECTION WAS TESTED AND IS MOOT. The objection: a fast voluntary
% movement is exactly what makes NIS spike, so a gate that has not yet detected
% the movement would veto the one mechanism that could have caught it. The
% alternative was implemented -- splitting the boost into its base and
% transient parts and gating only the base, Qeff = q_intent*(gate + boost - 1),
% so the transient part survives any gate -- and run on the 'reach' scenario at
% a matched sensor seed. Both forms produced the SAME result to four
% significant figures (5.21 % suppression, 0.9478 deg RMS).
%
% That identity is the finding: the boost is not firing at all during a fast
% voluntary movement, so which way it combines with the gate cannot matter.
% gamma_trust stays near 1 because intent + tremor still reconstruct the
% measurement accurately even while the decomposition is wrong -- the same NIS
% blindness documented in Sec 1 of docs/SEMG_INTENT_FUSION.md, and the reason
% this fusion exists at all. The residual cannot rescue voluntary onset, and
% the multiplicative form is kept because it is the simpler of two equivalents.
%
% (An earlier note here claimed the two forms differed by 20 points. That
% comparison was run at two different sensor seeds and was wrong; the matched
% comparison above replaces it.)
%
% The fast-voluntary-onset cost is real and is addressed in the GATE instead,
% by its two-speed structure -- see rcflex_semg_intent_gate.m and
% docs/SEMG_INTENT_FUSION.md Sec 4.3/9.
%
% The floor is a numerical safeguard only. The ceiling experiment
% (docs/SEMG_INTENT_FUSION.md Sec 2) showed the benefit saturates by a scale of
% 1/64, and the gate's own closed value is about 1/16 once the fast path's
% floor is included, so C.semg_gate_q_min at 0.01 does not bind on any measured
% case.
if nargin>=5&&~isempty(intentConf)
 qMin=.01;
 if isfield(C,'semg_gate_q_min');qMin=C.semg_gate_q_min;end
 qScale=max(min(max(intentConf(:),0),1),qMin);
 for a=1:2;Qeff(ivel(a),ivel(a))=Qeff(ivel(a),ivel(a))*qScale(a);end
end
if E.osc2
 % Starve the second oscillator when there is no low-band tremor to track.
 gi=[E.q3x_ix E.om2_ix(1)];gj=[E.q3y_ix E.om2_ix(2)];
 for k=1:numel(gi);Qeff(gi(k),gi(k))=Qeff(gi(k),gi(k))*conf2(1);end
 for k=1:numel(gj);Qeff(gj(k),gj(k))=Qeff(gj(k),gj(k))*conf2(2);end
end
Pm=F*P*F'+Qeff;

z=hand(:);resid=z-E.H*xpred;S=E.H*Pm*E.H'+E.R;K=Pm*E.H'/S;
x=xpred+K*resid;P=Pm-K*E.H*Pm;P=(P+P')/2;

% Gate pull: sequential scalar pseudo-measurements asserting a3=b3=0, with
% noise inflated by 1/(1-conf2) so the assertion vanishes as real low-band
% tremor appears and clamps hard when it is absent.
if E.osc2
 pullIx=[E.q3x_ix E.q3y_ix];
 pullR=[repmat(E.gate_r/max(1-conf2(1),1e-6),1,2), ...
        repmat(E.gate_r/max(1-conf2(2),1e-6),1,2)];
 for k=1:numel(pullIx)
  gk=pullIx(k);
  Sg=P(gk,gk)+pullR(k);Kg=P(:,gk)/Sg;
  x=x-Kg*x(gk);
  P=P-Kg*P(gk,:);P=(P+P')/2;
 end
end

% sEMG frequency pseudo-measurement (C.semg_fusion). Scalar, LINEAR, on state
% 13 only -- no new states, and states 1:12 stay informed by the IMU alone.
%
% This is the structural remedy Sec 1a called for: information about omega that
% does not come from the joint fit. Sequential scalar update after the IMU
% update, the same pattern the osc2 gate-pull above already uses.
%
% semgOmega = [omega_rad_per_s; confidence]. Confidence divides R rather than
% switching the update on and off, so the measurement fades in and out smoothly
% and a low-amplitude stretch cannot step the estimate. When confidence is zero
% R_eff is infinite and the update contributes nothing.
%
% Placed BEFORE the rate limit and bounds below deliberately: the sEMG estimate
% is subject to exactly the same guards as the joint-fit estimate, so it cannot
% slew or rail omega in ways the IMU-only filter could not.
if nargin>=4&&~isempty(semgOmega)&&semgOmega(2)>0
 wi=idx.omega;
 Rw=C.semg_fuse_r_omega/max(semgOmega(2),1e-6);
 Sw=P(wi,wi)+Rw;Kw=P(:,wi)/Sw;
 x=x+Kw*(semgOmega(1)-x(wi));
 P=P-Kw*P(wi,:);P=(P+P')/2;
end

% Rate-limit omega itself. Any single sample's information about omega is
% weak (the Jacobian is nearly singular at low amplitude -- e.g. during the
% startup ramp every test scenario begins with), so an otherwise-plausible
% Kalman step can occasionally propose an implausibly large jump with no
% real tremor behind it. A straightforward rate limit (real WFLC/BMFLC
% frequency-adaptation terms are bounded in practice too) is a simpler and
% more predictable safeguard than trying to make the gain itself amplitude-
% aware, and does not perturb the covariance bookkeeping.
domega=x(idx.omega)-omega;maxStep=C.ekf_omega_rate_Hz_per_s*2*pi*dt;
x(idx.omega)=omega+max(min(domega,maxStep),-maxStep);
if x(idx.omega)<E.omega_min||x(idx.omega)>E.omega_max
 x(idx.omega)=min(max(x(idx.omega),E.omega_min),E.omega_max);
 P(idx.omega,:)=0;P(:,idx.omega)=0;P(idx.omega,idx.omega)=E.P0(idx.omega,idx.omega);
end
if E.osc2
 % Same safeguard for omega2: its only information comes from the joint fit,
 % so the rate limit matters more here, not less.
 maxStep2=E.omega2_rate*2*pi*dt;
 for a=1:2
  wi=E.om2_ix(a);
  dw2=x(wi)-omega2(a);
  x(wi)=omega2(a)+max(min(dw2,maxStep2),-maxStep2);
  if x(wi)<E.omega2_min||x(wi)>E.omega2_max
   x(wi)=min(max(x(wi),E.omega2_min),E.omega2_max);
   P(wi,:)=0;P(:,wi)=0;P(wi,wi)=E.P0(wi,wi);
  end
 end
end

nis=resid'*(S\resid);
alpha=1-exp(-dt/C.ekf_nis_tau_s);nisBar=(1-alpha)*nisBar+alpha*nis;
gammaTrust=min(1,max(0,1-(nisBar-C.ekf_nis_lo)/(C.ekf_nis_hi-C.ekf_nis_lo)));

Dl=C.ekf_lead_s;
% The lead predictor uses the model the filter actually carries, so under 'ca'
% it extrapolates with the acceleration term too.
if E.ca
 ref=[x(idx.thy)+Dl*x(idx.thdy)+.5*Dl^2*x(idx.accy);...
      x(idx.thx)+Dl*x(idx.thdx)+.5*Dl^2*x(idx.accx)];
else
 ref=[x(idx.thy)+Dl*x(idx.thdy);x(idx.thx)+Dl*x(idx.thdx)];
end
info=[x(idx.omega)/(2*pi);nisBar;gammaTrust;0;0];
s(1:n)=x;s(n+1:n+n*n)=P(:);s(E.ix_nis)=nisBar;
if E.osc2
 % Band-limited power of the base-model residual, per axis, with the second
 % oscillator's own contribution added back (see the gate comment above).
 residBase=resid+[xpred(idx.a3x);xpred(idx.a3y)];
 gateLp=reshape(s(E.ix_gate_lp),2,2);v=residBase(:)';
 for st=1:2
  v=E.gate_pole*gateLp(st,:)+(1-E.gate_pole)*v;gateLp(st,:)=v;
 end
 gatePow=s(E.ix_gate_pow)+E.gate_alpha*((v(:).^2)-s(E.ix_gate_pow));
 s(E.ix_gate_lp)=gateLp(:);s(E.ix_gate_pow)=gatePow;
end
end
