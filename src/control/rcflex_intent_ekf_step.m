function [s,ref,info]=rcflex_intent_ekf_step(s,hand,C)
% Pure function, no persistent variables (determinism required by
% tests/test_rcflex_intent_ekf.m and tests/test_rcflex_control.m).
%
% s    : packed [x(13); P(:)(169); nis_bar(1); frequency-observable state]
%        = C.ekf.s_len (227) elements. Pass [] to initialise from C.ekf.s0.
% hand : [roll;pitch] raw handle attitude (rad), same convention as the
%        `hand` local in rcflex_attitude_step.m -- NOT axis-swapped.
% ref  : [pitch;roll] lead-predicted voluntary intent (rad), matching the
%        convention the notch-chain `ref` used so it drops straight into
%        Rd=rx(ref(2))*ry(ref(1)) unchanged.
% info : [omega_hat_Hz; nis_bar; gamma_trust; omega_indep_Hz; freq_confidence]
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

% ---------------------------------------------------------------------------
% Independent tremor-frequency observable.
%
% Why this exists. The joint intent + two-harmonic-tremor + shared-ω model is
% weakly identifiable during a transient in which BOTH the voluntary intent and
% the tremor amplitude are ramping simultaneously (the 'combined' scenario).
% Over a short window the near-constant-velocity intent states can absorb part
% of the oscillation, so the same data is explained almost as well at a wrong ω
% as at the right one: the filter converges to a false low ω while remaining
% internally self-consistent, which is why the measurement NIS (a single-step
% "does this sample fit" statistic) stays near zero throughout and cannot
% detect the failure. No amount of gating, slowing or filtering the single ω
% adaptation law fixes an identifiability problem -- it only trades false-lock
% robustness against convergence speed, since both route through that one
% state. The structural remedy is to ADD INFORMATION about ω that does not
% come from the joint fit at all.
%
% The added observable is a lag-D autocorrelation (Pisarenko-style) frequency
% estimate of a high-passed copy of the measurement:
%
%   h[k]                = HP(hand[k])          (2-pole, C.freq_hp_Hz)
%   cos(ω·D·dt)·2·E[h²] = E[ h[k-D]·(h[k] + h[k-2D]) ]
%
% for a single sinusoid. Intent is removed by the high-pass (voluntary reaching
% motion is < 2 Hz; the EKF already assumes tremor lies in 2-25 Hz), not by the
% ambiguous joint fit, so this estimate is structurally immune to the
% intent/tremor trade-off above. Measurement noise enters the numerator only
% as zero-mean cross terms, which the EWMA averages out, and the lag D=8 puts
% ω·D·dt near the well-conditioned middle of acos() over 2-25 Hz while keeping
% the estimate unambiguous up to 50 Hz.
%
% It is injected as a sequential scalar pseudo-measurement on state 13, with
% noise inflated by 1/confidence and rejected outright when the estimate falls
% outside the admissible band -- so it adds information when tremor is present
% and observable, and contributes nothing when it is not. Because it is a
% measurement rather than a filtered version of ω itself, it does not create
% the self-reinforcing phase-prediction error that lagging ω inside the
% process model does.
D=E.freq_lag;hl=E.freq_hist_len;
hpY=reshape(s(E.ix_hp_y),2,2);hpU=reshape(s(E.ix_hp_u),2,2);
hHist=reshape(s(E.ix_hist),hl,2);
hpPole=E.freq_hp_pole;u=hand(:)';
for st=1:2
 hpOut=hpPole*(hpY(st,:)+u-hpU(st,:));hpU(st,:)=u;hpY(st,:)=hpOut;u=hpOut;
end
hHist=[u;hHist(1:end-1,:)];
centre=hHist(D+1,:);
numRaw=sum(centre.*(hHist(1,:)+hHist(hl,:)));denRaw=sum(2*centre.^2);
numBar=s(E.ix_num)+E.freq_alpha*(numRaw-s(E.ix_num));
denBar=s(E.ix_den)+E.freq_alpha*(denRaw-s(E.ix_den));
cosHat=max(min(numBar/max(denBar,eps),.9999),-.9999);
omegaIndep=acos(cosHat)/(D*dt);
tremorPower=denBar/2;
freqConf=tremorPower/(tremorPower+E.freq_power_floor);
if omegaIndep<E.omega_min||omegaIndep>E.omega_max;freqConf=0;end
% ---------------------------------------------------------------------------

F=eye(n);F(idx.thx,idx.thdx)=dt;F(idx.thy,idx.thdy)=dt;
% [axis-a-index axis-b-index harmonic-number]
pairs=[idx.a1x idx.b1x 1;idx.a2x idx.b2x 2;idx.a1y idx.b1y 1;idx.a2y idx.b2y 2];
xpred=x;
for p=1:size(pairs,1)
 ai=pairs(p,1);bi=pairs(p,2);h=pairs(p,3);phi=h*omega*dt;c=cos(phi);sn=sin(phi);
 a=x(ai);b=x(bi);
 xpred(ai)=c*a-sn*b;xpred(bi)=sn*a+c*b;
 F(ai,ai)=c;F(ai,bi)=-sn;F(bi,ai)=sn;F(bi,bi)=c;
 % d/domega of R(phi)*[a;b], phi=h*omega*dt: dR/dphi=[-sin -cos; cos -sin]
 F(ai,idx.omega)=h*dt*(-sn*a-c*b);F(bi,idx.omega)=h*dt*(c*a-sn*b);
end

Qeff=E.Q;boost=1+C.ekf_q_boost*(1-gammaTrustPrev);
ivel=[idx.thdx idx.thdy];Qeff(ivel,ivel)=Qeff(ivel,ivel)*boost;
Pm=F*P*F'+Qeff;

z=hand(:);resid=z-E.H*xpred;S=E.H*Pm*E.H'+E.R;K=Pm*E.H'/S;
x=xpred+K*resid;P=Pm-K*E.H*Pm;P=(P+P')/2;

% Sequential scalar update with the independent frequency observable. Applied
% after the attitude update so `nis` below stays a pure attitude-fit statistic
% (the trust gate's meaning, and tests/test_rcflex_intent_ekf.m's step
% assertions, are unchanged by this addition).
if freqConf>1e-3
 Sw=P(idx.omega,idx.omega)+E.freq_r/freqConf;
 Kw=P(:,idx.omega)/Sw;
 x=x+Kw*(omegaIndep-x(idx.omega));
 P=P-Kw*P(idx.omega,:);P=(P+P')/2;
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

nis=resid'*(S\resid);
alpha=1-exp(-dt/C.ekf_nis_tau_s);nisBar=(1-alpha)*nisBar+alpha*nis;
gammaTrust=min(1,max(0,1-(nisBar-C.ekf_nis_lo)/(C.ekf_nis_hi-C.ekf_nis_lo)));

Dl=C.ekf_lead_s;
ref=[x(idx.thy)+Dl*x(idx.thdy);x(idx.thx)+Dl*x(idx.thdx)];
info=[x(idx.omega)/(2*pi);nisBar;gammaTrust;omegaIndep/(2*pi);freqConf];
s(1:n)=x;s(n+1:n+n*n)=P(:);s(E.ix_nis)=nisBar;
s(E.ix_hp_y)=hpY(:);s(E.ix_hp_u)=hpU(:);s(E.ix_hist)=hHist(:);
s(E.ix_num)=numBar;s(E.ix_den)=denBar;
end
