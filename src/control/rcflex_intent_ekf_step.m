function [s,ref,info]=rcflex_intent_ekf_step(s,hand,C)
% Pure function, no persistent variables (determinism required by
% tests/test_rcflex_intent_ekf.m and tests/test_rcflex_control.m).
%
% s    : packed [x(13); P(:)(169); nis_bar(1)] = 183 elements. Pass [] to
%        initialise from C.ekf.x0/P0.
% hand : [roll;pitch] raw handle attitude (rad), same convention as the
%        `hand` local in rcflex_attitude_step.m -- NOT axis-swapped.
% ref  : [pitch;roll] lead-predicted voluntary intent (rad), matching the
%        convention the notch-chain `ref` used so it drops straight into
%        Rd=rx(ref(2))*ry(ref(1)) unchanged.
% info : [omega_hat_Hz; nis_bar; gamma_trust]
%
% Deviation from docs/03_Control_Algorithm_Selection.md Sec 4.5 (which
% recommends BMFLC-KF over EKF): kept per docs/CONTROL_IMPROVEMENT_PLAN.md.
% ω is estimated rather than gridded, which is what lets an out-of-band
% 20 Hz case be tracked at all (a 4-12 Hz BMFLC basis cannot represent it).
E=C.ekf;idx=E.idx;n=E.n;

if isempty(s)
 x=E.x0;P=E.P0;nisBar=C.ekf_nis_lo;
else
 x=s(1:n);P=reshape(s(n+1:n+n*n),n,n);nisBar=s(end);
end

% Trust from the PREVIOUS step's smoothed NIS scales this step's intent
% process noise, so the estimator re-converges within a few samples of a
% transient instead of being dragged along by the oscillator model.
gammaTrustPrev=min(1,max(0,1-(nisBar-C.ekf_nis_lo)/(C.ekf_nis_hi-C.ekf_nis_lo)));

dt=1/C.outer_hz;omega=x(idx.omega);
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

if x(idx.omega)<E.omega_min||x(idx.omega)>E.omega_max
 x(idx.omega)=min(max(x(idx.omega),E.omega_min),E.omega_max);
 P(idx.omega,:)=0;P(:,idx.omega)=0;P(idx.omega,idx.omega)=E.P0(idx.omega,idx.omega);
end

nis=resid'*(S\resid);
alpha=1-exp(-dt/C.ekf_nis_tau_s);nisBar=(1-alpha)*nisBar+alpha*nis;
gammaTrust=min(1,max(0,1-(nisBar-C.ekf_nis_lo)/(C.ekf_nis_hi-C.ekf_nis_lo)));

Dl=C.ekf_lead_s;
ref=[x(idx.thy)+Dl*x(idx.thdy);x(idx.thx)+Dl*x(idx.thdx)];
info=[x(idx.omega)/(2*pi);nisBar;gammaTrust];
s=[x;P(:);nisBar];
end
