function C=rcflex_intent_ekf_init(C)
% Build the constant matrices and index map for the 13-state joint intent/
% tremor EKF (docs/CONTROL_IMPROVEMENT_PLAN.md). Called once from
% rcflex_prepare_control before the controller starts; the per-step
% Jacobian only ever needs the omega column recomputed (rcflex_intent_ekf_step).
%
% State: [th_ix thd_ix th_iy thd_iy a1x b1x a2x b2x a1y b1y a2y b2y omega]
% With C.ekf_osc2=true a second INDEPENDENT oscillator is appended, so the
% state becomes [ ... omega a3x b3x a3y b3y omega2] (n=18). States 1:13 keep
% their indices, so every downstream use of idx.omega is unaffected by the
% extension.
osc2=isfield(C,'ekf_osc2')&&C.ekf_osc2;
n=13+6*osc2;
idx=struct('thx',1,'thdx',2,'thy',3,'thdy',4,...
 'a1x',5,'b1x',6,'a2x',7,'b2x',8,'a1y',9,'b1y',10,'a2y',11,'b2y',12,'omega',13);
if osc2
 idx.a3x=14;idx.b3x=15;idx.a3y=16;idx.b3y=17;idx.omega2x=18;idx.omega2y=19;
end

Q=zeros(n);
Q(idx.thx,idx.thx)=1e-10;Q(idx.thdx,idx.thdx)=C.ekf_q_intent;   % intent, axis x (roll)
Q(idx.thy,idx.thy)=1e-10;Q(idx.thdy,idx.thdy)=C.ekf_q_intent;   % intent, axis y (pitch)
h1Ix=[idx.a1x idx.b1x idx.a1y idx.b1y];h2Ix=[idx.a2x idx.b2x idx.a2y idx.b2y];
for k=1:numel(h1Ix);Q(h1Ix(k),h1Ix(k))=C.ekf_q_tremor;end
for k=1:numel(h2Ix);Q(h2Ix(k),h2Ix(k))=C.ekf_q_tremor2;end
Q(idx.omega,idx.omega)=C.ekf_q_omega;
if osc2
 h3Ix=[idx.a3x idx.b3x idx.a3y idx.b3y];
 for k=1:numel(h3Ix);Q(h3Ix(k),h3Ix(k))=C.ekf_q_tremor3;end
 Q(idx.omega2x,idx.omega2x)=C.ekf_q_omega2;
 Q(idx.omega2y,idx.omega2y)=C.ekf_q_omega2;
end

% Measurement: y_x = hand(1)=roll = th_ix + a1x + a2x; y_y = hand(2)=pitch = th_iy + a1y + a2y.
Hmeas=zeros(2,n);Hmeas(1,[idx.thx idx.a1x idx.a2x])=1;Hmeas(2,[idx.thy idx.a1y idx.a2y])=1;
if osc2;Hmeas(1,idx.a3x)=1;Hmeas(2,idx.a3y)=1;end
R=C.ekf_r_meas*eye(2);

x0=zeros(n,1);x0(idx.omega)=2*pi*C.ekf_omega_init_Hz;
P0=1e-4*eye(n);
for k=1:numel(h2Ix);P0(h2Ix(k),h2Ix(k))=1e-8;end  % tight h2 prior, breaks the omega/2 degeneracy
P0(idx.omega,idx.omega)=(2*pi*C.ekf_omega_p0_Hz)^2;
if osc2
 % Seeded away from omega_init so the two oscillators do not start degenerate.
 % Each axis gets its OWN second-oscillator frequency: in the 'mixed' and
 % 'irregular' scenarios the unmodelled content is ~10-12 Hz on roll but 3 Hz
 % on pitch, and a single shared omega2 can only ever serve one of them.
 x0(idx.omega2x)=2*pi*C.ekf_omega2_init_Hz;
 x0(idx.omega2y)=2*pi*C.ekf_omega2_init_Hz;
 P0(idx.omega2x,idx.omega2x)=(2*pi*C.ekf_omega2_p0_Hz)^2;
 P0(idx.omega2y,idx.omega2y)=(2*pi*C.ekf_omega2_p0_Hz)^2;
end

% --- Packed-state layout -----------------------------------------------------
% The EKF carries all of its own history in one packed vector so the step
% function stays a pure function of (s,hand,C). Layout: x(n), P(:)(n*n),
% nis_bar, then the osc2 presence-gate state when C.ekf_osc2 is set.
ixNis=n+n*n+1;                       % 183 for the base 13-state filter
sLen=ixNis;
% Presence-gate state (osc2 only): 2-pole lowpass (2 stages x 2 axes) plus one
% EWMA band-power accumulator per axis.
ixGateLp=[];ixGatePow=[];
if osc2
 ixGateLp=ixNis+(1:4);ixGatePow=ixGateLp(end)+(1:2);sLen=ixGatePow(end);
end
% Second-oscillator bounds, defaulted so a C built without the ekf_osc2 block
% (older callers, hand-built test fixtures) still initialises cleanly.
om2min=2*pi*2;om2max=2*pi*25;om2rate=50;
gateHi=4.5;gateTau=.25;gateFloorDeg=.08;gateSdDeg=.004;
q3xIx=[];q3yIx=[];om2Ix=[];
if osc2
 q3xIx=[idx.a3x idx.b3x];q3yIx=[idx.a3y idx.b3y];
 om2Ix=[idx.omega2x idx.omega2y];
end
if isfield(C,'ekf_gate_hi_Hz');gateHi=C.ekf_gate_hi_Hz;end
if isfield(C,'ekf_gate_tau_s');gateTau=C.ekf_gate_tau_s;end
if isfield(C,'ekf_gate_floor_deg');gateFloorDeg=C.ekf_gate_floor_deg;end
if isfield(C,'ekf_gate_sd_deg');gateSdDeg=C.ekf_gate_sd_deg;end
if isfield(C,'ekf_omega2_min_Hz');om2min=2*pi*C.ekf_omega2_min_Hz;end
if isfield(C,'ekf_omega2_max_Hz');om2max=2*pi*C.ekf_omega2_max_Hz;end
if isfield(C,'ekf_omega2_rate_Hz_per_s');om2rate=C.ekf_omega2_rate_Hz_per_s;end

s0=zeros(sLen,1);s0(1:n)=x0;s0(n+1:n+n*n)=P0(:);s0(ixNis)=C.ekf_nis_lo;

C.ekf=struct('idx',idx,'Q',Q,'H',Hmeas,'R',R,'x0',x0,'P0',P0,...
 'omega_min',2*pi*C.ekf_omega_min_Hz,'omega_max',2*pi*C.ekf_omega_max_Hz,...
 'harmonics',C.ekf_harmonics,'n',n,'osc2',osc2,...
 'omega2_min',om2min,'omega2_max',om2max,'omega2_rate',om2rate,...
 'ix_nis',ixNis,...
 'ix_gate_lp',ixGateLp,'ix_gate_pow',ixGatePow,...
 'gate_pole',exp(-2*pi*gateHi/C.outer_hz),...
 'gate_alpha',1-exp(-1/(C.outer_hz*gateTau)),...
 'gate_floor',deg2rad(gateFloorDeg)^2,'gate_r',deg2rad(gateSdDeg)^2,...
 'q3x_ix',q3xIx,'q3y_ix',q3yIx,'om2_ix',om2Ix,'s_len',sLen,'s0',s0);
end
