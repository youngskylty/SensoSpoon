function C=rcflex_intent_ekf_init(C)
% Build the constant matrices and index map for the 13-state joint intent/
% tremor EKF (docs/CONTROL_IMPROVEMENT_PLAN.md). Called once from
% rcflex_prepare_control before the controller starts; the per-step
% Jacobian only ever needs the omega column recomputed (rcflex_intent_ekf_step).
%
% State: [th_ix thd_ix th_iy thd_iy a1x b1x a2x b2x a1y b1y a2y b2y omega]
n=13;
idx=struct('thx',1,'thdx',2,'thy',3,'thdy',4,...
 'a1x',5,'b1x',6,'a2x',7,'b2x',8,'a1y',9,'b1y',10,'a2y',11,'b2y',12,'omega',13);

Q=zeros(n);
Q(idx.thx,idx.thx)=1e-10;Q(idx.thdx,idx.thdx)=C.ekf_q_intent;   % intent, axis x (roll)
Q(idx.thy,idx.thy)=1e-10;Q(idx.thdy,idx.thdy)=C.ekf_q_intent;   % intent, axis y (pitch)
h1Ix=[idx.a1x idx.b1x idx.a1y idx.b1y];h2Ix=[idx.a2x idx.b2x idx.a2y idx.b2y];
for k=1:numel(h1Ix);Q(h1Ix(k),h1Ix(k))=C.ekf_q_tremor;end
for k=1:numel(h2Ix);Q(h2Ix(k),h2Ix(k))=C.ekf_q_tremor2;end
Q(idx.omega,idx.omega)=C.ekf_q_omega;

% Measurement: y_x = hand(1)=roll = th_ix + a1x + a2x; y_y = hand(2)=pitch = th_iy + a1y + a2y.
Hmeas=zeros(2,n);Hmeas(1,[idx.thx idx.a1x idx.a2x])=1;Hmeas(2,[idx.thy idx.a1y idx.a2y])=1;
R=C.ekf_r_meas*eye(2);

x0=zeros(n,1);x0(idx.omega)=2*pi*C.ekf_omega_init_Hz;
P0=1e-4*eye(n);
for k=1:numel(h2Ix);P0(h2Ix(k),h2Ix(k))=1e-8;end  % tight h2 prior, breaks the omega/2 degeneracy
P0(idx.omega,idx.omega)=(2*pi*C.ekf_omega_p0_Hz)^2;

% --- Independent frequency observable (see rcflex_intent_ekf_step) ---------
% A lag-D autocorrelation estimator running on a high-passed copy of the
% measurement. Its state is carried in the same packed vector as the EKF so
% the step function stays a pure function of (s,hand,C).
D=C.freq_lag_samples;histLen=2*D+1;
ixNis=n+n*n+1;                       % 183
ixHpY=ixNis+(1:4);                   % 184:187  (2 stages x 2 axes)
ixHpU=ixNis+(5:8);                   % 188:191
ixHist=ixHpU(end)+(1:2*histLen);     % 192:225
ixNum=ixHist(end)+1;ixDen=ixNum+1;   % 226, 227
sLen=ixDen;
s0=zeros(sLen,1);s0(1:n)=x0;s0(n+1:n+n*n)=P0(:);s0(ixNis)=C.ekf_nis_lo;

C.ekf=struct('idx',idx,'Q',Q,'H',Hmeas,'R',R,'x0',x0,'P0',P0,...
 'omega_min',2*pi*C.ekf_omega_min_Hz,'omega_max',2*pi*C.ekf_omega_max_Hz,...
 'harmonics',C.ekf_harmonics,'n',n,...
 'freq_lag',D,'freq_hist_len',histLen,...
 'freq_hp_pole',exp(-2*pi*C.freq_hp_Hz/C.outer_hz),...
 'freq_alpha',1-exp(-1/(C.outer_hz*C.freq_tau_s)),...
 'freq_power_floor',deg2rad(C.freq_power_floor_deg)^2,...
 'freq_r',(2*pi*C.freq_meas_sd_Hz)^2,...
 'ix_nis',ixNis,'ix_hp_y',ixHpY,'ix_hp_u',ixHpU,'ix_hist',ixHist,...
 'ix_num',ixNum,'ix_den',ixDen,'s_len',sLen,'s0',s0);
end
