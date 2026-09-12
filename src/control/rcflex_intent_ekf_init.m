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

C.ekf=struct('idx',idx,'Q',Q,'H',Hmeas,'R',R,'x0',x0,'P0',P0,...
 'omega_min',2*pi*C.ekf_omega_min_Hz,'omega_max',2*pi*C.ekf_omega_max_Hz,...
 'harmonics',C.ekf_harmonics,'n',n);
end
