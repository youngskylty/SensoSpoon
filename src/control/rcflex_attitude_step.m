function [z,command,diagnostic]=rcflex_attitude_step(z,sensor,H,B,C)
% Sensor-only outer loop. No plant truth, base input or future samples.
% command=[current references(2); estimated back-EMF(2)].
% State: rotations 1:18; integral 23:24; power/heat 38:41;
% guarded joints 52:53; IIR sections 61:88 (C.algorithm='simple_pid' only);
% previous desired rotation 91:99; EKF slice 101:283 (x(13) 101:113,
% P(:)(169) 114:282, nis_bar 283, C.algorithm='ekf_predictive' only);
% authority manager gamma_auth 284.
dt=1/C.outer_hz;
if isempty(z)
 z=zeros(300,1);z(1:18)=[reshape(eye(3),9,1);reshape(eye(3),9,1)];z(91:99)=reshape(eye(3),9,1);z(284)=1;
 if isfield(C,'algorithm')&&strcmp(C.algorithm,'ekf_predictive')
  z(101:283)=[C.ekf.x0;C.ekf.P0(:);C.ekf_nis_lo];
 end
end
R=cell(2,1);omega=zeros(3,2);
for j=1:2
 ix=(j-1)*9+(1:9);R{j}=reshape(z(ix),3,3);
 gyro=sensor((j-1)*6+(1:3))-C.gyro_bias((j-1)*3+(1:3));
 acc=sensor((j-1)*6+(4:6))-C.accel_bias((j-1)*3+(1:3));
 trust=max(0,1-abs(norm(acc)-9.80665)/(0.12*9.80665));
 if norm(acc)>1;gyro=gyro+2*pi*C.complement_hz*trust*cross(acc/norm(acc),R{j}'*[0;0;1]);end
 R{j}=R{j}*increment(gyro*dt);z(ix)=R{j}(:);omega(:,j)=R{j}*gyro;
end
Rh=R{1};Rs=R{2};rel=Rh'*Rs;
q=[atan2(-rel(3,1),rel(1,1));atan2(-rel(2,3),rel(2,2))];
hand=[atan2(Rh(3,2),Rh(2,2));atan2(Rh(1,3),Rh(1,1))];
% Reference-path branch -- the only place the intent/tremor algorithm
% choice appears. Everything before this point (attitude) and after it
% (guard, error, PID, feedforward, allocator) is common to both algorithms.
if strcmp(C.algorithm,'ekf_predictive')
 [ekfSlice,refIntent,ekfInfo]=rcflex_intent_ekf_step(z(101:283),hand,C);
 z(101:283)=ekfSlice;
 gammaTrust=ekfInfo(3);handRaw=hand([2 1]);
 blended=gammaTrust*refIntent+(1-gammaTrust)*handRaw;
 [gammaAuth,iPred]=rcflex_authority_manager(z(284),hand,ekfSlice(1:13),H,B,C);
 z(284)=gammaAuth;
 ref=handRaw+gammaAuth*(blended-handRaw);
 ekfDiag=[ekfInfo(1);ekfInfo(2);gammaTrust;gammaAuth;iPred;0;0;0];
else
 % Causal fixed IIR sections. Coefficients are prepared before execution.
 ref=hand([2 1]);
 for j=1:size(C.intent_sos,1)
  c=C.intent_sos(j,:);ix=60+4*(j-1)+(1:2);input=ref;
  ref=c(1)*input+z(ix);z(ix)=c(2)*input-c(5)*ref+z(ix+2);z(ix+2)=c(3)*input-c(6)*ref;
 end
 ekfDiag=zeros(8,1);
end
Rd=rx(ref(2))*ry(ref(1));rr=Rh'*Rd;
qr=[atan2(-rr(3,1),rr(1,1));atan2(-rr(2,3),rr(2,2))];
qr=max(min(qr,C.joint_limit),-C.joint_limit);
qr=z(52:53)+max(min(qr-z(52:53),C.reference_rate*dt),-C.reference_rate*dt);z(52:53)=qr;
Rd=Rh*ry(qr(1))*rx(qr(2));
errMat=(Rd'*Rs-Rs'*Rd)/2;err=Rd*[errMat(3,2);errMat(1,3);errMat(2,1)];
% Desired rate must follow the SAME guarded orientation, including clipping.
previousRd=reshape(z(91:99),3,3);W=(Rd-previousRd)/dt*Rd';W=(W-W')/2;
omegaDesired=[W(3,2);W(1,3);W(2,1)];z(91:99)=Rd(:);
axisMap=[Rh*[0;1;0],Rh*ry(q(1))*[1;0;0]];
qd=axisMap'*(omega(:,2)-omega(:,1));
e=axisMap'*err;rateErr=axisMap'*(omega(:,2)-omegaDesired);
[M,gravity]=rcflex_mass_bias(q,zeros(2,1),hand,zeros(2,1),zeros(2,1),H,B);
previousIntegral=z(23:24);z(23:24)=max(min(previousIntegral+dt*e,.02),-.02);
wn=2*pi*C.natural_hz;
accel=-wn^2*e-2*C.damping*wn*rateErr-C.integral_gain*z(23:24);
torque=M*accel+gravity+H.flexure_stiffness_Nm_rad.*q+H.flexure_damping_Nms_rad*qd;
L=rcflex_linkage(q,H);sd=L.J*qd;
K=H.vca_force_constant_N_A*max(1-.27*(L.s/(H.vca_stroke_mm/1000)).^2,.5);
friction=H.guide_friction_N*tanh(sd/.001)+H.guide_viscous_Ns_m*sd;
target=torque+L.J'*friction;
% Limits use current feedback and estimated thermal state, never truth.
current=sensor(13:14);power=H.coil_R_ohm*(1+H.coil_alpha_K*z(40:41)).*current.^2;
z(38:39)=z(38:39)+(1-exp(-dt/C.power_window_s))*(power-z(38:39));
z(40:41)=max(0,z(40:41)+dt*(power-z(40:41)/H.thermal_resistance_K_W)/H.thermal_capacity_J_K);
limit=.98*H.current_limit_A*ones(2,1);
continuous=sqrt(H.coil_continuous_power_W./(H.coil_R_ohm*(1+H.coil_alpha_K*z(40:41))));
limit(z(38:39)>H.coil_continuous_power_W)=continuous(z(38:39)>H.coil_continuous_power_W);
d=L.slider-L.attach;ratio=vecnorm(d(2:3,:))./abs(d(1,:));
limit=min(limit,.95*H.shaft_side_load_N./max(ratio(:).*K,eps));
% Enumerate the nine active sets of a two-variable bounded least squares.
A=M\(L.J'*diag(K));b=M\target;best=inf;iref=zeros(2,1);
for a1=-1:1
 for a2=-1:1
  active=[a1;a2];fixed=active~=0;free=~fixed;v=active.*limit;
  if any(free);v(free)=A(:,free)\(b-A(:,fixed)*v(fixed));end
  if any(abs(v)>limit+1e-10);continue;end
  cost=norm(A*v-b)^2+1e-6*sum(v.^2);
  if cost<best;best=cost;iref=v;end
 end
end
sat=norm(A*iref-b)>1;
if sat;z(23:24)=previousIntegral;end
command=[iref;K.*sd];
diagnostic=[q;ref;e;iref;limit;z(38:41);double(sat);cond(L.J);ekfDiag];
end
function R=increment(v)
t=norm(v);K=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
if t<1e-9;R=eye(3)+K;else;R=eye(3)+sin(t)/t*K+(1-cos(t))/t^2*K*K;end
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end

