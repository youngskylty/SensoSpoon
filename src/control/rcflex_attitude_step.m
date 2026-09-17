function [z,command,diagnostic]=rcflex_attitude_step(z,sensor,H,B,C,semgBlock)
% Sensor-only outer loop. No plant truth, base input or future samples.
% command=[current references(2); estimated back-EMF(2)].
%
% semgBlock is OPTIONAL: C.semg.sub x C.semg.nch electrode-referred sEMG
% samples covering this tick, or omitted/[] for the IMU-only baseline. It is a
% trailing argument rather than an extension of `sensor` deliberately -- that
% keeps the 14-wide sensor vector, and therefore every Simulink port width and
% every existing call site, completely unchanged. The features it produces are
% published to the diagnostic vector for characterisation and are NOT read by
% the reference path: sEMG is sensed here, not yet fused.
% State: rotations 1:18; integral 23:24; power/heat 38:41;
% guarded joints 52:53; IIR sections 61:88 (C.algorithm='simple_pid' only);
% previous desired rotation 91:99; EKF slice 101:100+C.ekf.s_len (x(13)
% 101:113, P(:)(169) 114:282, nis_bar 283 for the base 13-state filter,
% C.algorithm='ekf_predictive' only); authority manager gamma_auth at
% 100+C.ekf.s_len+1, floored at 328 so the base layout is unchanged; sEMG
% preprocessing state immediately after gamma_auth (C.semg_enabled only).
if nargin<6;semgBlock=[];end
dt=1/C.outer_hz;
% The EKF slice starts at 101 and runs C.ekf.s_len elements, so gamma_auth must
% sit immediately AFTER it rather than at a hard-coded index: the widened
% (C.ekf_osc2) filter is longer than the base one and would otherwise overwrite
% gamma_auth every step, corrupting the authority manager. The max(328,...)
% floor keeps gamma_auth where it has always been for the base filter.
gammaIx=328;
if isfield(C,'ekf')&&isfield(C.ekf,'s_len');gammaIx=max(328,100+C.ekf.s_len+1);end
semgOn=isfield(C,'semg_enabled')&&C.semg_enabled&&isfield(C,'semg');
% C.semg_fusion is the ONLY switch selecting the fusion arm. 'off' leaves the
% z layout, the EKF call and every number downstream exactly as they were.
%
% Read INDEPENDENTLY of C.semg_enabled. The 'intent_gate_fixed' ablation is a
% change to the EKF alone and must run with no sEMG subsystem at all -- that is
% exactly what makes it an IMU-only control group. Gating the switch on
% semg_enabled silently turned that arm back into the baseline and made the
% ablation look identical to the control.
fuseMode='off';
if isfield(C,'semg_fusion')&&~isempty(C.semg_fusion);fuseMode=C.semg_fusion;end
% Two families of fusion arm exist and are mutually exclusive:
%   omega arms  ('semg','oracle')        -- experiment E1, frequency observable
%                                           on EKF state 13. Kept for the record;
%                                           its result was negative.
%   intent arms ('intent_gate','intent_gate_fixed') -- experiment E2, voluntary-
%                                           drive confidence scaling the intent
%                                           process noise. 'intent_gate' is the
%                                           shipped sEMG+IMU system;
%                                           'intent_gate_fixed' is its IMU-only
%                                           ablation, which applies the same
%                                           mechanism at a CONSTANT confidence
%                                           and so needs no sEMG at all.
omegaOn=any(strcmp(fuseMode,{'semg','oracle'}));
gateOn=any(strcmp(fuseMode,{'intent_gate','intent_gate_fixed'}));
if ~strcmp(fuseMode,'off')&&~omegaOn&&~gateOn
 error('SensoSpoon:SemgFusion','Unknown C.semg_fusion: %s',fuseMode);
end
% The fixed-confidence ablation is an EKF-only change, so it must run whether or
% not the sEMG subsystem is enabled -- that is what makes it an IMU-only control.
gateStateOn=gateOn&&semgOn&&~strcmp(fuseMode,'intent_gate_fixed');
% Arms that need the stream must say so loudly rather than degrading into the
% baseline, which is how the first run of the ablation went unnoticed.
assert(~any(strcmp(fuseMode,{'semg','intent_gate'}))||semgOn,...
 'SensoSpoon:SemgFusion','C.semg_fusion=''%s'' needs C.semg_enabled=true.',fuseMode);
semgIx=[];fuseIx=[];gateIx=[];
if semgOn;semgIx=gammaIx+(1:C.semg.s_len);end
if omegaOn&&semgOn;fuseIx=semgIx(end)+(1:C.semg.s_len_fuse);end
if gateStateOn
 base=semgIx;if ~isempty(fuseIx);base=fuseIx;end
 gateIx=base(end)+(1:C.semg.s_len_gate);
end
nz=max(400,gammaIx);
if semgOn;nz=max(nz,semgIx(end));end
if ~isempty(fuseIx);nz=max(nz,fuseIx(end));end
if ~isempty(gateIx);nz=max(nz,gateIx(end));end
if isempty(z)
 z=zeros(nz,1);z(1:18)=[reshape(eye(3),9,1);reshape(eye(3),9,1)];z(91:99)=reshape(eye(3),9,1);z(gammaIx)=1;
 if isfield(C,'algorithm')&&strcmp(C.algorithm,'ekf_predictive')
  z(100+(1:C.ekf.s_len))=C.ekf.s0;
 end
 if semgOn;z(semgIx)=C.semg.s0;end
 if ~isempty(fuseIx);z(fuseIx)=C.semg.s0_fuse;end
 if ~isempty(gateIx);z(gateIx)=C.semg.s0_gate;end
end
% sEMG preprocessing runs before the reference branch because the reference
% branch now depends on it: under C.semg_fusion='intent_gate' the gate below
% reads semgFeat(1:nch), the envelope features, and its confidence scales the
% EKF's intent process noise. On every other arm semgFeat still goes only to
% the diagnostic vector and changes nothing downstream.
semgFeat=zeros(8,1);
if semgOn&&~isempty(semgBlock)
 [semgState,feat]=rcflex_semg_preprocess(z(semgIx),semgBlock,C);
 z(semgIx)=semgState;semgFeat=feat;
end
% Frequency observable for the EKF omega state. semgOmega stays [] in every
% path except an explicitly selected fusion arm, and the EKF step treats [] as
% "no pseudo-measurement", so the IMU-only result is untouched.
semgOmega=[];semgOmegaHz=0;semgConf=0;
if omegaOn
 switch fuseMode
  case 'semg'
   if ~isempty(semgBlock)
    [fuseState,wHz,cf]=rcflex_semg_omega(z(fuseIx),semgFeat(C.semg.nch+1:end),C);
    z(fuseIx)=fuseState;semgOmegaHz=wHz;semgConf=cf;
    semgOmega=[2*pi*wHz;cf];
   end
  case 'oracle'
   % Diagnostic upper bound: reads simulation ground truth through C, so it is
   % NOT a realisable mode. Its only job is to separate "the observable is
   % poor" from "omega was never the binding constraint".
   semgOmegaHz=C.semg_fusion_oracle_Hz;semgConf=1;
   semgOmega=[2*pi*semgOmegaHz;1];
 end
end
% Voluntary-drive confidence for the EKF intent process noise (experiment E2).
% intentConf stays [] on every path except an explicitly selected intent arm,
% and the EKF step treats [] as "leave Q alone", so the IMU-only results are
% untouched.
intentConf=[];
if gateOn
 if strcmp(fuseMode,'intent_gate_fixed')
  % IMU-only ablation: the same Q scaling held at a constant. This is the
  % control that shows what the MECHANISM achieves without the SENSOR, and
  % therefore what the sEMG stream is actually worth.
  intentConf=C.semg_gate_fixed_conf*[1;1];
 elseif ~isempty(gateIx)&&~isempty(semgBlock)
  [gateState,intentConf]=rcflex_semg_intent_gate(z(gateIx),semgFeat(1:C.semg.nch),C);
  z(gateIx)=gateState;
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
 ekfIx=100+(1:C.ekf.s_len);
 [ekfSlice,refIntent,ekfInfo]=rcflex_intent_ekf_step(z(ekfIx),hand,C,semgOmega,intentConf);
 z(ekfIx)=ekfSlice;
 gammaTrust=ekfInfo(3);handRaw=hand([2 1]);
 blended=gammaTrust*refIntent+(1-gammaTrust)*handRaw;
 [gammaAuth,iPred]=rcflex_authority_manager(z(gammaIx),hand,ekfSlice(1:13),H,B,C);
 z(gammaIx)=gammaAuth;
 ref=handRaw+gammaAuth*(blended-handRaw);
 % Slots 6:7 carry WHICHEVER sEMG observable the active fusion arm produces, so
 % the diagnostic stays 32 wide and no Simulink port width changes:
 %   'semg'/'oracle'  -> [tremor frequency (Hz), burst-power confidence]
 %   'intent_gate*'   -> [roll voluntary-drive confidence, pitch confidence]
 % Both read zero when C.semg_fusion='off'. The intent confidences invert to
 % the underlying mean-square drive as P = gate_floor*conf^(1/exponent) /
 % (1-conf^(1/exponent)), so nothing is lost by publishing the confidence only.
 semgObs=[semgOmegaHz;semgConf];
 if ~isempty(intentConf);semgObs=intentConf(:);end
 ekfDiag=[ekfInfo(1);ekfInfo(2);gammaTrust;gammaAuth;iPred;semgObs;0];
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
% Width 32: 16 baseline + 8 EKF/authority + 8 sEMG. Both trailing groups read
% zero when their subsystem is off, so downstream indexing is unaffected.
diagnostic=[q;ref;e;iref;limit;z(38:41);double(sat);cond(L.J);ekfDiag;semgFeat];
end
function R=increment(v)
t=norm(v);K=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
if t<1e-9;R=eye(3)+K;else;R=eye(3)+sin(t)/t*K+(1-cos(t))/t^2*K*K;end
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end

