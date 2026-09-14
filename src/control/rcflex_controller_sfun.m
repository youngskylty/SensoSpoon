function rcflex_controller_sfun(b)
% Sensor-only discrete outer loop; output uses previous stored state.
b.NumDialogPrms=3;b.NumInputPorts=1;b.NumOutputPorts=2;
b.SetPreCompInpPortInfoToDynamic;b.SetPreCompOutPortInfoToDynamic;
b.InputPort(1).Dimensions=14;b.InputPort(1).DirectFeedthrough=true;
b.OutputPort(1).Dimensions=4;b.OutputPort(2).Dimensions=32;
C=b.DialogPrm(3).Data;b.SampleTimes=[1/C.outer_hz 0];
b.RegBlockMethod('PostPropagationSetup',@setup);b.RegBlockMethod('InitializeConditions',@init);
b.RegBlockMethod('Outputs',@out);b.RegBlockMethod('Update',@update);
end
function setup(b)
% Width is derived from C, not hard-coded, so the Dwork cannot be narrower
% than the state rcflex_attitude_step actually writes when an optional
% subsystem (C.ekf_osc2, C.semg_enabled) widens it.
b.NumDworks=1;b.Dwork(1).Name='estimator_and_integral';b.Dwork(1).Dimensions=state_width(b.DialogPrm(3).Data);
b.Dwork(1).DatatypeID=0;b.Dwork(1).Complexity='Real';b.Dwork(1).UsedAsDiscState=true;
end
function init(b)
C=b.DialogPrm(3).Data;
% Simulink has no sEMG port yet, so this path always runs the IMU-only
% baseline: rcflex_attitude_step is called with five arguments and its sEMG
% diagnostic slots read zero. Wiring the sEMG stream into the model is the
% follow-up step, deferred until the characterisation says what to fuse.
% Must mirror rcflex_attitude_step's own z=[] initialisation exactly; the
% packed EKF initial slice is built once in rcflex_intent_ekf_init (C.ekf.s0)
% so the two sites cannot drift apart.
gammaIx=328;
if isfield(C,'ekf')&&isfield(C.ekf,'s_len');gammaIx=max(328,100+C.ekf.s_len+1);end
z=zeros(state_width(C),1);z(1:18)=[reshape(eye(3),9,1);reshape(eye(3),9,1)];z(91:99)=reshape(eye(3),9,1);z(gammaIx)=1;
if isfield(C,'algorithm')&&strcmp(C.algorithm,'ekf_predictive')
 z(100+(1:C.ekf.s_len))=C.ekf.s0;
end
b.Dwork(1).Data=z;
end
function out(b)
[~,c,d]=rcflex_attitude_step(b.Dwork(1).Data,b.InputPort(1).Data,b.DialogPrm(1).Data,b.DialogPrm(2).Data,b.DialogPrm(3).Data);
b.OutputPort(1).Data=c;b.OutputPort(2).Data=d;
end
function update(b)
b.Dwork(1).Data=rcflex_attitude_step(b.Dwork(1).Data,b.InputPort(1).Data,b.DialogPrm(1).Data,b.DialogPrm(2).Data,b.DialogPrm(3).Data);
end
function nz=state_width(C)
% Single definition of the controller state width, shared by setup and init.
% Must agree with the nz computed inside rcflex_attitude_step.
gammaIx=328;
if isfield(C,'ekf')&&isfield(C.ekf,'s_len');gammaIx=max(328,100+C.ekf.s_len+1);end
nz=max(400,gammaIx);
if isfield(C,'semg_enabled')&&C.semg_enabled&&isfield(C,'semg')
 nz=max(nz,gammaIx+C.semg.s_len);
end
end
