function rcflex_controller_sfun(b)
% Sensor-only discrete outer loop; output uses previous stored state.
b.NumDialogPrms=3;b.NumInputPorts=1;b.NumOutputPorts=2;
b.SetPreCompInpPortInfoToDynamic;b.SetPreCompOutPortInfoToDynamic;
b.InputPort(1).Dimensions=14;b.InputPort(1).DirectFeedthrough=true;
b.OutputPort(1).Dimensions=4;b.OutputPort(2).Dimensions=24;
C=b.DialogPrm(3).Data;b.SampleTimes=[1/C.outer_hz 0];
b.RegBlockMethod('PostPropagationSetup',@setup);b.RegBlockMethod('InitializeConditions',@init);
b.RegBlockMethod('Outputs',@out);b.RegBlockMethod('Update',@update);
end
function setup(b)
b.NumDworks=1;b.Dwork(1).Name='estimator_and_integral';b.Dwork(1).Dimensions=400;
b.Dwork(1).DatatypeID=0;b.Dwork(1).Complexity='Real';b.Dwork(1).UsedAsDiscState=true;
end
function init(b)
C=b.DialogPrm(3).Data;
% Must mirror rcflex_attitude_step's own z=[] initialisation exactly; the
% packed EKF initial slice is built once in rcflex_intent_ekf_init (C.ekf.s0)
% so the two sites cannot drift apart.
z=zeros(400,1);z(1:18)=[reshape(eye(3),9,1);reshape(eye(3),9,1)];z(91:99)=reshape(eye(3),9,1);z(328)=1;
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
