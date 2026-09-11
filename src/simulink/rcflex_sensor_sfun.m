function rcflex_sensor_sfun(b)
% Sampled BMI270 pair and current ADC. Noise/bias are explicit design scenarios.
b.NumDialogPrms=1;b.NumInputPorts=2;b.NumOutputPorts=1;
b.SetPreCompInpPortInfoToDynamic;b.SetPreCompOutPortInfoToDynamic;
b.InputPort(1).Dimensions=26;b.InputPort(2).Dimensions=15;
b.InputPort(1).DirectFeedthrough=false;b.InputPort(2).DirectFeedthrough=false;
b.OutputPort(1).Dimensions=14;b.SampleTimes=[1/800 0];b.SimStateCompliance='DefaultSimState';
b.RegBlockMethod('PostPropagationSetup',@setup);b.RegBlockMethod('InitializeConditions',@init);
b.RegBlockMethod('Outputs',@out);b.RegBlockMethod('Update',@update);
end
function setup(b)
b.NumDworks=2;b.Dwork(1).Name='sampled_measurement';b.Dwork(1).Dimensions=14;
b.Dwork(1).DatatypeID=0;b.Dwork(1).Complexity='Real';b.Dwork(1).UsedAsDiscState=true;
b.Dwork(2).Name='filtered_imu';b.Dwork(2).Dimensions=12;b.Dwork(2).DatatypeID=0;b.Dwork(2).Complexity='Real';b.Dwork(2).UsedAsDiscState=true;
end
function init(b)
f=zeros(12,1);H=b.DialogPrm(1).Data;
if isfield(H,'sensor_seed');f=repmat([0;0;0;0;0;9.80665],2,1);end
b.Dwork(1).Data=[f;zeros(2,1)];b.Dwork(2).Data=f;
end
function out(b);b.OutputPort(1).Data=b.Dwork(1).Data;end
function update(b)
H=b.DialogPrm(1).Data;x=b.InputPort(1).Data;base=b.InputPort(2).Data;
y=rcflex_sensor_truth(x,base,H);seed=20260910;
if isfield(H,'sensor_seed');seed=H.sensor_seed;end
[measurement,filtered]=rcflex_imu_sample(y,b.Dwork(2).Data,b.CurrentTime,H,seed);
current=round(x(5:6)/(7/4096))*(7/4096);
b.Dwork(1).Data=[measurement;current];b.Dwork(2).Data=filtered;
end
