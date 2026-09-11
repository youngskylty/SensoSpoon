function rcflex_current_sfun(b)
b.NumDialogPrms=2;b.NumInputPorts=2;b.NumOutputPorts=1;
b.SetPreCompInpPortInfoToDynamic;b.SetPreCompOutPortInfoToDynamic;
b.InputPort(1).Dimensions=3;b.InputPort(2).Dimensions=4;
b.InputPort(1).DirectFeedthrough=true;b.InputPort(2).DirectFeedthrough=true;
b.OutputPort(1).Dimensions=2;C=b.DialogPrm(2).Data;b.SampleTimes=[1/C.current_hz 0];
b.RegBlockMethod('PostPropagationSetup',@setup);b.RegBlockMethod('InitializeConditions',@init);
b.RegBlockMethod('Outputs',@out);b.RegBlockMethod('Update',@update);
end
function setup(b)
b.NumDworks=1;b.Dwork(1).Name='PI_integral';b.Dwork(1).Dimensions=2;
b.Dwork(1).DatatypeID=0;b.Dwork(1).Complexity='Real';b.Dwork(1).UsedAsDiscState=true;
end
function init(b);b.Dwork(1).Data=zeros(2,1);end
function out(b)
[~,v]=rcflex_current_step(b.Dwork(1).Data,b.InputPort(1).Data,b.InputPort(2).Data,b.DialogPrm(1).Data,b.DialogPrm(2).Data);b.OutputPort(1).Data=v;
end
function update(b)
b.Dwork(1).Data=rcflex_current_step(b.Dwork(1).Data,b.InputPort(1).Data,b.InputPort(2).Data,b.DialogPrm(1).Data,b.DialogPrm(2).Data);
end
