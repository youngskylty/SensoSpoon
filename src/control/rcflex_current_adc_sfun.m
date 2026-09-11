function rcflex_current_adc_sfun(b)
b.NumDialogPrms=2;b.NumInputPorts=1;b.NumOutputPorts=1;
b.SetPreCompInpPortInfoToDynamic;b.SetPreCompOutPortInfoToDynamic;
b.InputPort(1).Dimensions=26;b.InputPort(1).DirectFeedthrough=false;b.OutputPort(1).Dimensions=3;
C=b.DialogPrm(2).Data;b.SampleTimes=[1/C.current_hz 0];
b.RegBlockMethod('PostPropagationSetup',@setup);b.RegBlockMethod('InitializeConditions',@init);
b.RegBlockMethod('Outputs',@out);b.RegBlockMethod('Update',@update);
end
function setup(b)
b.NumDworks=1;b.Dwork(1).Name='delayed_ADC';b.Dwork(1).Dimensions=3;
b.Dwork(1).DatatypeID=0;b.Dwork(1).Complexity='Real';b.Dwork(1).UsedAsDiscState=true;
end
function init(b);H=b.DialogPrm(1).Data;b.Dwork(1).Data=[0;0;H.bus_voltage_V];end
function out(b);b.OutputPort(1).Data=b.Dwork(1).Data;end
function update(b)
x=b.InputPort(1).Data;b.Dwork(1).Data=rcflex_current_sample(x(5:6),b.CurrentTime,b.DialogPrm(1).Data,b.DialogPrm(2).Data);
end
