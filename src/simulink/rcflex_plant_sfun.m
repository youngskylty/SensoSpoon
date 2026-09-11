function rcflex_plant_sfun(block)
% Transparent continuous-state electromechanical plant, integrated by Simulink.
block.NumDialogPrms=3;block.NumInputPorts=2;block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic;block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=15;block.InputPort(1).DirectFeedthrough=true;
block.InputPort(2).Dimensions=2;block.InputPort(2).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=26;block.NumContStates=8;
block.SampleTimes=[0 0];block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('InitializeConditions',@init);block.RegBlockMethod('Outputs',@out);block.RegBlockMethod('Derivatives',@deriv);
end
function init(b);b.ContStates.Data=zeros(8,1);end
function out(b)
base=b.InputPort(1).Data;
[~,y]=rcflex_physics(b.ContStates.Data,[base(1:6);b.InputPort(2).Data;base(7:15)],b.DialogPrm(1).Data,b.DialogPrm(2).Data,b.DialogPrm(3).Data);b.OutputPort(1).Data=y;
end
function deriv(b)
base=b.InputPort(1).Data;
b.Derivatives.Data=rcflex_physics(b.ContStates.Data,[base(1:6);b.InputPort(2).Data;base(7:15)],b.DialogPrm(1).Data,b.DialogPrm(2).Data,b.DialogPrm(3).Data);
end
