function build_rcflex_analytic()
% Build a continuous-state simulation. No spoon joint motion is prescribed.
mdl='SensoSpoon_Analytic';if bdIsLoaded(mdl);close_system(mdl,0);end
file=project_path('models',[mdl '.slx']);
if isfile(file)
 load_system(file);Simulink.BlockDiagram.deleteContents(mdl);
else
 new_system(mdl);
end
H=rcflex_parameters();B=rcflex_body_properties(H);
w=get_param(mdl,'ModelWorkspace');w.assignin('H',H);w.assignin('B',B);w.assignin('mode','on');
w.assignin('base_motion',timeseries(zeros(2,15),[0;H.duration]));
w.assignin('voltage_command',timeseries(zeros(2,2),[0;H.duration]));
add_block('simulink/Sources/From Workspace',[mdl '/Base_motion'],'VariableName','base_motion','Position',[50 60 230 100]);
add_block('simulink/Sources/From Workspace',[mdl '/Voltage_A_B'],'VariableName','voltage_command','Position',[50 150 230 190]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[mdl '/Electromechanical_plant'], ...
    'FunctionName','rcflex_plant_sfun','Parameters','H,B,mode','Position',[310 80 560 185]);
add_block('simulink/Sinks/To Workspace',[mdl '/Physical_states_and_loads'],'VariableName','analytic_log','SaveFormat','Timeseries','Position',[665 105 865 155]);
add_line(mdl,'Base_motion/1','Electromechanical_plant/1');add_line(mdl,'Voltage_A_B/1','Electromechanical_plant/2');add_line(mdl,'Electromechanical_plant/1','Physical_states_and_loads/1');
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[mdl '/BMI270_pair_and_current_ADC'], ...
    'FunctionName','rcflex_sensor_sfun','Parameters','H','Position',[650 300 895 360]);
add_block('simulink/Sinks/To Workspace',[mdl '/Sensor_readings'],'VariableName','sensor_log','SaveFormat','Timeseries','Position',[970 310 1120 350]);
add_line(mdl,'Electromechanical_plant/1','BMI270_pair_and_current_ADC/1');
add_line(mdl,'Base_motion/1','BMI270_pair_and_current_ADC/2');
add_line(mdl,'BMI270_pair_and_current_ADC/1','Sensor_readings/1');
a=Simulink.Annotation(mdl,sprintf('RC-Flex plant ONLY: no stabilisation algorithm\nVoltage -> RL + back EMF -> current -> force -> exact rod Jacobian -> rigid-body dynamics\nStates: pitch, roll, rates, currents, coil temperatures\nMoticont GVCM-016-019-01M x2 / DRV8876 averaged bridges\nGravity, moving-coil inertia, flexures, bearing friction and compliant stops.'));
a.Position=[45 430];
set_param(mdl,'Solver','ode15s','MaxStep','.001','RelTol','1e-5','AbsTol','1e-7', ...
    'StopTime',num2str(H.duration),'ReturnWorkspaceOutputs','on');
save_system(mdl,project_path('models',[mdl '.slx']));close_system(mdl,0);
end
