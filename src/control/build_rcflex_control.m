function build_rcflex_control()
% Reuse the authoritative continuous physical plant and sampled IMUs.
source=project_path('models','SensoSpoon_Analytic.slx');
if ~isfile(source);build_rcflex_analytic;end
mdl='SensoSpoon_Control';if bdIsLoaded(mdl);close_system(mdl,0);end
load_system(source);save_system('SensoSpoon_Analytic',project_path('models',[mdl '.slx']));
H=rcflex_parameters;H.sensor_seed=1701;Hnom=rcflex_parameters;Bnom=rcflex_body_properties(Hnom);C=rcflex_calibrate_control(rcflex_control_parameters,Hnom);
w=get_param(mdl,'ModelWorkspace');w.assignin('H',H);w.assignin('Hnom',Hnom);w.assignin('Bnom',Bnom);w.assignin('C',C);w.assignin('mode','voltage');
delete_line(mdl,'Voltage_A_B/1','Electromechanical_plant/2');delete_block([mdl '/Voltage_A_B']);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[mdl '/Attitude_PID_800Hz'], 'FunctionName','rcflex_controller_sfun','Parameters','Hnom,Bnom,C','Position',[980 430 1190 495]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[mdl '/Current_PI_10kHz'], 'FunctionName','rcflex_current_sfun','Parameters','Hnom,C','Position',[330 540 535 610]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[mdl '/Current_bus_ADC_10kHz'], 'FunctionName','rcflex_current_adc_sfun','Parameters','H,C','Position',[650 535 865 600]);
add_block('simulink/Sinks/To Workspace',[mdl '/Controller_diagnostics'],'VariableName','control_log','SaveFormat','Timeseries','Position',[1270 450 1430 490]);
add_line(mdl,'BMI270_pair_and_current_ADC/1','Attitude_PID_800Hz/1','autorouting','on');
add_line(mdl,'Attitude_PID_800Hz/1','Current_PI_10kHz/2','autorouting','on');
add_line(mdl,'Attitude_PID_800Hz/2','Controller_diagnostics/1','autorouting','on');
add_line(mdl,'Electromechanical_plant/1','Current_bus_ADC_10kHz/1','autorouting','on');
add_line(mdl,'Current_bus_ADC_10kHz/1','Current_PI_10kHz/1','autorouting','on');
add_line(mdl,'Current_PI_10kHz/1','Electromechanical_plant/2','autorouting','on');
annotations=find_system(mdl,'FindAll','on','Type','annotation');delete(annotations);
a=Simulink.Annotation(mdl,sprintf('Causal dual-IMU attitude PID (800 Hz) + current PI (10 kHz)\nContinuous RL / mechanical / thermal plant; sampled, delayed sensor feedback\nNo base-motion truth or intended-motion truth enters the controller.'));a.Position=[50 700];
save_system(mdl,project_path('models',[mdl '.slx']));close_system(mdl,0);
end
