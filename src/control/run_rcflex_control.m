function S=run_rcflex_control(D,C,H,resultFile)
% Native continuous Simulink reference run (slower than benchmark integrator).
if nargin<1;D=rcflex_control_case('sine',4,8,1);end
if nargin<2;C=rcflex_control_parameters;end
if nargin<3;H=rcflex_parameters;end
if nargin<4;resultFile='control_native.mat';end
Hnom=rcflex_parameters;Bnom=rcflex_body_properties(Hnom);B=rcflex_body_properties(H);
C=rcflex_calibrate_control(C,Hnom);H.sensor_seed=C.seed;
mdl='SensoSpoon_Control';file=project_path('models',[mdl '.slx']);if ~isfile(file);build_rcflex_control;end
load_system(file);w=get_param(mdl,'ModelWorkspace');
w.assignin('H',H);w.assignin('B',B);w.assignin('Hnom',Hnom);w.assignin('Bnom',Bnom);w.assignin('C',C);w.assignin('mode','voltage');
w.assignin('base_motion',timeseries(D.base,D.time));
set_param(mdl,'StopTime',num2str(D.time(end)),'Solver','ode23t','RelTol','1e-4','AbsTol','1e-6');
o=sim(mdl);assert(isempty(o.ErrorMessage),o.ErrorMessage);
t=(0:.001:D.time(end))';y=o.analytic_log;c=o.control_log;s=o.sensor_log;
S=struct('time',t,'Y',interp1(y.Time,squeeze(y.Data),t),'base',interp1(D.time,D.base,t),...
 'intent',interp1(D.time,D.intent,t),'control',interp1(c.Time,squeeze(c.Data),t,'previous'),...
 'sensor_samples',interp1(s.Time,squeeze(s.Data),t,'previous'),'H',H,'C',C,'mode','closed_loop','solver','Simulink_ode23t');
save(project_path('results',resultFile),'S');save_system(mdl,file);close_system(mdl,0);
end
