function S=run_rcflex_analytic(mode,duration,H,volts,base)
% Supply Nx2 voltage and Nx15 base motion (legacy Nx6 is zero-padded).
% No tremor estimator/controller is invoked. Default: common/differential drive.
if nargin<1;mode='voltage';end
if nargin<2;duration=6;end
if nargin<3;H=rcflex_parameters();end
rebuild=~exist(project_path('assets','assembly.mat'),'file');
if ~rebuild;old=load(project_path('assets','assembly.mat'),'H');rebuild=~isequaln(old.H,H);end
if rebuild;generate_rcflex_assets(H);end
B=rcflex_body_properties(H);t=(0:1/H.fs:duration)';
if nargin<4
    volts=zeros(numel(t),2);volts(t>=1 & t<2,:)=-1.5;volts(t>=2 & t<3,:)=1.5;
    volts(t>=3 & t<4,:)=repmat([-1.2 1.2],sum(t>=3&t<4),1);
    volts(t>=4 & t<5,:)=repmat([1.2 -1.2],sum(t>=4&t<5),1);
end
if nargin<5;base=zeros(numel(t),15);end
if size(base,2)==6;base=[base zeros(size(base,1),9)];end
assert(isequal(size(volts),[numel(t),2])&&isequal(size(base),[numel(t),15]),'Input array size mismatch.');
mdl='SensoSpoon_Analytic';if ~exist(project_path('models',[mdl '.slx']),'file');build_rcflex_analytic();end
load_system(project_path('models',[mdl '.slx']));w=get_param(mdl,'ModelWorkspace');w.assignin('H',H);w.assignin('B',B);w.assignin('mode',mode);
w.assignin('base_motion',timeseries(base,t));w.assignin('voltage_command',timeseries(volts,t));
set_param(mdl,'StopTime',num2str(duration));o=sim(mdl);assert(isempty(o.ErrorMessage),o.ErrorMessage);
log=o.analytic_log;Y=interp1(log.Time,squeeze(log.Data),t,'linear');
S=struct('H',H,'time',t,'Y',Y,'mode',mode,'voltage_request',volts,'base',base);
S.sensors=o.sensor_log;
S.motor_power_mean_W=mean(Y(:,21:22),1);S.coil_current_rms_A=sqrt(mean(Y(:,5:6).^2,1));
S.rated_continuous_power_exceeded=any(S.motor_power_mean_W>H.coil_continuous_power_W);
if ~exist(project_path('results'),'dir');mkdir(project_path('results'));end
save(fullfile(project_path('results'),['analytic_' mode '.mat']),'S');save_system(mdl,project_path('models',[mdl '.slx']));close_system(mdl,0);
fprintf('Plant %s: %.1f s; peak %.3f A / %.3f mm / %.2f deg; coil rise %.2f C.\n',mode,duration,max(abs(Y(:,5:6)),[],'all'),1000*max(abs(Y(:,15:16)),[],'all'),max(abs(rad2deg(Y(:,1:2))),[],'all'),max(Y(:,7:8),[],'all'));
end
