function D=rcflex_control_case(name,duration,hz,amplitude,seed,C)
% Offline test generator. Intent truth is never supplied to the controller.
% When C.semg_enabled is set, a synthetic sEMG stream is generated alongside
% the mechanical scenario and returned in D.semg / D.semg_time. It is built
% from the SAME intent and tremor components used to build D.base, which is
% what makes it a usable characterisation dataset: the ground-truth
% decomposition the controller must infer is known exactly here.
if nargin<6;C=[];end
if nargin<1;name='sine';end
if nargin<2;duration=4;end
if nargin<3;hz=8;end
if nargin<4;amplitude=1;end
if nargin<5;seed=1701;end
dt=1/20000;t=(0:dt:duration)';u=max(0,min(1,(t-.5)/.5));ramp=10*u.^3-15*u.^4+6*u.^5;
intent=zeros(numel(t),3);tremor=zeros(numel(t),3);pos=zeros(numel(t),3);
phase=hz*t;
if strcmp(name,'chirp');phase=3*t+9*t.^2/(2*duration);end
shape=[sin(2*pi*phase),sin(2*pi*phase+.7)];
if strcmp(name,'mixed')||strcmp(name,'irregular')
 shape=.65*shape+.25*[sin(2*pi*(hz+2)*t+.4),sin(2*pi*(hz-1)*t+1.2)]+.1*[sin(2*pi*12*t),sin(2*pi*3*t)];
end
if strcmp(name,'step');ramp=ramp.*(1+max(0,min(1,(t-duration/2)/.15)));end
if strcmp(name,'intent')||strcmp(name,'combined');intent(:,1)=deg2rad(8)*ramp.*sin(2*pi*.2*t);intent(:,2)=deg2rad(5)*ramp.*sin(2*pi*.25*t+.2);end
% 'reach': a FAST voluntary movement superimposed on tremor. Every other
% intent-bearing scenario drives intent at 0.2-0.25 Hz, which a slow estimator
% can track by being slow; nothing in the 20-case benchmark asks whether a
% voluntary movement is preserved when it happens QUICKLY. That is exactly the
% question the sEMG intent gate has to answer -- the gate must open before the
% limb has moved, which is what the electromechanical delay is for -- so the
% scenario exists to test it. Minimum-jerk profile, 0.8 s, starting at t=2 s so
% it falls inside the metrics' t>=1.5 s scoring window with time either side.
% Additive: no existing scenario name changes behaviour.
if strcmp(name,'reach')
 uR=max(0,min(1,(t-2)/.8));sR=10*uR.^3-15*uR.^4+6*uR.^5;
 intent(:,1)=deg2rad(8)*ramp.*sR;intent(:,2)=deg2rad(5)*ramp.*sR;
end
if ~strcmp(name,'intent');tremor(:,1:2)=deg2rad(amplitude)*ramp.*shape;end
if strcmp(name,'combined')||strcmp(name,'irregular');pos=.001*ramp.*[.5*sin(2*pi*hz*t+.2),.3*sin(2*pi*hz*t+1.1),sin(2*pi*hz*t-.4)];end
angles=intent(:,1:2)+tremor(:,1:2);rate=deriv(angles,dt);acc=deriv(rate,dt);pv=deriv(pos,dt);pa=deriv(pv,dt);
D=struct('name',name,'time',t,'base',[angles rate acc pos pv pa],'intent',intent,...
 'tremor',tremor,'seed',seed,'frequency',hz,'amplitude',amplitude);
D.semg=[];D.semg_time=[];D.semg_activation=[];
if ~isempty(C)&&isfield(C,'semg_enabled')&&C.semg_enabled
 % Routed through rcflex_semg_source so a recorded dataset or the physical
 % front end can replace the synthetic generator without touching this file.
 [D.semg,D.semg_time,meta]=rcflex_semg_source(D,C);
 D.semg_meta=meta;
 if isfield(meta,'activation');D.semg_activation=meta.activation;end
 if isfield(meta,'clean');D.semg_clean=meta.clean;end
end
end
function v=deriv(p,dt);v=zeros(size(p));for j=1:size(p,2);v(:,j)=gradient(p(:,j),dt);end;end
