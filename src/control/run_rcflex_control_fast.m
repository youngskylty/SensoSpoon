function S=run_rcflex_control_fast(D,C,H,mechanicalStep)
% Multirate benchmark integrator; held acceleration at mechanicalStep.
% Must be checked against finer steps and native continuous Simulink plant.
if nargin<2;C=rcflex_control_parameters;end
if nargin<3;H=rcflex_parameters;end
if nargin<4;mechanicalStep=.0005;end
Hnom=rcflex_parameters;Bnom=rcflex_body_properties(Hnom);B=rcflex_body_properties(H);
C=rcflex_calibrate_control(C,Hnom);dt=D.time(2)-D.time(1);n=numel(D.time);outerN=round(1/C.outer_hz/dt);innerN=round(1/C.current_hz/dt);mechN=round(mechanicalStep/dt);logN=round(.001/dt);
x=zeros(8,1);acc=zeros(2,1);z=[];integ=zeros(2,1);command=zeros(4,1);voltage=zeros(2,1);adc=[0;0;H.bus_voltage_V];
f=repmat([0;0;0;0;0;9.80665],2,1);measurement=[f;zeros(2,1)];diagnostic=zeros(16,1);
Y=zeros(ceil(n/logN),26);time=zeros(size(Y,1),1);base=zeros(size(Y,1),15);intent=zeros(size(Y,1),3);logs=zeros(size(Y,1),16);sensors=zeros(size(Y,1),14);count=0;
for k=1:n
 t=D.time(k);b=D.base(k,:)';
 if mod(k-1,mechN)==0||mod(k-1,outerN)==0||mod(k-1,logN)==0
  [dx,y]=rcflex_physics(x,[b(1:6);voltage;b(7:15)],H,B,'voltage');acc=dx(3:4);
 end
 if mod(k-1,outerN)==0
  [z,command,diagnostic]=rcflex_attitude_step(z,measurement,Hnom,Bnom,C);
  truth=rcflex_sensor_truth(y,b,H);[m,f]=rcflex_imu_sample(truth,f,t,H,C.seed);
  measurement=[m;round(x(5:6)/C.current_quantum)*C.current_quantum];
 end
 if mod(k-1,innerN)==0
  [integ,voltage]=rcflex_current_step(integ,adc,command,Hnom,C);
  adc=rcflex_current_sample(x(5:6),t,H,C);
 end
 if mod(k-1,logN)==0
  count=count+1;Y(count,:)=y';time(count)=t;base(count,:)=b';intent(count,:)=D.intent(k,:);logs(count,:)=diagnostic';sensors(count,:)=measurement';
 end
 if k==n;break;end
 % Exact held-coefficient RL solution; same voltage/chopping/thermal limits.
 L=rcflex_linkage(x(1:2),H);sd=L.J*x(3:4);K=H.vca_force_constant_N_A*max(1-.27*(L.s/(H.vca_stroke_mm/1000)).^2,.5);
 R=H.coil_R_ohm*(1+H.coil_alpha_K*x(7:8));bus=max(0,H.bus_voltage_V-H.battery_R_ohm*sum(abs(x(5:6))));v=max(min(voltage,bus),-bus);if bus<H.driver_min_voltage_V;v=zeros(2,1);end
 target=(v-K.*sd)./(R+H.driver_R_ohm);x(5:6)=target+(x(5:6)-target).*exp(-(R+H.driver_R_ohm)*dt/H.coil_L_H);
 x(5:6)=max(min(x(5:6),H.current_limit_A),-H.current_limit_A);
 x(7:8)=x(7:8)+dt*(R.*x(5:6).^2-x(7:8)/H.thermal_resistance_K_W)/H.thermal_capacity_J_K;
 x(1:2)=x(1:2)+dt*x(3:4)+.5*dt^2*acc;x(3:4)=x(3:4)+dt*acc;
 if any(~isfinite(x))||any(abs(x(1:2))>deg2rad(25));error('SensoSpoon:ControlUnstable','Unstable at %.3f s',t);end
end
S=struct('time',time(1:count),'Y',Y(1:count,:),'base',base(1:count,:),'intent',intent(1:count,:),...
 'control',logs(1:count,:),'sensor_samples',sensors(1:count,:),'H',H,'C',C,'mode','closed_loop','solver','multirate_benchmark','mechanical_step',mechanicalStep);
end
