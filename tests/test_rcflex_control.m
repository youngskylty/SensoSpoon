function test_rcflex_control()
% Independent electrical step/reversal/recovery and causal feedback checks.
H=rcflex_parameters;C=rcflex_control_parameters;dt=1/C.current_hz;
i=zeros(2,1);integral=i;adc=[i;H.bus_voltage_V];log=zeros(4000,2);
for k=1:4000
 t=(k-1)*dt;r=.7*ones(2,1);if t>=.1;r=-r;end;if t>=.2&&t<.3;r=3*ones(2,1);end;if t>=.3;r=zeros(2,1);end
 [integral,v]=rcflex_current_step(integral,adc,[r;0;0],H,C);
 adc=rcflex_current_sample(i,t,H,C);
 target=v/(H.coil_R_ohm+H.driver_R_ohm);i=target+(i-target)*exp(-(H.coil_R_ohm+H.driver_R_ohm)*dt/H.coil_L_H);
 i=max(min(i,H.current_limit_A),-H.current_limit_A);log(k,:)=i';
 assert(all(isfinite(i))&&max(abs(v))<=H.bus_voltage_V+1e-9);
end
assert(max(abs(mean(log(500:900,:))-.7))<.015,'Positive current tracking');
assert(max(abs(mean(log(1500:1900,:))+.7))<.015,'Current reversal');
assert(max(abs(log(3400:end,:)),[],'all')<.015,'Voltage saturation recovery');
B=rcflex_body_properties(H);C=rcflex_calibrate_control(C,H);
for j=1:size(C.intent_sos,1);assert(all(abs(roots(C.intent_sos(j,4:6)))<1),'Intent filter poles');end
dc=prod(sum(C.intent_sos(:,1:3),2)./sum(C.intent_sos(:,4:6),2));assert(abs(dc-1)<1e-8,'Intent DC tracking');
s=repmat([0;0;0;0;0;9.80665],2,1);s=[s;0;0];z=[];
for k=1:100;[z,u]=rcflex_attitude_step(z,s,H,B,C);end
[z1,u1]=rcflex_attitude_step(z,s,H,B,C);[z2,u2]=rcflex_attitude_step(z,s,H,B,C);
assert(isequal(z1,z2)&&isequal(u1,u2),'Controller must be deterministic');
assert(all(abs(u(1:2))<=.98*H.current_limit_A+eps));
% Orientation-feedback sign: positive spoon pitch should reduce pitch torque.
ztilt=z;R=[cos(.01) 0 sin(.01);0 1 0;-sin(.01) 0 cos(.01)];ztilt(10:18)=R(:);
[~,ut]=rcflex_attitude_step(ztilt,s,H,B,C);L=rcflex_linkage([0;0],H);
dtau=L.J'*(H.vca_force_constant_N_A*(ut(1:2)-u(1:2)));
assert(dtau(1)<0,'Pitch feedback sign');
fprintf('Control checks: current tracking, reversal, saturation recovery, deterministic feedback and pitch sign passed.\n');
end
