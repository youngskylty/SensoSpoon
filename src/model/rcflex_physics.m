function [dx,Y]=rcflex_physics(x,u,H,B,mode)
% x=[q(2);qd(2);coil current(2);temperature rise(2)].
% u=[hand(2);hand velocity(2);hand acceleration(2);voltage(2);world p/v/a(9)].
% Actual voltage drives L*di/dt = V-R(T)*i-K*sdot. Force=K*i, Q=J'*F.
q=x(1:2);v=x(3:4);i=x(5:6);temperature=x(7:8);
hand=u(1:2);hd=u(3:4);hdd=u(5:6);
L=rcflex_linkage(q,H);sd=L.J*v;
translation=zeros(9,1);if numel(u)>=17;translation=u(9:17);end
[M,bias,E]=rcflex_mass_bias(q,v,hand,hd,hdd,H,B,translation);
k=H.flexure_stiffness_Nm_rad;limit=deg2rad(H.physical_travel_deg);
spring=k.*q.*(1+H.flexure_cubic_ratio*(q/limit).^2);
damping=H.flexure_damping_Nms_rad*v;
penetration=max(abs(q)-limit,0);
stop=sign(q).*penetration*H.stop_stiffness_Nm_rad+H.stop_damping_Nms_rad*v.*(penetration>0);
strokeExcess=max(abs(L.s)-H.vca_stroke_mm/1000,0);
sliderStop=-H.slider_stop_N_m*sign(L.s).*strokeExcess-H.slider_stop_Ns_m*sd.*(strokeExcess>0);
% Force/position curve is an approximate digitization of the motor drawing,
% normalized to its stated 1.5 N/A at mid-stroke; its uncertainty is documented.
curve=1-.27*(L.s/(H.vca_stroke_mm/1000)).^2;
K=H.vca_force_constant_N_A*max(curve,.5);R=H.coil_R_ohm*(1+H.coil_alpha_K*temperature);
bus=max(0,H.bus_voltage_V-H.battery_R_ohm*sum(abs(i)));
voltage=max(min(u(7:8),bus),-bus);
if bus<H.driver_min_voltage_V;voltage=zeros(2,1);end
if strcmp(mode,'off');voltage=zeros(2,1);end % shorted H-bridge brake state
% Averaged hardware current chopping: constrain current derivative, not q/qd.
totalR=R+H.driver_R_ohm;
emf=K.*sd;
holdV=totalR.*i+emf;
for j=1:2
    if i(j)>=H.current_limit_A&&voltage(j)>holdV(j);voltage(j)=holdV(j);end
    if i(j)<=-H.current_limit_A&&voltage(j)<holdV(j);voltage(j)=holdV(j);end
end
friction=H.guide_friction_N*tanh(sd/.001)+H.guide_viscous_Ns_m*sd;
force=K.*i;Q=L.J'*(force+sliderStop-friction);
acc=M\(Q-bias-spring-damping-stop);
di=(voltage-totalR.*i-emf)/H.coil_L_H;
dT=(R.*i.^2-temperature/H.thermal_resistance_K_W)/H.thermal_capacity_J_K;
dx=[v;acc;di;dT];
% Outputs retain fixed 26-column interface: states1:8, reserved9:10,
% V11:12, F13:14, s15:16, beta17:18, qdd19:20, Joule W21:22,
% kinetic/potential23:24, slider velocity25:26.
Y=[x;zeros(2,1);voltage;force;L.s;L.beta;acc;R.*i.^2;E.kinetic;E.gravity+.5*sum(k.*q.^2)+H.flexure_cubic_ratio*sum(k.*q.^4)/(4*limit^2);sd];
end
