function [integral,v]=rcflex_current_step(integral,measured,command,H,C)
% 10-kHz PI, sampled current feedback, resistance/back-EMF feedforward.
bus=measured(3);measured=measured(1:2);e=command(1:2)-measured;
raw=(H.coil_R_ohm+H.driver_R_ohm)*command(1:2)+command(3:4)+C.current_kp*e+integral;
v=max(min(raw,bus),-bus);if bus<H.driver_min_voltage_V;v=zeros(2,1);end
integral=integral+(C.current_ki*e+C.current_aw*(v-raw))/C.current_hz;
integral=max(min(integral,H.bus_voltage_V),-H.bus_voltage_V);
end
