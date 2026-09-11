function adc=rcflex_current_sample(i,t,H,C)
% Independent delayed 10-kHz current and bus-voltage ADC model.
s=RandStream('mt19937ar','Seed',mod(C.seed+300000+round(t*C.current_hz),2^32-1));
a=i+C.current_offset+C.current_noise_A*randn(s,2,1);
a=max(min(round(a/C.current_quantum)*C.current_quantum,3.5),-3.5);
bus=max(0,H.bus_voltage_V-H.battery_R_ohm*sum(abs(i)));
adc=[a;round(bus/(10/4096))*(10/4096)];
end
