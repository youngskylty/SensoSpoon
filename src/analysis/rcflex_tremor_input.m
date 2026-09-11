function base=rcflex_tremor_input(t,frequency,amplitude_deg,translation_mm)
% Reproducible synthetic scenario, NOT a clinical patient recording.
% Columns: roll/pitch, rates, accelerations, world xyz position/velocity/accel.
if nargin<4;translation_mm=1;end
t=t(:);w=2*pi*frequency;a=deg2rad(amplitude_deg);phase=[0 .7];
ang=a*sin(w*t+phase);vel=a*w*cos(w*t+phase);acc=-a*w*w*sin(w*t+phase);
amp=translation_mm/1000*[.5 .3 1];ph=[.2 1.1 -.4];
pos=sin(w*t+ph).*amp;pv=w*cos(w*t+ph).*amp;pa=-w*w*sin(w*t+ph).*amp;
base=[ang vel acc pos pv pa];
end
