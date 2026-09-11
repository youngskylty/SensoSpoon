function C=rcflex_prepare_control(C)
% Fixed second-order sections, computed once before starting the controller.
if strcmp(C.intent_filter,'lowpass')
 C.intent_sos=lowpass(C.intent_hz,C.outer_hz);return
end
assert(strcmp(C.intent_filter,'notch'),'Unknown intent filter');
f=C.notch_frequencies_Hz;assert(numel(f)<=5,'Too many sections for controller state');C.intent_sos=zeros(numel(f)+2,6);
for j=1:numel(f)
 k=tan(pi*f(j)/C.outer_hz);d=C.notch_bandwidth_Hz/f(j)*k;den=1+d+k*k;
 C.intent_sos(j,:)=[1+k*k 2*(k*k-1) 1+k*k den 2*(k*k-1) 1-d+k*k]/den;
end
k=tan(pi*C.intent_lead_pole_Hz/C.outer_hz);lead=1+C.intent_lead_s*2*pi*C.intent_lead_pole_Hz;
C.intent_sos(end-1,:)=[(k+lead)/(1+k) (k-lead)/(1+k) 0 1 (k-1)/(k+1) 0];
C.intent_sos(end,:)=lowpass(C.intent_rolloff_Hz,C.outer_hz);
end
function c=lowpass(f,fs)
k=tan(pi*f/fs);den=1+sqrt(2)*k+k*k;
c=[k*k 2*k*k k*k den 2*(k*k-1) 1-sqrt(2)*k+k*k]/den;
end
