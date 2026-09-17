function [ok,info]=rcflex_case_feasible(frequency_Hz,amplitude_deg,H)
%RCFLEX_CASE_FEASIBLE Is this tremor input inside the hardware envelope at all?
% Inverse-dynamics bound only: it asks what current, voltage, copper power,
% stroke and side load would be required to track the input EXACTLY under
% perfect knowledge, and reports whether the hardware could supply them. It
% says nothing about any particular controller, which is precisely why it can
% be used to decide which benchmark cases a controller may fairly be scored on.
%
% Extracted from evaluate_rcflex_envelope.m, which now calls it, so the
% per-case flag recorded in the benchmark and the envelope table published in
% output/results/tremor_hardware_envelope.csv cannot drift apart.
%
% Scored on one full tremor period at 121 points -- enough for the peak of a
% sinusoid and its first two derivatives, and identical to what the envelope
% table has always used.
%
% CAVEAT, stated because the benchmark relies on it: amplitude_deg is the
% scenario's NOMINAL amplitude. The 'mixed' and 'irregular' scenarios
% superimpose components whose coefficients sum to 1.0, so their peak demand is
% close to but not exactly the pure sinusoid's. The flag is therefore a
% single-tone bound, and it is used only to exclude cases that miss the
% envelope by a wide margin, never to make a marginal call.
if nargin<3;H=rcflex_parameters();end
B=rcflex_body_properties(H);
t=linspace(0,1/frequency_Hz,121)';
if amplitude_deg<=0
 % No tremor input at all, so the demand is identically zero and the case is
 % trivially inside the envelope. The struct is still filled out in full: a
 % caller that reads a field only some branches define is exactly how a
 % feasibility flag silently becomes an error halfway through a long run.
 ok=true;
 info=struct('required_peak_A',0,'required_peak_V',0,'max_coil_mean_W',0,...
  'peak_stroke_mm',0,'peak_side_load_N',0,'q',zeros(numel(t),2),...
  'base',rcflex_tremor_input(t,frequency_Hz,0,0));
 return
end
base=rcflex_tremor_input(t,frequency_Hz,amplitude_deg,1);
q=-base(:,[2 1]);qd=-base(:,[4 3]);qdd=-base(:,[6 5]);
n=numel(t);ii=zeros(n,2);sd=ii;stroke=ii;side=ii;
for k=1:n
 [iReq,rc]=rcflex_required_current(q(k,:)',qd(k,:)',qdd(k,:)',base(k,1:2)',base(k,3:4)',base(k,5:6)',base(k,7:15)',H,B);
 ii(k,:)=iReq.';stroke(k,:)=rc.stroke.';side(k,:)=rc.side_load_N.';sd(k,:)=(rc.L.J*qd(k,:)').';
end
di=[gradient(ii(:,1),t) gradient(ii(:,2),t)];
K=H.vca_force_constant_N_A*max(1-.27*(stroke/(H.vca_stroke_mm/1000)).^2,.5);
voltage=(H.coil_R_ohm+H.driver_R_ohm)*ii+H.coil_L_H*di+K.*sd;
bus=H.bus_voltage_V-H.battery_R_ohm*sum(abs(ii),2);
copper=max(mean(H.coil_R_ohm*ii.^2,1));
info=struct('required_peak_A',max(abs(ii),[],'all'),'required_peak_V',max(abs(voltage),[],'all'),...
 'max_coil_mean_W',copper,'peak_stroke_mm',max(abs(stroke),[],'all')*1000,...
 'peak_side_load_N',max(side,[],'all'),'q',q,'base',base);
ok=info.required_peak_A<=H.current_limit_A && all(abs(voltage)<=bus,'all') && ...
   copper<=H.coil_continuous_power_W && info.peak_side_load_N<=H.shaft_side_load_N && ...
   max(abs(stroke),[],'all')<=H.vca_stroke_mm/1000;
end
