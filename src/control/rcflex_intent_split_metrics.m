function m=rcflex_intent_split_metrics(S,splitHz)
%RCFLEX_INTENT_SPLIT_METRICS Score the estimator's intent/tremor decomposition.
%
% The headline suppression figure scores the SPOON. This scores the ESTIMATOR,
% which is where the sEMG fusion acts, and it separates the two ways the
% decomposition can be wrong -- which the single RMS number cannot.
%
% DEFINITIONS (fixed before any fusion result was produced, so that neither the
% split frequency nor the pass direction could be chosen to flatter an arm):
%
%   e(t) = ref(t) - intent_truth(t)      the error in the reference the
%                                        controller actually applied, i.e. what
%                                        the estimator called voluntary motion
%                                        minus what was actually voluntary.
%
%   tremor_leak_deg   = RMS of HIGHPASS(e) above splitHz.
%                       Tremor the estimator ABSORBED INTO INTENT and therefore
%                       reproduced faithfully at the spoon instead of
%                       cancelling it. Lower is better. This is the quantity
%                       the omega-capture experiment identified as the residual
%                       EKF failure, and the one the sEMG gate targets.
%
%   intent_track_deg  = RMS of LOWPASS(e) below splitHz.
%                       Voluntary motion the estimator FAILED TO FOLLOW, i.e.
%                       movement the device would suppress. Lower is better.
%                       This is the quantity that must NOT degrade when
%                       tremor_leak_deg improves -- the two trade against each
%                       other, and reporting either alone hides the trade.
%
% splitHz defaults to 2 Hz. Chosen structurally, not fitted: it sits above the
% voluntary band every scenario drives (0.2-0.25 Hz sinusoids; the 'reach'
% minimum-jerk profile puts essentially all of its energy below 1.5 Hz) and
% below the lowest tremor content anywhere in the benchmark (4 Hz fundamentals,
% and the 3 Hz off-harmonic component the 'mixed' and 'irregular' scenarios
% carry -- which lands on the tremor side, where it belongs).
%
% The filters are the same causal 2-pole sections used throughout the project,
% applied forward only. Zero-phase filtering would be acausal and would smear
% the startup transient backwards across the scoring boundary.
if nargin<2;splitHz=2;end
t=S.time;dt=t(2)-t(1);fs=1/dt;
% ref is logged as [pitch; roll]; S.intent columns are [roll pitch].
ref=S.control(:,3:4);
truth=S.intent(:,[2 1]);
e=ref-truth;
lo=filt2(lowpass2(splitHz,fs),e);
hi=e-lo;
w=t>=1.5;                     % the same steady window every other metric uses
assert(any(w),'Need at least 1.5 seconds for steady scoring');
m.split_Hz=splitHz;
m.tremor_leak_deg=rad2deg(sqrt(mean(sum(hi(w,:).^2,2))));
m.intent_track_deg=rad2deg(sqrt(mean(sum(lo(w,:).^2,2))));
m.intent_ref_rms_deg=rad2deg(sqrt(mean(sum(truth(w,:).^2,2))));
% Gate confidence, when an intent arm published one (ekfDiag slots 6:7 ->
% diagnostic columns 22:23). Reads zero on every other arm.
m.gate_conf_roll=mean(S.control(w,22));
m.gate_conf_pitch=mean(S.control(w,23));
m.gate_conf_min=min(min(S.control(w,22:23),[],1));
end
function y=filt2(c,x)
y=zeros(size(x));
for j=1:size(x,2)
 z=[0;0];
 for k=1:size(x,1)
  u=x(k,j);v=c(1)*u+z(1);
  z(1)=c(2)*u-c(4)*v+z(2);z(2)=c(3)*u-c(5)*v;
  y(k,j)=v;
 end
end
end
function c=lowpass2(f,fs)
k=tan(pi*f/fs);den=1+sqrt(2)*k+k*k;
c=[k*k 2*k*k k*k]/den;c=[c 2*(k*k-1)/den (1-sqrt(2)*k+k*k)/den];
end
