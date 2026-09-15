function [s,omegaHz,conf]=rcflex_semg_omega(s,burst,C)
%RCFLEX_SEMG_OMEGA Tremor frequency from the sEMG burst band, causally.
% The independent omega observable that CONTROL_IMPROVEMENT_PLAN.md Sec 1a
% concluded was needed. Sec 1a's diagnosis was that the joint intent + two-
% harmonic + shared-omega fit is weakly identifiable while intent and tremor
% ramp together: omega was dragged to a false 2.0-2.3 Hz while nis_bar stayed
% below 0.15, so the filter was internally self-consistent while being wrong
% and no statistic of the IMU measurement could detect it. The remedy is
% information about omega that does not come from the joint fit at all.
%
% Sec 1a's own lag-autocorrelation estimator was reverted because, applied to
% the same handle-attitude signal the EKF was already fitting, it was not
% independent. Applied to sEMG it is: a different transducer on a different
% physical quantity. The identity is unchanged. For a single sinusoid h,
%
%   E[ h(k-D) * ( h(k) + h(k-2D) ) ] = 2*cos(omega*D*dt)*E[h^2]
%
% so with num/den accumulated as EWMAs,
%
%   omega = acos( num_bar / den_bar ) / (D*dt)
%
% WHY FREQUENCY AND NOT PHASE. The burst band also leads the mechanical tremor
% by the electromechanical delay (~35 ms measured synthetically), and that lead
% is the only thing sEMG offers which a better IMU-only estimator could not.
% It is deliberately NOT used here. A constant or drifting EMD shifts burst
% PHASE but leaves burst FREQUENCY untouched, so this observable is insensitive
% to the one assumption the real-data gate could not verify (stage 4 remains
% SKIPPED; no real recording exists). Frequency is the defensible thing to fuse
% first.
%
% ACCUMULATE PER CHANNEL, THEN SUM. The four channels are summed at the
% accumulator level, not at the signal level. Agonist and antagonist bursts can
% sit in antiphase under reciprocal activation, so summing the waveforms first
% would let them cancel; each channel's own num/den are sign-definite in the
% way that matters, so summing those is phase-insensitive and still averages
% four channels' worth of noise down.
%
% s     : packed state, [] to initialise. Layout is C.semg.ix_fuse_*.
% burst : 1 x nch (or nch x 1) burst features for this tick, i.e. elements
%         nch+1:end of the vector rcflex_semg_preprocess returns.
% omegaHz : frequency estimate (Hz). Meaningless when conf is ~0.
% conf  : 0..1 burst-power confidence. Zero while the delay line is still
%         filling (it starts at zero, so the power EWMA starts at zero) and
%         near zero whenever there is no tremor burst to measure. The caller
%         must use it to inflate R, never ignore it -- without the gate this
%         function returns the arccos of a noise ratio during rest.
E=C.semg;
if isempty(s);s=E.s0_fuse;end
D=E.fuse_D;nch=E.nch;nBuf=2*D+1;

buf=reshape(s(E.ix_fuse_buf),nBuf,nch);
buf=[burst(:)';buf(1:end-1,:)];          % row 1 is the newest tick

h0=buf(1,:);hD=buf(D+1,:);h2D=buf(nBuf,:);
num=sum(hD.*(h0+h2D));
den=sum(2*hD.^2);
pow=sum(h0.^2);

a=E.fuse_alpha;
numBar=s(E.ix_fuse_num)+a*(num-s(E.ix_fuse_num));
denBar=s(E.ix_fuse_den)+a*(den-s(E.ix_fuse_den));
powBar=s(E.ix_fuse_pow)+a*(pow-s(E.ix_fuse_pow));

% acos is single-valued on [0,pi], and D was chosen so omega*D*dt cannot leave
% that interval anywhere in the 2-25 Hz clamp band. Clamping the ratio guards
% the numerics only; it is not hiding an ambiguity.
if denBar>0;r=numBar/denBar;else;r=1;end
r=max(min(r,1),-1);
omegaHz=acos(r)/(2*pi*D/C.outer_hz);
conf=powBar/(powBar+C.semg_fuse_floor);

s(E.ix_fuse_buf)=buf(:);
s(E.ix_fuse_num)=numBar;s(E.ix_fuse_den)=denBar;s(E.ix_fuse_pow)=powBar;
end
