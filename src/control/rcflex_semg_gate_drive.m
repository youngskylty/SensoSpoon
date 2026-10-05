function [u,tTick]=rcflex_semg_gate_drive(raw,C,c,t0)
%RCFLEX_SEMG_GATE_DRIVE Lowpassed agonist-antagonist envelope difference.
% The open-loop observable behind rcflex_semg_intent_gate, evaluated over a
% whole stream at the outer-loop rate. Byte-for-byte the same chain the gate
% runs online -- source -> rcflex_semg_features -> the selected gate lowpass --
% so a constant fitted here is the constant the controller uses, and a
% separation measured here is the separation the gate sees.
%
% Factored out of calibrate_rcflex_semg_intent.m so the calibration, the
% fast-movement experiment and the real-dataset analysis cannot drift apart.
% Do not re-implement this loop in an analysis script.
%
% raw   : n x C.semg_channels electrode-referred volts at C.semg_hz.
% c     : biquad row to apply, one of C.semg.gate_lp (slow path, the default),
%         C.semg.gate_lp_fast (fast path) or C.semg.gate_lp_rate (the rate
%         detector's post-differentiator smoother; see rcflex_semg_init.m).
% t0    : optional stream start time (s), default 0.
% u     : nTick x 2 filtered differential envelope, [roll pitch], in VOLTS.
%         Multiply by C.semg.gate_cal to get the equivalent voluntary angle.
% tTick : nTick x 1 outer-rate times, same convention as rcflex_semg_features.
if nargin<3||isempty(c);c=C.semg.gate_lp;end
if nargin<4;t0=0;end
[F,tTick]=rcflex_semg_features(raw,C,t0);
d=[F(:,1)-F(:,2), F(:,3)-F(:,4)];
u=zeros(size(d));z=zeros(2,2);
for k=1:size(d,1)
 for a=1:2
  y=c(1)*d(k,a)+z(1,a);
  z(1,a)=c(2)*d(k,a)-c(4)*y+z(2,a);
  z(2,a)=c(3)*d(k,a)-c(5)*y;
  u(k,a)=y;
 end
end
end
