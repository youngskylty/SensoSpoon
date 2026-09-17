function [s,conf,drive]=rcflex_semg_intent_gate(s,env,C)
%RCFLEX_SEMG_INTENT_GATE Voluntary-drive confidence per axis, causally.
% The sEMG->EKF fusion observable used by C.semg_fusion='intent_gate'. It
% answers ONE question per axis -- "is the muscle pair currently driving a
% voluntary joint torque, and how hard?" -- and returns it as a confidence in
% [0,1]. The EKF consumes that confidence as a scale on the intent process
% noise (rcflex_intent_ekf_step), i.e. as the estimator's licence to attribute
% motion to voluntary intent.
%
% WHY THIS OBSERVABLE AND NOT AN INTENT ANGLE. A calibrated angle was measured
% first and rejected on evidence. The differential envelope does track the
% voluntary angle well in aggregate (r = 0.945-0.984 across the intent-bearing
% scenarios), but its error is 1.6-1.8 deg RMS on the roll axis -- dominated by
% the activation saturation the truth model imposes at full MVC -- and its
% residual noise in the 1-5 Hz band is 0.6-1.4 deg. The intent absorption this
% fusion exists to prevent is a 0.02-0.18 deg effect, so an angle
% pseudo-measurement would inject an order of magnitude more error than it
% removed. The MAGNITUDE of voluntary drive, by contrast, separates the two
% classes by 14:1 on roll and 9:1 on pitch across all 20 benchmark cases
% (tremor-only <=0.28 deg equivalent, intent-bearing >=1.99 deg). A gate is
% therefore the strongest claim this signal actually supports, and it is the
% claim the estimator needs. See docs/SEMG_INTENT_FUSION.md Sec 3.
%
% s     : packed state, C.semg.s_len_gate elements. Pass [] to initialise from
%         C.semg.s0_gate. Pure function, no persistent variables -- the same
%         determinism contract rcflex_semg_preprocess.m and
%         rcflex_intent_ekf_step.m work under.
% env   : 1 x nch (or nch x 1) ENVELOPE features for this tick, i.e. elements
%         1:nch of the vector rcflex_semg_preprocess returns. Channel order
%         [roll agonist, roll antagonist, pitch agonist, pitch antagonist].
% conf  : 2 x 1 voluntary-drive confidence in [0,1], [roll; pitch].
% drive : 2 x 1 calibrated voluntary-angle-equivalent drive magnitude (rad) on
%         the SLOW path, published for diagnostics only.
E=C.semg;
if isempty(s);s=E.s0_gate;end

% Agonist MINUS antagonist. The signed difference, not the sum: a reciprocally
% activated pair carries the net joint torque in its difference, while the sum
% is dominated by the co-contraction floor, which rises with tremor as well as
% with voluntary effort and so does not discriminate.
e=env(:);
d=[e(1)-e(2);e(3)-e(4)];

% TWO PATHS, at two speeds. Both are second-order lowpasses on the same
% differential envelope; they differ only in corner frequency, and each has its
% own floor measured at its own bandwidth.
%
% The differential envelope carries tremor-driven activation as well as
% voluntary drive -- muscle force follows tremor ACCELERATION, which grows as
% omega^2, so the leakage is worst at high tremor frequency. Rejecting it needs
% a low corner, and a low corner costs group delay. That delay is the problem:
% at 1 Hz it is ~225 ms, and with the envelope stage ahead of it the gate needs
% 0.3-0.5 s to notice a voluntary movement. Measured cost on the 'reach'
% scenario with the slow path alone, four sensor seeds: 0.73-1.15 deg attitude
% RMS against 0.285 deg for the IMU-only estimator -- i.e. the gate suppressed
% the very voluntary movement it exists to protect.
%
% The resolution follows from the asymmetry this whole function is built on.
% Wrongly CLOSING the gate is expensive; wrongly OPENING it only returns the
% estimator to its ungated baseline. So the two paths are combined with max():
%
%   SLOW path (C.semg_gate_fc_Hz, 1 Hz)      clean, high separation, late.
%                                            Decides when the gate may CLOSE.
%   FAST path (C.semg_gate_fast_fc_Hz, 4 Hz) noisier, lower separation, early.
%                                            Can only ever OPEN the gate.
%
% The fast path is given its own, higher floor, measured at its own bandwidth
% by the same geometric-mean rule, so its extra tremor leakage is priced in
% rather than ignored. It cannot close the gate and it cannot lower the
% confidence, so the worst a false fast-path trigger can do is cost some tremor
% suppression for a moment.
Z=reshape(s(E.ix_gate_z),2,2);
Zf=reshape(s(E.ix_gate_zf),2,2);
u=zeros(2,1);uf=zeros(2,1);
for a=1:2
 [u(a),Z(:,a)]=biquad(E.gate_lp,d(a),Z(:,a));
 [uf(a),Zf(:,a)]=biquad(E.gate_lp_fast,d(a),Zf(:,a));
end

% Calibrated to an equivalent voluntary joint angle so the floor below is a
% physical quantity rather than a property of this particular electrode gain.
% E.gate_cal comes from a tremor-free voluntary calibration trial scored
% against the IMU-observable handle attitude -- a procedure that is realisable
% on a real subject and needs no ground-truth decomposition. See
% src/analysis/calibrate_rcflex_semg_intent.m.
% The same calibration constant serves both paths: both lowpasses have unit DC
% gain, so they differ in bandwidth and delay but not in scale.
drive=E.gate_cal(:).*u;
driveFast=E.gate_cal(:).*uf;

% Mean-square drive, tracked with an ASYMMETRIC envelope follower: a fast
% attack and a slow release.
%
% The asymmetry is deliberate, because the two errors do not cost the same.
% Wrongly CLOSING the gate withdraws the estimator's licence to track voluntary
% motion, and shows up directly as suppressed voluntary movement -- the one
% failure this device must not have. Wrongly leaving it OPEN merely returns the
% estimator to its ungated behaviour, which is the baseline. So the gate opens
% on the first evidence of drive and closes only once that evidence has been
% absent for a while.
%
% It also removes a specific measured fault. Voluntary drive follows the intent
% ANGLE, so a symmetric window makes the confidence collapse at every zero
% crossing of a reciprocating movement -- precisely when the intent VELOCITY is
% largest and the intent states need their freedom most. Measured on benchmark
% case 9: the pitch confidence fell to 0.40 at each crossing.
alpha=E.gate_alpha_rel*ones(2,1);
alpha(drive.^2>s(E.ix_gate_pow))=E.gate_alpha_att;
P=s(E.ix_gate_pow)+alpha.*(drive.^2-s(E.ix_gate_pow));
alphaF=E.gate_alpha_rel*ones(2,1);
alphaF(driveFast.^2>s(E.ix_gate_powf))=E.gate_alpha_att;
Pf=s(E.ix_gate_powf)+alphaF.*(driveFast.^2-s(E.ix_gate_powf));

% Standard soft gate, the same form this codebase already uses for presence
% detection (rcflex_semg_omega, and the osc2 gate in rcflex_intent_ekf_step):
% conf = P/(P+Pfloor).
%
% SQUARED. The first-order soft gate spans only about 0.85 decades of output
% over the 1.7 decades of measured class separation, and the ceiling experiment
% showed the benefit needs roughly 1.8 decades of swing in the intent process
% noise to be realised. Squaring the gate matches the achievable output range
% to the measured input range; it introduces no new threshold, only an
% exponent, and the exponent is set by that measurement rather than by the
% closed-loop outcome.
conf=max((P./(P+E.gate_floor)).^E.gate_exponent, ...
         (Pf./(Pf+E.gate_floor_fast)).^E.gate_exponent);

% WARM-UP: hold the gate fully OPEN until its own filter chain has settled.
%
% A gate may only ever REMOVE authority on positive evidence. Before its
% filters have settled it has none, and P starts at zero, so without this the
% gate spends its first second asserting that there is no voluntary drive on
% the strength of an unsettled filter. Measured consequence: on benchmark case
% 9 the confidence was pinned at 0.000 for the first 0.85 s -- exactly the
% window in which intent and tremor ramp together and the joint fit is least
% identifiable -- and the case fell from 85.9 % to 62.0 %. P keeps being
% tracked throughout the warm-up, so the gate is fully informed the instant it
% is allowed to act and there is no release lag at the handover.
k=s(E.ix_gate_k)+1;s(E.ix_gate_k)=k;
if k<=E.gate_warmup_n;conf=ones(2,1);end

s(E.ix_gate_z)=Z(:);s(E.ix_gate_pow)=P;
s(E.ix_gate_zf)=Zf(:);s(E.ix_gate_powf)=Pf;
end
function [y,z]=biquad(c,u,z)
% Direct form II transposed, identical to rcflex_semg_preprocess.
y=c(1)*u+z(1);
z(1)=c(2)*u-c(4)*y+z(2);
z(2)=c(3)*u-c(5)*y;
end
