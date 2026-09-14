function [emg,tEmg,activation]=rcflex_semg_truth(time,intent,tremor,C,seed)
%RCFLEX_SEMG_TRUTH Clean 4-channel surface EMG for a scenario.
% Reproducible synthetic scenario, NOT a clinical patient recording -- the same
% caveat that rcflex_tremor_input.m carries. It exists so the intent-vs-tremor
% information content of sEMG can be characterised against a KNOWN ground
% truth before any of it is allowed near the estimator.
%
% time   : scenario time column (s), the grid rcflex_control_case.m uses.
% intent : n x >=2 voluntary handle angles (rad), columns [roll pitch].
% tremor : n x >=2 tremor handle angles (rad), columns [roll pitch].
% C      : control parameters (see the C.semg_* block in
%          rcflex_control_parameters.m).
% emg    : numel(tEmg) x 4 clean electrode voltages (V), channel order
%          [roll agonist, roll antagonist, pitch agonist, pitch antagonist].
% tEmg   : sample times on the ANALOGUE grid, spaced 1/(C.semg_hz *
%          C.semg_oversample). This function models the muscle and the
%          electrode, which are continuous-time; sampling happens in
%          rcflex_semg_sample.m, after the anti-alias filter. Generating here at
%          the final 4 kHz rate would be assuming the very thing the anti-alias
%          stage exists to guarantee -- that nothing lives above Nyquist -- so
%          the band-limiting filter below leaves genuine content above 2 kHz
%          for that stage to remove.
% activation: numel(tEmg) x 4 muscle activation envelopes in [0,1]; the
%          noise-free drive the interference pattern is modulated by. Returned
%          so the characterisation script can separate "what the muscle did"
%          from "what the electrode measured".
%
% Model. Surface EMG is the interference pattern of many motor-unit action
% potentials; over the bandwidth of interest it is well approximated as a
% band-limited zero-mean Gaussian process whose amplitude is modulated by the
% muscle activation (Hogan & Mann 1980; De Luca 1997). So:
%
%   emg(t) = activation(t) .* w(t),   w = 20-450 Hz band-limited unit-RMS noise
%
% Activation is driven by the joint torque the muscle must supply. For the
% voluntary component at ~0.2 Hz that is dominated by the posture (stiffness)
% term, so it follows the intent ANGLE; for the tremor component at 4-20 Hz it
% is dominated by the inertial term, so it follows the tremor ACCELERATION.
% Each axis is spanned by an agonist/antagonist pair: a signed drive loads one
% and unloads the other, over a co-contraction floor that rises with voluntary
% effort.
%
% Electromechanical delay. The muscle's electrical activity precedes the
% resulting movement by 30-50 ms (Cavanagh & Komi 1979). This is the single
% most important property of the signal for this project -- it is why sEMG can
% in principle supply tremor phase information that the IMU cannot yet know --
% so it is modelled explicitly as C.semg_emd_s of LEAD, not left implicit.
if nargin<5;seed=C.seed+8100;end
time=time(:);nch=4;
fsAnalog=C.semg_hz*C.semg_oversample;
dtEmg=1/fsAnalog;tEmg=(time(1):dtEmg:time(end))';

% Resample the drive onto the sEMG grid, advanced by the electromechanical
% delay so the electrical signal leads the mechanical one. Times past the end
% of the scenario are held at the final value rather than extrapolated.
tDrive=min(tEmg+C.semg_emd_s,time(end));
intentE=interp1(time,intent(:,1:2),tDrive,'linear');
tremorE=interp1(time,tremor(:,1:2),tDrive,'linear');

% Tremor acceleration on the sEMG grid. Differentiating the resampled signal
% (rather than resampling a differentiated one) keeps the lead exact.
tremorAcc=zeros(size(tremorE));
for j=1:2;tremorAcc(:,j)=gradient(gradient(tremorE(:,j),dtEmg),dtEmg);end

% Dimensionless drives, scaled so a reference-sized intent and a
% reference-sized tremor both produce a drive of order 1.
volRef=deg2rad(C.semg_intent_ref_deg);
treRef=deg2rad(C.semg_tremor_ref_deg)*(2*pi*C.semg_tremor_ref_Hz)^2;
driveVol=intentE/volRef;
driveTre=tremorAcc/treRef;
drive=C.semg_intent_gain*driveVol+C.semg_tremor_gain*driveTre;

% Agonist/antagonist split over a voluntary-effort-dependent co-contraction
% floor. Half-wave rectification of the signed drive is what makes the pair
% reciprocally active, which is the pattern the characterisation looks for.
co=C.semg_cocontraction+C.semg_cocontraction_gain*abs(driveVol);
activation=zeros(numel(tEmg),nch);
for j=1:2
 d=drive(:,j);c=co(:,j);
 activation(:,2*j-1)=c+max(d,0);
 activation(:,2*j)=c+max(-d,0);
end
activation=min(max(activation,0),C.semg_activation_max);

% Band-limited unit-RMS interference pattern, one independent realisation per
% channel. Deterministic in the scenario seed.
stream=RandStream('mt19937ar','Seed',mod(seed,2^32-1));
w=randn(stream,numel(tEmg),nch);
[bh,ah]=highpass2(C.semg_band_lo_Hz,fsAnalog);
[bl,al]=lowpass2(C.semg_band_hi_Hz,fsAnalog);
w=filter(bh,ah,w);w=filter(bl,al,w);
w=w./max(sqrt(mean(w.^2,1)),eps);

emg=activation.*w*C.semg_mvc_V;
end
function [b,a]=lowpass2(f,fs)
k=tan(pi*f/fs);den=1+sqrt(2)*k+k*k;
b=[k*k 2*k*k k*k]/den;a=[1 2*(k*k-1)/den (1-sqrt(2)*k+k*k)/den];
end
function [b,a]=highpass2(f,fs)
k=tan(pi*f/fs);den=1+sqrt(2)*k+k*k;
b=[1 -2 1]/den;a=[1 2*(k*k-1)/den (1-sqrt(2)*k+k*k)/den];
end
