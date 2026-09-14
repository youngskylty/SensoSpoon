function [measurement,tMeas]=rcflex_semg_sample(emg,tEmg,C,seed)
%RCFLEX_SEMG_SAMPLE Instrumentation model for the sEMG front end.
% Mirrors the role rcflex_imu_sample.m plays for the IMU: takes clean sensor
% truth and returns what the controller actually gets to see, with every
% corruption an explicit, named design scenario rather than an implicit
% assumption. Vectorised over the whole scenario because the sEMG stream is
% generated offline with the scenario, unlike the IMU which is sampled inside
% the loop.
%
% emg  : n x 4 clean electrode voltages (V) from rcflex_semg_truth, on the
%        ANALOGUE grid (C.semg_hz * C.semg_oversample).
% tEmg : n x 1 matching analogue-grid times (s).
% measurement : m x 4 quantised electrode-referred voltages (V) at C.semg_hz,
%        i.e. the ADC reading divided back through the instrumentation gain, so
%        the preprocessing chain works in physical units.
% tMeas : m x 1 sample times at C.semg_hz.
%
% Signal chain, in physical order:
%   electrode -> corruptions -> AC coupling -> gain -> anti-alias -> decimate
%   -> quantise/clip
% Getting this order right is the point of the function. AC coupling must
% precede gain (1.5) and anti-aliasing must precede sampling (see below), and
% both constraints are on the ORDER, not on the individual blocks.
%
% The corruptions modelled, and why each is here:
%
% * Electrode DC offset (C.semg_offset_V). Half-cell potential at the
%   skin-gel interface, orders of magnitude larger than the EMG itself. This
%   is the reason the preprocessing chain MUST lead with a highpass.
% * Mains interference at C.semg_mains_Hz. 50 Hz in New Zealand, and it sits
%   squarely inside the 20-450 Hz EMG band, so it cannot be removed by band
%   limiting alone -- hence the notch in rcflex_semg_preprocess.m.
% * Motion artefact. Low-frequency (<10 Hz) baseline wander from electrode
%   movement and cable sway. It is deliberately placed IN the tremor band,
%   because a motion artefact at tremor frequency is precisely the failure
%   mode that would make sEMG look more informative than it is.
% * Broadband amplifier/electrode thermal noise, which sets the floor at rest.
% * Finite ADC range and resolution, with saturation.
%
% Anti-aliasing. Everything above is computed on an oversampled grid so that
% the anti-alias filter has genuine out-of-band content to remove: the truth
% generator's 450 Hz band limit is only 2-pole, and the corruption noise is
% white, so real power exists above the 2 kHz Nyquist of the final rate.
% Without oversampling the model would ASSUME the band limiting that this stage
% exists to enforce, and the filter would be a decorative block that could never
% be shown to earn its place. Two poles at C.semg_aa_Hz is the simplest filter
% consistent with the 20-450 Hz requirement and a 4x oversample ratio.
if nargin<4;seed=C.seed+8200;end
tEmg=tEmg(:);n=numel(tEmg);nch=size(emg,2);
os=C.semg_oversample;fsAnalog=C.semg_hz*os;
stream=RandStream('mt19937ar','Seed',mod(seed,2^32-1));

% Per-channel DC offsets and mains phases: fixed for a session, not per sample.
offset=C.semg_offset_V*(2*rand(stream,1,nch)-1);
mainsPhase=2*pi*rand(stream,1,nch);
mains=C.semg_mains_V*sin(2*pi*C.semg_mains_Hz*tEmg+mainsPhase);

% Motion artefact: narrowband wander inside the tremor band, independent per
% channel in both frequency and phase so it cannot be mistaken for a common
% physiological rhythm.
artFreq=C.semg_artifact_Hz_range(1)+diff(C.semg_artifact_Hz_range)*rand(stream,1,nch);
artPhase=2*pi*rand(stream,1,nch);
artifact=C.semg_artifact_V*sin(2*pi*artFreq.*tEmg+artPhase);

noise=C.semg_noise_V*randn(stream,n,nch);

raw=emg+offset+mains+artifact+noise;

% Analog AC coupling, ahead of the gain stage. This is not cosmetic: the
% electrode half-cell offset is ~10x the EMG itself, so at C.semg_gain it would
% demand ~7.5 V of a +/-1.65 V converter and rail the channel solid. Every real
% sEMG front end therefore AC-couples (or runs a DC servo) BEFORE amplifying,
% and the model has to do the same or the offset silently destroys whichever
% channels happen to draw a large one. A single-pole highpass is the standard
% arrangement; the digital highpass in rcflex_semg_preprocess.m then removes
% the residual wander this one leaves behind.
p=exp(-2*pi*C.semg_ac_couple_Hz/C.semg_hz);
% Started from the steady state for the initial DC level, not from zero. A
% zero-state highpass sees the electrode offset arrive as a step at t=0 and
% rings at nearly the full offset amplitude -- ~14.6 mV of a 3.3 mV full scale,
% which rails every channel for the first fraction of a second REGARDLESS of
% gain, and so makes the gain look unfixable. Physically the front end is
% powered up and settled long before acquisition starts, so the settled state
% is the correct initial condition.
zi=-p*raw(1,:);
coupled=filter([1 -1]*p,[1 -p],raw,zi);

% Gain stage.
amplified=coupled*C.semg_gain;

% Analogue anti-alias filter, BEFORE sampling. Cascaded 2-pole Butterworth
% sections (C.semg_aa_poles is a multiple of 2), corner C.semg_aa_Hz.
c=lowpass2(C.semg_aa_Hz,fsAnalog);
filtered=amplified;
for k=1:round(C.semg_aa_poles/2)
 filtered=filter(c(1:3),[1 c(4:5)],filtered);
end

% Sample: decimate the analogue grid down to C.semg_hz. The ratio is an exact
% integer, so this is a pure decimation with no interpolation.
ix=1:os:n;
sampled=filtered(ix,:);tMeas=tEmg(ix);

% ADC. Quantised and clipped in converter units, then referred back to the
% electrode so downstream code is independent of the gain actually used.
adc=max(min(sampled,C.semg_adc_range_V),-C.semg_adc_range_V);
quantum=2*C.semg_adc_range_V/2^C.semg_adc_bits;
measurement=round(adc/quantum)*quantum/C.semg_gain;
end
function c=lowpass2(f,fs)
k=tan(pi*f/fs);den=1+sqrt(2)*k+k*k;
c=[k*k 2*k*k k*k]/den;c=[c 2*(k*k-1)/den (1-sqrt(2)*k+k*k)/den];
end
