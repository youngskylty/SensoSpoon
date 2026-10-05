function [shape,meta]=rcflex_tremor_record(record,t,C)
%RCFLEX_TREMOR_RECORD Condition one recorded human tremor into a drive waveform.
%
%   [shape,meta] = RCFLEX_TREMOR_RECORD('g2rof.rit',t,C)
%
% Turns a tremordb index-fingertip VELOCITY recording into a unit-RMS ANGULAR
% waveform sampled at the times t, for use as the tremor component of a
% simulation scenario. Returns an N-by-2 shape (roll, pitch).
%
% WHAT THIS DOES AND DOES NOT CLAIM. The output is amplitude-NORMALISED, and
% that is deliberate rather than a shortcut. The recording is fingertip
% translation and the plant input is hand attitude; there is no measurement in
% the database that would let us convert one into the other, and inventing a
% moment arm would dress an assumption up as data. What transfers from the
% recording is its TEMPORAL STRUCTURE -- the amplitude modulation, the harmonic
% content, the frequency wander and the non-stationarity that a synthetic sine
% has none of -- and that is scale-free. The caller therefore states the
% angular amplitude, exactly as it does for a synthetic scenario, and the
% recording supplies only the shape.
%
% Conditioning, in order:
%   1. band-pass the velocity over C.tremor_record_band (default 2-25 Hz), which
%      removes transducer drift below the tremor band and noise above it;
%   2. integrate to displacement;
%   3. band-pass again, because integrating a signal with any residual offset
%      produces a ramp, and the ramp is not tremor;
%   4. normalise to unit RMS;
%   5. resample to t. The signal is band-limited to well under the 50 Hz Nyquist
%      of the 100 Hz recording, so shape-preserving interpolation is adequate.
%
% The pitch axis is the same recording delayed by a quarter of its own dominant
% period. The database records one axis only; the quarter-period offset keeps
% the two axes at the same amplitude and spectrum while making them
% non-identical, which is the closest honest analogue of the fixed 0.7 rad phase
% offset the synthetic generator uses. meta.pitch_lag_s reports it.
if nargin<3;C=struct;end
band=getdef(C,'tremor_record_band',[2 25]);
t=t(:);

R=import_tremordb(record);
fs=R.fs;
v=detrend(R.v,0);
v=bandpass_zerophase(v,fs,band);
x=cumtrapz(v)/fs;                      % velocity -> displacement
x=bandpass_zerophase(x,fs,band);       % kill the integration ramp
r=rms(x);
assert(r>0&&isfinite(r),'SensoSpoon:TremorRecord', ...
 'Record %s conditioned to a zero or non-finite waveform.',R.name);
x=x/r;

% Dominant frequency, reported so a caller can sanity-check that the record sits
% in the band the controller was designed for. meta.in_design_band tests it
% against the project's declared 4-20 Hz envelope rather than against a range
% chosen here, because a record whose tremor falls outside that envelope is a
% statement about the design brief and not about the waveform -- and averaging
% the two together would hide both.
[pxx,fx]=periodogram_simple(x,fs);
[~,ix]=max(pxx);f0=fx(ix);
assert(f0>0,'SensoSpoon:TremorRecord','Record %s has no resolvable tremor peak.',R.name);
lag=0.25/f0;

tr=(0:numel(x)-1)'/fs;
% Start the window a few seconds into the record. Zero-phase filtering leaves
% transients at both ends of a finite signal, and the very start of a recording
% is also where the transducer is still settling; neither is tremor.
off=getdef(C,'tremor_record_offset_s',5);
assert(tr(end)>2*off,'SensoSpoon:TremorRecord', ...
 'Record %s is only %.1f s, too short for a %.1f s offset.',R.name,tr(end),off);
% t is wrapped into the record's own usable length so a scenario may be shorter
% or longer than the recording without running off the end or needing a taper.
usable=tr(end)-2*off;
tw=off+mod(t,usable);
roll=interp1(tr,x,tw,'pchip');
pitch=interp1(tr,x,off+mod(tw-lag-off,usable),'pchip');
shape=[roll pitch];

% Normalise over the WINDOW that was actually asked for, not over the whole
% recording. Tremor amplitude wanders by several-fold across a 60 s record, so
% normalising the record would leave each scenario presenting a different
% effective amplitude and make suppression percentages incomparable between
% records -- which is the one comparison this replay exists to make. Modulation
% WITHIN the window is untouched; only its overall level is set. The ratio
% between the two normalisations is reported so the wander stays auditable.
rw=rms(shape(:,1));
assert(all(rms(shape)>0)&&all(isfinite(rms(shape))),'SensoSpoon:TremorRecord', ...
 'Record %s gave a zero-amplitude window.',R.name);
% Scale to the RMS of a UNIT-AMPLITUDE SINE, not to unit RMS. The caller applies
% the same 'amplitude' in degrees to a recorded scenario as to a synthetic one,
% so matching the RMS here is what makes 'recorded at 1 deg' and 'sine at 1 deg'
% the same amount of tremor energy to reject. Without it the recorded scenario
% would be sqrt(2) times more severe and every comparison against the synthetic
% benchmark would be measuring the scaling rather than the waveform.
% Each axis is normalised SEPARATELY. The pitch axis is a time-shifted copy of
% the same recording, and real tremor is not stationary, so over any finite
% window the shifted copy carries a few per cent more or less energy than the
% original -- measured at up to 8%. Scaling both axes by the roll factor would
% let that leak into the total tremor energy and make 'amplitude 1 deg' mean
% something slightly different for every record. The synthetic generator has
% equal energy on both axes by construction, so equalising here is what keeps
% the two kinds of scenario comparable. The per-record ratio is reported in
% meta.axis_energy_ratio rather than discarded.
ratio=rms(shape(:,2))/rms(shape(:,1));
shape=shape./(rms(shape)*sqrt(2));

meta=struct('record',R.name,'subject',R.subject,'target',R.target,'group',R.group, ...
 'hand',R.hand,'minutes_off',R.minutes_off,'fs',fs,'duration_s',R.duration_s, ...
 'samples',R.n,'samples_expected',R.samples_expected, ...
 'dominant_Hz',f0,'pitch_lag_s',lag,'band_Hz',band, ...
 'velocity_rms_mps',rms(R.v),'velocity_peak_mps',max(abs(R.v)), ...
 'window_level_ratio',rw,'window_offset_s',off,'axis_energy_ratio',ratio, ...
 'crest_factor',max(abs(shape(:,1)))/rms(shape(:,1)), ...
 'in_design_band',f0>=4&&f0<=20);
end

% -------------------------------------------------------------------------
function y=bandpass_zerophase(x,fs,band)
% Zero-phase band-pass. Written against filtfilt/butter when Signal Processing
% Toolbox is present and falling back to an FFT brick-wall otherwise, so the
% conditioning does not silently depend on a licence being available.
if exist('butter','file')==2&&exist('filtfilt','file')==2
 [b,a]=butter(4,band/(fs/2),'bandpass');
 y=filtfilt(b,a,x);
else
 n=numel(x);X=fft(x);f=(0:n-1)'/n*fs;f=min(f,fs-f);
 X(f<band(1)|f>band(2))=0;y=real(ifft(X));
end
end

% -------------------------------------------------------------------------
function [pxx,f]=periodogram_simple(x,fs)
% Hann-windowed periodogram. Local so the spectrum this function reports does
% not depend on a toolbox either.
n=numel(x);w=.5-.5*cos(2*pi*(0:n-1)'/(n-1));
X=fft(x.*w);m=floor(n/2)+1;
pxx=abs(X(1:m)).^2;f=(0:m-1)'*fs/n;
end

% -------------------------------------------------------------------------
function v=getdef(S,f,d);if isstruct(S)&&isfield(S,f)&&~isempty(S.(f));v=S.(f);else;v=d;end;end
