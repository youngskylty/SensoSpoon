function Q=rcflex_semg_quality(raw,C,verbose)
%RCFLEX_SEMG_QUALITY Source-agnostic health check on a raw sEMG stream.
% Runs on whatever rcflex_semg_source returns -- synthetic, recorded or
% captured from the physical front end -- and reports the acquisition-side
% properties that decide whether the stream is usable at all, BEFORE any
% question about what it contains. These are the checks to run first on a real
% dataset: they catch a badly set gain, a railed channel, a dead electrode, a
% mains problem or a wrong sample rate, all of which look like "sEMG is
% uninformative" if you skip straight to feature extraction.
%
% Per channel, Q is a table with:
%   rms_uV, peak_uV       electrode-referred amplitude
%   crest                 peak/rms. Drives the gain choice: gain must be set
%                         from the PEAK, not the rms, or the peaks clip.
%   clip_percent          samples sitting on the ADC rail. Detected by value,
%                         so it works on a recording whose gain is unknown.
%   headroom_dB           20*log10(full scale / peak). Negative means the
%                         signal is already clipping; a healthy channel sits
%                         around 3-10 dB.
%   oob_nV_rtHz         noise floor as an amplitude spectral density, measured
%                         ABOVE the EMG band (band_hi to Nyquist). It cannot be
%                         measured inside 20-450 Hz: there is no quiet region
%                         there, so an in-band estimate reads back the signal
%                         and reports a meaninglessly low SNR.
%                         Quoted this way because it is directly comparable
%                         with the input-referred noise density on an amplifier
%                         datasheet, which is how the gain stage gets chosen.
%   oob_snr_dB                rms over that floor integrated across 20-450 Hz.
%   mains_dB              power at C.semg_mains_Hz relative to the 20-450 Hz
%                         band total. Large values mean the notch will be doing
%                         structural work rather than cosmetic tidying.
%   band_frac             fraction of 0-Nyquist power inside 20-450 Hz. A low
%                         value means the stream is dominated by out-of-band
%                         content (offset drift, motion artefact) rather than
%                         EMG.
if nargin<3;verbose=true;end
n=size(raw,1);nch=size(raw,2);fs=C.semg_hz;

fullScale=C.semg_adc_range_V/C.semg_gain;
quantum=2*C.semg_adc_range_V/2^C.semg_adc_bits/C.semg_gain;

rms_uV=zeros(nch,1);peak_uV=zeros(nch,1);crest=zeros(nch,1);
clip_percent=zeros(nch,1);headroom_dB=zeros(nch,1);oob_nV_rtHz=zeros(nch,1);
oob_snr_dB=zeros(nch,1);mains_dB=zeros(nch,1);band_frac=zeros(nch,1);
bandWidth=C.semg_band_hi_Hz-C.semg_band_lo_Hz;

nfft=2^nextpow2(min(n,4*fs));
f=(0:nfft/2)'*fs/nfft;
inBand=f>=C.semg_band_lo_Hz&f<=C.semg_band_hi_Hz;
% Noise-floor band: above the EMG content, below Nyquist. Falls back to the top
% quarter of the spectrum if the sample rate leaves little room above the band.
floorLo=min(C.semg_band_hi_Hz*1.3,0.7*fs/2);
hiBand=f>=floorLo&f<=0.95*fs/2;
if nnz(hiBand)<8;hiBand=f>=0.75*fs/2;end
mainsBin=abs(f-C.semg_mains_Hz)<=2;

for ch=1:nch
 x=raw(:,ch);
 rms_uV(ch)=1e6*sqrt(mean(x.^2));
 peak_uV(ch)=1e6*max(abs(x));
 crest(ch)=peak_uV(ch)/max(rms_uV(ch),eps);
 % A clipped sample sits within one quantum of the rail.
 clip_percent(ch)=100*mean(abs(x)>=fullScale-quantum);
 headroom_dB(ch)=20*log10(fullScale/max(max(abs(x)),eps));

 seg=x(1:min(n,nfft));win=hann_local(numel(seg));seg=detrend(seg).*win;
 % One-sided PSD, properly normalised so the result is V^2/Hz and Parseval
 % holds -- without this the "floor" is an arbitrary number that happens to
 % scale with the signal, which is worse than not reporting it.
 X=fft(seg,nfft);P=abs(X(1:nfft/2+1)).^2;
 P(2:end-1)=2*P(2:end-1);
 P=P/(fs*sum(win.^2));
 band_frac(ch)=sum(P(inBand))/max(sum(P),eps);
 mains_dB(ch)=10*log10(sum(P(mainsBin))/max(sum(P(inBand)),eps));
 dens=mean(P(hiBand));                      % V^2/Hz in the quietest band
 oob_nV_rtHz(ch)=1e9*sqrt(dens);
 floorRms=sqrt(dens*bandWidth);             % equivalent in-band noise rms
 oob_snr_dB(ch)=20*log10(1e-6*rms_uV(ch)/max(floorRms,eps));
end

channel=(1:nch)';
Q=table(channel,rms_uV,peak_uV,crest,clip_percent,headroom_dB,oob_nV_rtHz,oob_snr_dB,mains_dB,band_frac);

if verbose
 fprintf('\nsEMG stream quality (%d ch, %g Hz, %.2f s, full scale %.2f mV):\n',...
  nch,fs,n/fs,1e3*fullScale);
 disp(Q);
 flag(any(Q.clip_percent>0.1),sprintf('CLIPPING: up to %.2f%% of samples on the rail -- reduce analogue gain',max(Q.clip_percent)));
 flag(any(Q.headroom_dB<0),'NEGATIVE HEADROOM: peak exceeds full scale');
 flag(any(Q.headroom_dB>20),'EXCESSIVE HEADROOM: gain could be raised to use the converter');
 flag(any(Q.rms_uV<1),'DEAD CHANNEL: rms below 1 uV -- check electrode contact');
 flag(any(Q.mains_dB>-6),'MAINS DOMINANT: interference comparable to the EMG band');
 flag(any(Q.band_frac<0.5),'OUT-OF-BAND DOMINANT: most power outside 20-450 Hz -- offset or motion artefact');
end
end
function flag(cond,msg)
if cond;fprintf('  [!] %s\n',msg);end
end
function w=hann_local(n)
w=.5*(1-cos(2*pi*(0:n-1)'/(n-1)));
end
