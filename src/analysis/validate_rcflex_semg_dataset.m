function [R,Dg]=validate_rcflex_semg_dataset(dataset,C,writeOutputs)
%VALIDATE_RCFLEX_SEMG_DATASET Run the six real-data stages, in order.
% The standalone real-sEMG validation gate. It answers one question -- is this
% signal worth fusing with the IMU? -- and it answers it BEFORE any estimator
% work, using the same preprocessing, features and quality checks the synthetic
% source goes through.
%
% dataset : a struct or .mat path meeting rcflex_semg_dataset_spec, OR a
%           scenario struct from rcflex_control_case (so the synthetic source
%           can be run through the identical gate as a control).
% R       : table of stage / status / value / threshold / note. Always the six
%           GATE stages, in order, and nothing else -- diagnostics never appear
%           here, so a row index is a stable reference to a stage.
% Dg      : diagnostics that are reported but do NOT gate. Carries the stage 3
%           per-segment detail (every segment's signed r, |r| and cross-
%           correlation peak lag) and stage 3b coactivation_corr.
%
% STAGE 3 AND STAGE 3b ARE DIFFERENT PHYSICAL QUANTITIES.
%   stage 3  raw band-limited waveform correlation, zero lag
%            -> acquisition/electrical cross-talk. GATED.
%   stage 3b envelope correlation within a condition (coactivation_corr)
%            -> physiological agonist/antagonist co-activation. DIAGNOSTIC.
% Envelope correlation must never be used as the cross-talk gate: the synthetic
% source generates its channels INDEPENDENTLY -- zero cross-talk by
% construction -- and still shows envelope |r| ~ 0.8, because both channels
% share the same activation drive.
%
% Stages run in the documented order, and each reports one of:
%   PASS     criterion met
%   FAIL     criterion not met -- stop and fix before continuing
%   SKIPPED  a required dataset field is absent; the note names it
%
% SKIPPED is deliberately distinct from PASS. A stage that could not run has
% told you nothing, and the most expensive failure mode at this point would be
% reading an absent measurement as a satisfied one.
%
% Reference values from the synthetic source (docs/SEMG_SUBSYSTEM.md 10.4) are
% carried in the note column so real numbers can be read against something.
if nargin<2||isempty(C)
 C=rcflex_control_parameters;C.semg_enabled=true;C=rcflex_prepare_control(C);
end
if nargin<3;writeOutputs=true;end
if ~isfield(C,'semg');C.semg_enabled=true;C=rcflex_prepare_control(C);end

[raw,tRaw,meta,labels,imu,imuFs]=resolve(dataset,C);
rows={};

% =========================================================================
% 1. Acquisition health
% =========================================================================
Q=rcflex_semg_quality(raw,C,false);
ok=all(Q.clip_percent<0.1)&&all(Q.headroom_dB>=0)&&all(Q.rms_uV>1)&&...
   all(Q.band_frac>0.5)&&all(Q.mains_dB<-6);
rows(end+1,:)={'1 acquisition health',stat(ok),max(Q.clip_percent),0.1,...
 sprintf('clip%%<0.1, headroom %.1f-%.1f dB, band_frac %.2f, mains %.1f dB',...
 min(Q.headroom_dB),max(Q.headroom_dB),min(Q.band_frac),max(Q.mains_dB))};

% Features are computed once and reused by every stage below.
[F,tTick]=rcflex_semg_features(raw,C,tRaw(1));
nch=C.semg.nch;burst=F(:,nch+1:end);env=F(:,1:nch);

% =========================================================================
% 2. Burst-band SNR, tremor vs tremor-free  (the first go/no-go)
% =========================================================================
[ixT,ixQ,why]=tremor_segments(labels,tTick);
if isempty(ixT)||isempty(ixQ)
 rows(end+1,:)={'2 burst SNR (dB)','SKIPPED',NaN,6,why};
else
 pT=mean(mean(burst(ixT,:).^2,1));pQ=mean(mean(burst(ixQ,:).^2,1));
 snr=10*log10(pT/max(pQ,eps));
 rows(end+1,:)={'2 burst SNR (dB)',stat(snr>6),snr,6,'synthetic reference: 8.0-12.7 dB'};
end

% =========================================================================
% 3. Raw-waveform cross-talk between the paired channels        [GATE]
% =========================================================================
% WHAT THIS MEASURES: acquisition-side ELECTRICAL cross-talk -- the same motor
% unit action potentials reaching both electrodes of a pair by volume
% conduction, plus whatever common-mode the bipolar montage failed to reject.
% Volume conduction is quasi-static at EMG frequencies, so genuine cross-talk
% appears in the RAW waveform at ZERO LAG. That is what is gated here.
%
% WHAT IT DELIBERATELY DOES NOT MEASURE: physiological co-activation. Two
% muscles driven together have correlated ENVELOPES while their raw interference
% patterns stay independent, because their motor unit pools are independent.
% Envelope correlation therefore measures shared neural drive, not recording
% independence, and it is reported separately as stage 3b. Do not conflate them.
%
% WHY CONDITIONS ARE NOT POOLED: flexion, extension and rest carry relationships
% of opposite sign, so one correlation over a whole multi-condition record
% cancels to near zero regardless of the underlying cross-talk. Each isolated
% contraction segment is scored on its own and the WORST case gates, because
% cross-talk is a property of electrode placement and a single badly sited pair
% breaks the agonist/antagonist feature layout that a median would hide.
%
% THE THRESHOLD IS PROVISIONAL. 0.30 is an engineering gate derived from the
% distance-decay structure of volume conduction and the residual common-mode
% floor of a shared-reference montage; it is NOT a clinically established
% universal cross-talk limit. Every segment's value is reported so it can be
% reassessed against our own recordings. It must not be retuned to make a
% dataset pass.
xtThresh=0.30;
[segs,why3]=isolated_segments(labels,tRaw);
S3=stage3_segments(raw,tRaw,env,tTick,segs,C);
if isempty(S3)
 if isempty(why3);why3='no isolated segment yielded two non-degenerate channels';end
 worst=NaN;med=NaN;
 rows(end+1,:)={'3 raw cross-talk |r|','SKIPPED',NaN,xtThresh,why3};
else
 av=[S3.abs_r];worst=max(av);med=median(av);
 rows(end+1,:)={'3 raw cross-talk |r|',stat(worst<xtThresh),worst,xtThresh,...
  sprintf('worst of %d isolated segment-axes, median %.3f; zero-lag raw |r|; PROVISIONAL gate, not a clinical limit',...
  numel(S3),med)};
end

% =========================================================================
% 4. sEMG lead over the mechanical tremor
% =========================================================================
% The single measurement that justifies fusion at all: it is the only thing
% sEMG offers that a better IMU-only estimator could not also provide.
if isempty(imu)
 rows(end+1,:)={'4 sEMG lead (ms)','SKIPPED',NaN,10,'needs imu_accel + imu_fs'};
else
 leads=zeros(2,1);
 for ax=1:2
  m=interp1((0:size(imu,1)-1)'/imuFs,imu(:,ax),tTick,'linear',0);
  leads(ax)=lead_of(burst(:,2*ax-1),m,1/C.outer_hz,dominant_freq(m,C.outer_hz));
 end
 L=1000*mean(leads);
 rows(end+1,:)={'4 sEMG lead (ms)',stat(L>10&&L<100),L,10,...
  sprintf('roll %.1f, pitch %.1f ms; synthetic reference: +35 to +37.5 ms',1000*leads(1),1000*leads(2))};
end

% =========================================================================
% 5. Frequency agreement with the mechanical tremor
% =========================================================================
if isempty(imu)
 rows(end+1,:)={'5 frequency agreement (Hz)','SKIPPED',NaN,1,'needs imu_accel + imu_fs'};
else
 d=zeros(2,1);
 for ax=1:2
  m=interp1((0:size(imu,1)-1)'/imuFs,imu(:,ax),tTick,'linear',0);
  d(ax)=abs(dominant_freq(burst(:,2*ax-1),C.outer_hz)-dominant_freq(m,C.outer_hz));
 end
 dm=max(d);
 rows(end+1,:)={'5 frequency agreement (Hz)',stat(dm<1),dm,1,...
  'agreement here is what would establish sEMG as the independent omega observable'};
end

% =========================================================================
% 6. Intent / tremor discrimination
% =========================================================================
[ixI,ixTr,why6]=intent_segments(labels,tTick);
if isempty(ixI)||isempty(ixTr)
 rows(end+1,:)={'6 intent/tremor discrimination','SKIPPED',NaN,2,why6};
else
 % Envelope should track voluntary effort; burst should track tremor. The
 % discriminating quantity is the RATIO of the two feature bands between the
 % two conditions -- either feature alone moves with overall effort.
 rI=mean(mean(burst(ixI,:).^2,1))/max(mean(mean(env(ixI,:).^2,1)),eps);
 rT=mean(mean(burst(ixTr,:).^2,1))/max(mean(mean(env(ixTr,:).^2,1)),eps);
 sep=rT/max(rI,eps);
 rows(end+1,:)={'6 intent/tremor discrimination',stat(sep>2),sep,2,...
  'burst/envelope power ratio, tremor segments over intent segments'};
end

R=cell2table(rows,'VariableNames',{'stage','status','value','threshold','note'});

% Diagnostics. Reported, never gated -- see the header note on stage 3 vs 3b.
Dg=struct('stage3_threshold',xtThresh,'stage3_worst',worst,'stage3_median',med,...
 'stage3_segments',seg_table(S3),'coactivation',coact_summary(S3));

fprintf('\n=== Real-sEMG validation: %s ===\n',describe(meta));
disp(R(:,{'stage','status','value','threshold'}));
for k=1:height(R)
 fprintf('  %-32s %s\n',R.stage{k},R.note{k});
end

if ~isempty(S3)
 fprintf('\n-- stage 3 detail: raw zero-lag |r| per isolated segment (GATE) --\n');
 disp(Dg.stage3_segments);
 fprintf('  worst-case |r| %.3f (gates, threshold %.2f provisional), median |r| %.3f\n',...
  worst,xtThresh,med);
 % Only worth flagging where the correlation is substantial enough for its peak
 % to mean anything: on a near-zero correlation the peak lag is just noise, and
 % warning about it would train the reader to ignore the warning.
 lg=[S3.peak_lag_ms];av=[S3.abs_r];
 odd=abs(lg)>1&av>=0.1;
 if any(odd)
  fprintf(['  [!] %d of %d segment-axes with |r| >= 0.1 peak away from zero lag (max |lag| %.1f ms).\n' ...
           '      Correlation peaking off zero lag is NOT volume-conducted cross-talk;\n' ...
           '      do not reinterpret it as such. The gate uses zero lag regardless.\n'],...
   nnz(odd),numel(lg),max(abs(lg(odd))));
 end
 fprintf('\n-- stage 3b: agonist/antagonist co-activation (DIAGNOSTIC, does not gate) --\n');
 ca=Dg.coactivation;
 fprintf('  coactivation_corr  roll   median %s   pitch median %s   overall |r| median %s\n',...
  num2str(ca.roll_median,'%.3f'),num2str(ca.pitch_median,'%.3f'),num2str(ca.abs_median,'%.3f'));
 fprintf('  Envelope correlation measures shared neural drive, NOT cross-talk.\n');
end
nf=sum(strcmp(R.status,'FAIL'));ns=sum(strcmp(R.status,'SKIPPED'));
fprintf('\n%d passed, %d failed, %d skipped.\n',sum(strcmp(R.status,'PASS')),nf,ns);
if nf>0
 fprintf('STOP: fix the failing stage before proceeding toward fusion.\n');
elseif ns>0
 fprintf('No failures, but %d stage(s) could not run. Fusion is NOT yet justified --\n',ns);
 fprintf('a skipped stage is an unanswered question, not a passed one.\n');
else
 fprintf('All stages passed. The evidence for IMU+sEMG fusion is in place.\n');
end

if writeOutputs
 out=project_path('results','semg_dataset_validation.csv');
 writetable(R,out);fprintf('Wrote %s\n',out);
 if ~isempty(S3)
  % Per-segment detail is written out so the provisional 0.30 gate can be
  % reassessed later against our own recordings without re-deriving it.
  out3=project_path('results','semg_stage3_segments.csv');
  writetable(Dg.stage3_segments,out3);fprintf('Wrote %s\n',out3);
 end
end
end
% -------------------------------------------------------------------------
function [raw,tRaw,meta,labels,imu,imuFs]=resolve(dataset,C)
% Accepts a recorded dataset, a path, or a synthetic scenario struct, so the
% synthetic source can be pushed through the identical gate as a control.
labels=[];imu=[];imuFs=[];
if isstruct(dataset)&&isfield(dataset,'semg')&&isfield(dataset,'time')&&isfield(dataset,'tremor')
 % A rcflex_control_case scenario: synthesise the optional fields from truth.
 raw=dataset.semg;tRaw=dataset.semg_time;
 meta=struct('source','synthetic scenario','name',dataset.name);
 imuFs=1/(dataset.time(2)-dataset.time(1));
 imu=[gradient(gradient(dataset.tremor(:,1),dataset.time),dataset.time),...
      gradient(gradient(dataset.tremor(:,2),dataset.time),dataset.time)];
 labels=labels_from_truth(dataset);
 return
end
Cf=C;Cf.semg_source='file';
D=struct('semg_dataset',dataset);
[raw,tRaw,meta]=rcflex_semg_source(D,Cf);
if isfield(meta,'labels');labels=meta.labels;end
if isfield(meta,'imu_accel');imu=meta.imu_accel;imuFs=meta.imu_fs;end
end
function L=labels_from_truth(D)
% Segment a synthetic scenario by which components are actually active, so the
% control run exercises the same label-driven code path a recording will.
t=D.time;half=t>t(1)+1.5;
hasT=any(abs(D.tremor(half,1:2))>deg2rad(0.05),'all');
hasI=any(abs(D.intent(half,1:2))>deg2rad(0.05),'all');
if hasT&&hasI;cls='both';elseif hasT;cls='tremor';elseif hasI;cls='intent';else;cls='rest';end
L=struct('t_start',t(1)+1.5,'t_end',t(end),'class',cls);
end
function [segs,why]=isolated_segments(L,tRaw)
%ISOLATED_SEGMENTS Valid isolated single-muscle contraction windows for stage 3.
% Per docs/SEMG_SUBSYSTEM.md 6.3 the cued "isolated flexion, then isolated
% extension" trials are the ones logged with class 'intent'. Those are the only
% segments that isolate cross-talk: one muscle of the pair should be silent, so
% whatever appears in its channel arrives by some route other than its own motor
% units. 'rest' carries no signal to correlate, and 'tremor'/'both' are not
% isolated contractions, so neither can score this stage.
%
% The first SKIP seconds of every segment are discarded, excluding the
% contraction ramp and any filter transient at a trial join.
segs=struct('t_start',{},'t_end',{},'class',{});
why='needs labels with at least one isolated contraction segment (class ''intent'')';
if isempty(L)
 why='needs labels; stage 3 scores isolated contraction segments (class ''intent'') only';
 return
end
if istable(L);L=table2struct(L);end
SKIP=1.5;MINLEN=0.25;
for k=1:numel(L)
 if ~strcmpi(char(L(k).class),'intent');continue;end
 a=max(L(k).t_start+SKIP,tRaw(1));b=min(L(k).t_end,tRaw(end));
 if b-a<MINLEN;continue;end
 segs(end+1)=struct('t_start',a,'t_end',b,'class',char(L(k).class)); %#ok<AGROW>
end
if ~isempty(segs);why='';end
end
function S3=stage3_segments(raw,tRaw,env,tTick,segs,C)
%STAGE3_SEGMENTS Per-segment, per-axis raw cross-talk and co-activation.
% Returns one row per (isolated segment x axis). raw_r is the GATED quantity:
% zero-lag Pearson correlation of the band-limited raw waveforms. peak_lag_ms
% and coactivation_corr are diagnostics carried alongside it, never gated.
S3=struct('segment',{},'class',{},'axis',{},'t_start',{},'t_end',{},...
 'raw_r',{},'abs_r',{},'peak_lag_ms',{},'coactivation_corr',{});
if isempty(segs);return;end
rawBL=bandlimit_raw(raw,C);
maxLagS=max(1,round(0.020*C.semg_hz));   % +/-20 ms is ample for volume conduction
axName={'roll','pitch'};
for k=1:numel(segs)
 mr=tRaw>=segs(k).t_start&tRaw<=segs(k).t_end;
 me=tTick>=segs(k).t_start&tTick<=segs(k).t_end;
 for ax=1:2
  a=rawBL(mr,2*ax-1);b=rawBL(mr,2*ax);
  if numel(a)<8||std(a)<eps||std(b)<eps;continue;end
  r=corr(a,b);                                  % zero lag -- this is the gate
  [c,lg]=xcorr(detrend(a),detrend(b),maxLagS,'normalized');
  [~,j]=max(abs(c));lagMs=1000*lg(j)/C.semg_hz; % diagnostic only
  ca=NaN;
  if nnz(me)>2
   ea=env(me,2*ax-1);eb=env(me,2*ax);
   if std(ea)>eps&&std(eb)>eps;ca=corr(ea,eb);end
  end
  S3(end+1)=struct('segment',k,'class',segs(k).class,'axis',axName{ax},...
   't_start',segs(k).t_start,'t_end',segs(k).t_end,...
   'raw_r',r,'abs_r',abs(r),'peak_lag_ms',lagMs,'coactivation_corr',ca); %#ok<AGROW>
 end
end
end
function y=bandlimit_raw(x,C)
%BANDLIMIT_RAW Restrict to the pipeline's EMG band before correlating.
% Zero-phase, so the lag diagnostic is not biased by the filter itself. This is
% offline validation, not the causal control path.
fs=C.semg_hz;wn=[C.semg_band_lo_Hz C.semg_band_hi_Hz]/(fs/2);
wn(2)=min(wn(2),0.99);
[b,a]=butter(4,wn,'bandpass');
y=filtfilt(b,a,x);
end
function T=seg_table(S3)
if isempty(S3)
 T=cell2table(cell(0,9),'VariableNames',{'segment','class','axis','t_start',...
  't_end','raw_r','abs_r','peak_lag_ms','coactivation_corr'});
else
 T=struct2table(S3,'AsArray',true);
end
end
function s=coact_summary(S3)
%COACT_SUMMARY Stage 3b. Envelope correlation = physiological co-activation.
% Reported per axis and overall. This is NOT cross-talk and never gates.
s=struct('roll_median',NaN,'pitch_median',NaN,'abs_median',NaN,'n',0);
if isempty(S3);return;end
ca=[S3.coactivation_corr];ax=string({S3.axis});
s.n=nnz(isfinite(ca));
if s.n==0;return;end
s.roll_median=median(ca(ax=="roll"&isfinite(ca)));
s.pitch_median=median(ca(ax=="pitch"&isfinite(ca)));
s.abs_median=median(abs(ca(isfinite(ca))));
end
function [ixT,ixQ,why]=tremor_segments(L,tTick)
% Tremor-bearing samples versus tremor-free ones.
ixT=[];ixQ=[];why='needs labels with a tremor-bearing and a tremor-free segment';
if isempty(L);return;end
if istable(L);L=table2struct(L);end
mT=false(size(tTick));mQ=false(size(tTick));
for k=1:numel(L)
 m=tTick>=L(k).t_start&tTick<=L(k).t_end;c=lower(char(L(k).class));
 if ismember(c,{'tremor','both'});mT=mT|m;else;mQ=mQ|m;end
end
if any(mT)&&any(mQ);ixT=find(mT);ixQ=find(mQ);why='';end
end
function [ixI,ixT,why]=intent_segments(L,tTick)
ixI=[];ixT=[];why='needs labels with both an intent-only and a tremor-bearing segment';
if isempty(L);return;end
if istable(L);L=table2struct(L);end
mI=false(size(tTick));mT=false(size(tTick));
for k=1:numel(L)
 m=tTick>=L(k).t_start&tTick<=L(k).t_end;c=lower(char(L(k).class));
 if strcmp(c,'intent');mI=mI|m;elseif ismember(c,{'tremor','both'});mT=mT|m;end
end
if any(mI)&&any(mT);ixI=find(mI);ixT=find(mT);why='';end
end
function s=stat(ok)
if ok;s='PASS';else;s='FAIL';end
end
function s=describe(meta)
s=meta.source;
if isfield(meta,'name');s=[s ' (' char(meta.name) ')'];end
if isfield(meta,'subject');s=[s ' subject ' char(string(meta.subject))];end
end
function L=lead_of(a,m,dt,fHz)
% Positive => a leads m. Window capped at half a tremor period, so a lead and a
% lag of the opposite sign cannot be confused in a periodic signal.
if std(a)<eps||std(m)<eps||~isfinite(fHz);L=NaN;return;end
maxLag=max(1,floor(.5/max(fHz,1)/dt));
[c,lags]=xcorr(detrend(a),detrend(m),maxLag,'normalized');
[~,j]=max(c);L=-lags(j)*dt;
end
function f=dominant_freq(x,fs)
if std(x)<eps;f=NaN;return;end
x=detrend(x(:));w=.5*(1-cos(2*pi*(0:numel(x)-1)'/(numel(x)-1)));
x=x.*w;n=2^nextpow2(numel(x));P=abs(fft(x,n)).^2;P=P(1:n/2+1);
fAx=(0:n/2)'*fs/n;P(fAx<1)=0;
[~,j]=max(P);f=fAx(j);
end
