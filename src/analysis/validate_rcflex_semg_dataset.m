function R=validate_rcflex_semg_dataset(dataset,C,writeOutputs)
%VALIDATE_RCFLEX_SEMG_DATASET Run the six real-data stages, in order.
% The standalone real-sEMG validation gate. It answers one question -- is this
% signal worth fusing with the IMU? -- and it answers it BEFORE any estimator
% work, using the same preprocessing, features and quality checks the synthetic
% source goes through.
%
% dataset : a struct or .mat path meeting rcflex_semg_dataset_spec, OR a
%           scenario struct from rcflex_control_case (so the synthetic source
%           can be run through the identical gate as a control).
% R       : table of stage / status / value / threshold / note.
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
% 3. Paired-channel cross-talk
% =========================================================================
% The largest untested assumption in the design: the synthetic source generates
% channels independently, so it assumes ZERO cross-talk, and the whole
% agonist/antagonist feature layout rests on reciprocal activation. Forearm
% flexors and extensors are closely packed, so this is where the real signal is
% most likely to disappoint.
xt=zeros(2,1);
for ax=1:2
 a=env(:,2*ax-1);b=env(:,2*ax);
 if std(a)<eps||std(b)<eps;xt(ax)=NaN;else;xt(ax)=corr(a,b);end
end
worst=max(abs(xt));
rows(end+1,:)={'3 paired cross-talk |r|',stat(worst<0.6),worst,0.6,...
 sprintf('roll %.2f, pitch %.2f; high r collapses the reciprocal-activation assumption',xt(1),xt(2))};

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

fprintf('\n=== Real-sEMG validation: %s ===\n',describe(meta));
disp(R(:,{'stage','status','value','threshold'}));
for k=1:height(R)
 fprintf('  %-32s %s\n',R.stage{k},R.note{k});
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
