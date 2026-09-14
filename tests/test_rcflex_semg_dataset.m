function test_rcflex_semg_dataset()
%TEST_RCFLEX_SEMG_DATASET Exercise the recorded-dataset path without real data.
% The real-sEMG stage will arrive with a dataset attached and a limited amount
% of patience for debugging the loader. So the whole file path -- spec
% validation, unit checks, resampling, and the six validation stages -- is
% proved here against the synthetic stream, which is the one stream whose
% correct answers are already known.
%
% The key property under test: a synthetic scenario exported to the dataset
% format and read back through C.semg_source='file' must produce the SAME
% features as the synthetic source produced directly. If that holds, then any
% difference later seen on a real recording is a property of the recording and
% not of the plumbing.
C=rcflex_control_parameters;C.semg_enabled=true;C=rcflex_prepare_control(C);
D=rcflex_control_case('combined',4,8,1,9102,C);

% --- Round trip: synthetic -> dataset struct -> file source -> features -----
S=struct('semg',D.semg,'fs',C.semg_hz,'channels',{{'FCR','ECRL','FCU','ECU'}},...
 'subject','synthetic','notes','round-trip fixture');
Cf=C;Cf.semg_source='file';
[raw,tRaw,meta]=rcflex_semg_source(struct('semg_dataset',S),Cf);
assert(isequal(raw,D.semg),'File source altered the samples at matching rate');
assert(abs(tRaw(2)-tRaw(1)-1/C.semg_hz)<1e-12,'File source returned the wrong grid');
assert(~meta.ground_truth,'Recorded data must not claim ground truth');

Fdirect=rcflex_semg_features(D.semg,C,0);
Ffile=rcflex_semg_features(raw,C,0);
assert(isequal(Fdirect,Ffile),'Features differ between the direct and file paths');

% --- Round trip through an actual .mat on disk ------------------------------
tmp=[tempname '.mat'];cleanup=onCleanup(@()delete_if(tmp));
save(tmp,'-struct','S');
[raw2,~,meta2]=rcflex_semg_source(struct('semg_dataset',tmp),Cf);
assert(isequal(raw2,D.semg),'.mat round trip altered the samples');
assert(strcmp(meta2.source,'file'),'meta.source wrong for a .mat dataset');

% --- Resampling: a dataset at a different rate lands on our grid ------------
% 2 kHz is a realistic acquisition rate that still satisfies Nyquist for the
% 450 Hz band, so it must be accepted and regridded rather than rejected.
fs2=2000;n2=round(size(D.semg,1)*fs2/C.semg_hz);
S2=S;S2.fs=fs2;S2.semg=resample(D.semg,1,C.semg_hz/fs2);S2.semg=S2.semg(1:n2,:);
[raw3,tRaw3]=rcflex_semg_source(struct('semg_dataset',S2),Cf);
assert(abs(1/(tRaw3(2)-tRaw3(1))-C.semg_hz)<1e-6,'Resampled dataset is not on C.semg_hz');
assert(abs(size(raw3,1)/C.semg_hz-n2/fs2)<.01,'Resampled dataset changed duration');

% --- The spec rejects the mistakes a real dataset will actually make --------
mustFail(@()rcflex_semg_source(struct('semg_dataset',rmfield(S,'fs')),Cf),...
 'missing fs');
mustFail(@()rcflex_semg_source(struct('semg_dataset',setfield(S,'semg',S.semg')),Cf),...
 'transposed semg');
mustFail(@()rcflex_semg_source(struct('semg_dataset',setfield(S,'semg',S.semg*1000)),Cf),...
 'gain-included / ADC-count units');
mustFail(@()rcflex_semg_source(struct('semg_dataset',setfield(S,'semg',S.semg(:,1:2))),Cf),...
 'wrong channel count');
S3=S;S3.fs=500;
mustFail(@()rcflex_semg_source(struct('semg_dataset',S3),Cf),...
 'sample rate below the band requirement');
S4=S;S4.labels=struct('t_start',0,'t_end',1,'class','wiggling');
mustFail(@()rcflex_semg_source(struct('semg_dataset',S4),Cf),'invalid label class');

% --- The spec is machine-readable, so a recording script can target it ------
spec=rcflex_semg_dataset_spec();
assert(all(ismember({'semg','fs'},spec.required)),'Spec lost a required field');
assert(numel(spec.channel_order)==C.semg_channels,'Spec channel order disagrees with C');

% --- The six-stage gate runs, and reports SKIPPED rather than PASS ----------
% A dataset with no labels and no IMU can only answer stages 1 and 3. The
% remaining four must report SKIPPED: treating an unanswerable stage as passed
% is the failure mode this whole gate exists to prevent.
R=validate_rcflex_semg_dataset(S,C,false);
assert(height(R)==6,'Expected six validation stages, got %d',height(R));
assert(all(ismember(R.status,{'PASS','FAIL','SKIPPED'})),'Unexpected status value');
skipped=R.status(strcmp(R.status,'SKIPPED'));
assert(numel(skipped)==4,'Expected 4 skipped stages without labels/IMU, got %d',numel(skipped));
assert(strcmp(R.status{1},'PASS'),'Acquisition health should pass on the synthetic stream');

% --- A single scenario cannot answer the contrast stages --------------------
% Stages 2 and 6 compare tremor-bearing against tremor-free segments, and one
% 4 s scenario contains only one condition. The gate must say SKIPPED here
% rather than manufacture a comparison from a single condition.
Rone=validate_rcflex_semg_dataset(D,C,false);
assert(strcmp(Rone.status{2},'SKIPPED')&&strcmp(Rone.status{6},'SKIPPED'),...
 'A single-condition scenario must skip the contrast stages, not score them');
assert(strcmp(Rone.status{4},'PASS'),'Synthetic stream must show a positive sEMG lead');
assert(strcmp(Rone.status{5},'PASS'),'Synthetic burst frequency must agree with the mechanical tremor');

% --- A multi-trial protocol answers all six ---------------------------------
% This fixture is also the recording protocol template: cued rest/tremor-free,
% tremor, and combined trials in one session with logged boundaries. Labelling
% by cue at record time is far more reliable than labelling after the fact.
P=protocol_fixture(C);
Rfull=validate_rcflex_semg_dataset(P,C,false);
assert(~any(strcmp(Rfull.status,'SKIPPED')),...
 'A multi-trial protocol should answer all six stages; %d skipped',sum(strcmp(Rfull.status,'SKIPPED')));
assert(strcmp(Rfull.status{2},'PASS'),'Synthetic stream must clear the 6 dB SNR gate');
assert(strcmp(Rfull.status{6},'PASS'),'Synthetic stream must discriminate intent from tremor');

fprintf('sEMG dataset checks: round trip (struct and .mat), resampling, six spec rejections, and the six-stage gate (skip accounting and full run) passed.\n');
end
function P=protocol_fixture(C)
%PROTOCOL_FIXTURE Concatenate cued trials into one labelled dataset.
% The template a real recording session should follow: distinct cued conditions
% in one continuous capture, with trial boundaries logged as they happen.
trials={'intent','intent';'sine','tremor';'combined','both'};
semg=[];imu=[];L=struct('t_start',{},'t_end',{},'class',{});
tOff=0;imuFs=[];
for k=1:size(trials,1)
 D=rcflex_control_case(trials{k,1},4,8,1,9200+k,C);
 if isempty(imuFs);imuFs=1/(D.time(2)-D.time(1));end
 semg=[semg;D.semg]; %#ok<AGROW>
 acc=[gradient(gradient(D.tremor(:,1),D.time),D.time),...
      gradient(gradient(D.tremor(:,2),D.time),D.time)];
 imu=[imu;acc]; %#ok<AGROW>
 % Skip the first 1.5 s of each trial: every scenario opens with an amplitude
 % ramp, and concatenation adds a filter transient at the join.
 L(end+1)=struct('t_start',tOff+1.5,'t_end',tOff+D.time(end),'class',trials{k,2}); %#ok<AGROW>
 tOff=tOff+D.time(end)+1/C.semg_hz;
end
P=struct('semg',semg,'fs',C.semg_hz,'imu_accel',imu,'imu_fs',imuFs,'labels',L,...
 'channels',{{'FCR','ECRL','FCU','ECU'}},'subject','synthetic protocol');
end
function mustFail(fn,what)
ok=false;
try fn();catch;ok=true;end
assert(ok,'Dataset spec accepted %s; it must be rejected',what);
end
function delete_if(f)
if isfile(f);delete(f);end
end
