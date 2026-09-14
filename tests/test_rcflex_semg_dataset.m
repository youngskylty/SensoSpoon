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
% A dataset with no labels and no IMU can answer only stage 1. Stage 3 now
% scores isolated contraction segments individually, so with no labels it has
% no segment to score and must SKIP rather than fall back to a pooled number.
% The remaining five must report SKIPPED: treating an unanswerable stage as
% passed is the failure mode this whole gate exists to prevent.
R=validate_rcflex_semg_dataset(S,C,false);
assert(height(R)==6,'Expected six validation stages, got %d',height(R));
assert(all(ismember(R.status,{'PASS','FAIL','SKIPPED'})),'Unexpected status value');
skipped=R.status(strcmp(R.status,'SKIPPED'));
assert(numel(skipped)==5,'Expected 5 skipped stages without labels/IMU, got %d',numel(skipped));
assert(strcmp(R.status{1},'PASS'),'Acquisition health should pass on the synthetic stream');
assert(strcmp(R.status{3},'SKIPPED'),'Stage 3 must SKIP without labelled isolated segments');

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

% =========================================================================
% Stage 3 (raw cross-talk, GATE) vs stage 3b (co-activation, DIAGNOSTIC)
% =========================================================================
% These tests exist because stage 3 originally correlated ENVELOPES over the
% whole POOLED record, and both of those choices were wrong in a way that
% produced a confident PASS on data that had never been tested. Each assertion
% below pins one half of that failure so it cannot return.

% --- 1. Correlated envelopes, independent raw: the core confusion -----------
% The fixture shares one activation drive across all four channels (so the
% envelopes correlate strongly, exactly as genuine co-contraction does) while
% the raw carriers stay independent (so there is NO electrical cross-talk).
% Envelope correlation must be high and raw cross-talk must be low. Anything
% that reports this as cross-talk is measuring shared neural drive instead.
Sclean=xtalk_fixture(C,[0 0 0],{'intent','intent','intent'},4101);
[Rc,Dc]=validate_rcflex_semg_dataset(Sclean,C,false);
assert(strcmp(Rc.status{3},'PASS'),'Zero-cross-talk fixture must pass stage 3');
assert(Rc.value(3)<0.30,'Independent raw carriers must give low raw |r|, got %.3f',Rc.value(3));
assert(Dc.coactivation.abs_median>0.5,...
 'Shared drive must show high co-activation; got %.2f',Dc.coactivation.abs_median);
assert(Dc.coactivation.abs_median>Rc.value(3)*2,...
 'Envelope correlation must exceed raw cross-talk on a co-activated, independent fixture');

% --- 2. Pooling conditions produces a misleading correlation ----------------
% Two segments whose agonist/antagonist envelopes move oppositely. Each segment
% on its own is strongly correlated; pooled into one correlation they cancel.
% This is the artefact that made the old stage 3 report |r| ~ 0.1 on channels
% that were in fact strongly related within every condition.
Sp=pooling_fixture(C,4102);
[Fp,tp]=rcflex_semg_features(Sp.semg,C,0);Ep=Fp(:,1:4);
mA=tp>=0.5&tp<=5;mB=tp>=5.5&tp<=10;mAll=(tp>=0.5&tp<=5)|(tp>=5.5&tp<=10);
rA=corr(Ep(mA,1),Ep(mA,2));rB=corr(Ep(mB,1),Ep(mB,2));
rPooled=corr(Ep(mAll,1),Ep(mAll,2));
assert(abs(rA)>0.5&&abs(rB)>0.5,...
 'Fixture should be strongly correlated within each condition (%.2f, %.2f)',rA,rB);
assert(sign(rA)~=sign(rB),'Fixture conditions should carry opposite-sign relationships');
assert(abs(rPooled)<min(abs(rA),abs(rB)),...
 'Pooling must demonstrably understate the within-condition relationship (pooled %.2f vs %.2f/%.2f)',...
 rPooled,rA,rB);

% --- 3 & 4. Per-segment scoring, gated on the WORST segment -----------------
% One badly coupled segment among otherwise clean ones. A pooled or averaged
% statistic would dilute it; the gate must take the worst case, because
% cross-talk is a property of electrode placement and one bad pair breaks the
% agonist/antagonist layout.
Sbad=xtalk_fixture(C,[0 0 1.2],{'intent','intent','intent'},4103);
[Rb,Db]=validate_rcflex_semg_dataset(Sbad,C,false);
assert(height(Db.stage3_segments)==6,...
 'Expected 3 segments x 2 axes scored separately, got %d',height(Db.stage3_segments));
assert(numel(unique(Db.stage3_segments.segment))==3,'Segments must be scored individually');
assert(strcmp(Rb.status{3},'FAIL'),'A strongly coupled segment must fail the gate');
assert(abs(Rb.value(3)-Db.stage3_worst)<1e-12,'Stage 3 must report the worst-case segment');
assert(Db.stage3_worst>Db.stage3_median,...
 'Worst case must exceed the median when only one segment is coupled');
% The clean segments must still read clean -- the failure is localised, not smeared.
clean=Db.stage3_segments.abs_r(Db.stage3_segments.segment<3);
assert(all(clean<0.30),'Uncoupled segments must not inherit the bad segment''s cross-talk');

% --- 5. Stage 3b never affects PASS/FAIL -----------------------------------
% Same raw cross-talk, wildly different co-activation. The gate must not move.
assert(Dc.coactivation.abs_median>0.5&&strcmp(Rc.status{3},'PASS'),...
 'High co-activation with clean raw channels must still PASS stage 3');
Sflat=xtalk_fixture(C,[0 0 0],{'intent','intent','intent'},4104);
[Rf,Df]=validate_rcflex_semg_dataset(Sflat,C,false);
assert(strcmp(Rf.status{3},'PASS'),'Co-activation level must not drive the stage 3 verdict');
assert(isfinite(Df.coactivation.abs_median),'Stage 3b must still be reported');
assert(~any(strcmp(R.Properties.VariableNames,'coactivation_corr')),...
 'Stage 3b must not appear as a gate column');
assert(height(Rf)==6,'Diagnostics must not add rows to the gate table');

% --- 6. Insufficient labels SKIP, they never PASS ---------------------------
Srest=xtalk_fixture(C,[0 0],{'rest','rest'},4105);
Rr=validate_rcflex_semg_dataset(Srest,C,false);
assert(strcmp(Rr.status{3},'SKIPPED'),'Rest-only labels cannot score cross-talk');
Stre=xtalk_fixture(C,[0 0],{'tremor','both'},4106);
Rt=validate_rcflex_semg_dataset(Stre,C,false);
assert(strcmp(Rt.status{3},'SKIPPED'),'Non-isolated conditions cannot score cross-talk');
% A labelled segment too short to survive the 1.5 s ramp exclusion.
Sshort=xtalk_fixture(C,0,{'intent'},4107);
Sshort.labels=struct('t_start',3.4,'t_end',5,'class','intent');
Rs=validate_rcflex_semg_dataset(Sshort,C,false);
assert(strcmp(Rs.status{3},'SKIPPED'),'An over-short segment must SKIP, not score');

fprintf(['sEMG dataset checks: round trip (struct and .mat), resampling, six spec rejections, ' ...
 'the six-stage gate (skip accounting and full run), and stage 3 raw cross-talk vs stage 3b ' ...
 'co-activation (envelope/raw separation, pooling artefact, per-segment worst case, ' ...
 'diagnostic isolation, skip accounting) passed.\n']);
end
function S=xtalk_fixture(C,alphas,classes,seed)
%XTALK_FIXTURE Raw sEMG with KNOWN cross-talk and KNOWN co-activation.
% Every trial carries four band-limited carriers that are mutually INDEPENDENT
% -- no electrical cross-talk -- amplitude-modulated by one SHARED activation
% envelope, which is what genuine co-contraction looks like and which makes the
% envelopes correlate strongly. Electrical cross-talk is then added explicitly
% and instantaneously as alpha x the partner channel, which is how volume
% conduction actually couples two electrodes. alpha = 0 therefore means "high
% co-activation, zero cross-talk": the case the old envelope-based stage 3
% could not distinguish from genuine coupling.
rng(seed);
fs=C.semg_hz;dur=5;n=round(dur*fs);
[b,a]=butter(4,[C.semg_band_lo_Hz C.semg_band_hi_Hz]/(fs/2),'bandpass');
semg=[];L=struct('t_start',{},'t_end',{},'class',{});tOff=0;
for k=1:numel(alphas)
 t=(0:n-1)'/fs;
 % One slowly varying drive shared by all four channels. It must keep VARYING
 % inside the scored window: a drive that ramps up and then sits flat leaves the
 % envelopes constant, and their correlation would then be measuring noise
 % rather than the shared activation this fixture exists to represent.
 act=0.3+0.7*(0.5+0.5*sin(2*pi*0.5*t+0.3));        % one drive, all channels
 w=filtfilt(b,a,randn(n,4));w=w./std(w);
 x=1e-4*(w.*act);
 x(:,2)=x(:,2)+alphas(k)*x(:,1);                   % roll  pair coupling
 x(:,4)=x(:,4)+alphas(k)*x(:,3);                   % pitch pair coupling
 semg=[semg;x]; %#ok<AGROW>
 L(end+1)=struct('t_start',tOff+0.5,'t_end',tOff+dur,'class',classes{k}); %#ok<AGROW>
 tOff=tOff+dur;
end
S=struct('semg',semg,'fs',fs,'labels',L,'channels',{{'FCR','ECRL','FCU','ECU'}},...
 'subject','cross-talk fixture');
end
function S=pooling_fixture(C,seed)
%POOLING_FIXTURE Two conditions whose agonist/antagonist relationship inverts.
% Segment A drives both channels together; segment B drives one up as the other
% comes down. Each is strongly correlated on its own; pooled they cancel. This
% is the Simpson's-paradox artefact that made a pooled correlation look clean.
rng(seed);
fs=C.semg_hz;dur=5;n=round(dur*fs);t=(0:n-1)'/fs;
[b,a]=butter(4,[C.semg_band_lo_Hz C.semg_band_hi_Hz]/(fs/2),'bandpass');
ramp=max(0,min(1,(t-0.5)/(dur-1.5)));
mk=@(e1,e2)local_mod(filtfilt(b,a,randn(n,4)),e1,e2);
A=mk(0.2+0.8*ramp,0.2+0.8*ramp);        % both rise together      -> r > 0
B=mk(0.2+0.8*ramp,1.0-0.8*ramp);        % one rises, one falls    -> r < 0
S=struct('semg',[A;B],'fs',fs,'channels',{{'FCR','ECRL','FCU','ECU'}},...
 'labels',struct('t_start',{0.5,dur+0.5},'t_end',{dur,2*dur},'class',{'intent','intent'}),...
 'subject','pooling fixture');
end
function x=local_mod(w,e1,e2)
w=w./std(w);
x=1e-4*[w(:,1).*e1,w(:,2).*e2,w(:,3).*e1,w(:,4).*e2];
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
