function test_rcflex_semg()
%TEST_RCFLEX_SEMG Standalone checks on the sEMG sensing stage.
% Run in isolation -- no plant, no controller, no closed loop -- so a failure
% here is attributable to the sensing chain itself. The sEMG stage is being
% introduced BEFORE any decision about how it should influence the estimator,
% so these are the acceptance criteria that decide whether the signal is worth
% fusing at all, stated up front rather than after the fact.
C=rcflex_control_parameters;C.semg_enabled=true;C=rcflex_prepare_control(C);

% --- Synchronisation is structural, not approximate -------------------------
assert(mod(C.semg_hz,C.outer_hz)==0,'sEMG rate must be an integer multiple of the control rate');
assert(C.semg.sub==C.semg_hz/C.outer_hz,'Sub-sample count inconsistent with the rates');
D=rcflex_control_case('sine',4,8,1,1701,C);
ticks=floor((numel(D.time)-1)/round(1/C.outer_hz/(D.time(2)-D.time(1))))+1;
assert(size(D.semg,1)>=ticks*C.semg.sub-C.semg.sub,...
 'sEMG stream is too short to cover every control tick');
assert(abs(D.semg_time(2)-D.semg_time(1)-1/C.semg_hz)<1e-12,'sEMG grid spacing wrong');
assert(abs(D.semg_time(1)-D.time(1))<1e-12,'sEMG stream must start at the scenario start');

% --- The scenario carries the ground-truth decomposition --------------------
assert(isfield(D,'tremor')&&size(D.tremor,2)>=2,'Scenario must expose tremor truth');
assert(max(abs(D.base(:,1:2)-(D.intent(:,1:2)+D.tremor(:,1:2))),[],'all')<1e-12,...
 'base must equal intent + tremor, or the characterisation ground truth is wrong');

% --- Preprocessing is a pure, deterministic function ------------------------
block=D.semg(1:C.semg.sub,:);
s1=[];s2=[];
for k=1:40;[s1,f1]=rcflex_semg_preprocess(s1,block,C);end
for k=1:40;[s2,f2]=rcflex_semg_preprocess(s2,block,C);end
assert(isequal(s1,s2)&&isequal(f1,f2),'sEMG preprocessing must be deterministic');
assert(numel(f1)==C.semg.n_feat,'Feature width disagrees with C.semg.n_feat');

% --- The highpass actually rejects the electrode offset ---------------------
% The half-cell offset is ~10x the signal, so if it survived preprocessing it
% would swamp every feature. Drive the chain with a pure DC block and require
% the steady-state envelope to settle near zero.
s=[];dcBlock=C.semg_offset_V*ones(C.semg.sub,C.semg.nch);
for k=1:round(2*C.outer_hz);[s,f]=rcflex_semg_preprocess(s,dcBlock,C);end
assert(max(abs(f))<.02*C.semg_offset_V,...
 sprintf('Electrode DC offset leaks through preprocessing (%.3g V of %.3g V)',max(abs(f)),C.semg_offset_V));

% --- Mains interference is rejected ----------------------------------------
% A pure 50 Hz tone at the modelled amplitude must not produce an envelope
% comparable to a real contraction.
tE=(0:1/C.semg_hz:2)';mains=C.semg_mains_V*sin(2*pi*C.semg_mains_Hz*tE);
envMains=run_chain(mains*ones(1,C.semg.nch),C);
tremorless=C.semg_mvc_V*.3*randn(RandStream('mt19937ar','Seed',7),numel(tE),C.semg.nch);
envSignal=run_chain(tremorless,C);
assert(mean(envMains)<.25*mean(envSignal),...
 sprintf('Mains notch insufficient: mains envelope %.3g vs signal envelope %.3g',mean(envMains),mean(envSignal)));

% --- Every channel survives the front end ----------------------------------
% Checked PER CHANNEL, not in aggregate. The electrode offset is drawn
% independently per channel, so a front-end headroom fault (e.g. gain set
% without AC coupling ahead of it) rails only the channels that happen to draw
% a large offset -- and an aggregate or single-channel check passes happily
% while half the array is dead.
Fs=semg_features(rcflex_control_case('sine',4,8,1,9101,C),C);
live=sqrt(mean(Fs(round(end/2):end,:).^2,1));
chanFloor=.05*max(live);
assert(all(live>chanFloor),sprintf(...
 'sEMG channel(s) %s are dead after preprocessing (feature rms %s) -- check front-end headroom',...
 mat2str(find(live<=chanFloor)),mat2str(live,3)));

% --- ACCEPTANCE: sEMG separates intent from tremor -------------------------
% This is the criterion the whole staged plan turns on. If the burst-band
% feature does not discriminate a tremor-bearing scenario from a tremor-free
% one, sEMG has nothing to contribute to the estimator and the next stage
% should not be attempted.
bTremor=burst_power(rcflex_control_case('sine',4,8,1,1701,C),C);
bIntent=burst_power(rcflex_control_case('intent',4,8,1,1702,C),C);
ratio=bTremor/max(bIntent,eps);
assert(ratio>4,sprintf(...
 'sEMG burst feature does not separate tremor from voluntary motion (power ratio %.2f, need >4)',ratio));

% --- ACCEPTANCE: the sEMG burst LEADS the mechanical tremor -----------------
% The electromechanical delay is the only thing sEMG offers that the IMU
% cannot already provide. If the measured lead is not positive and of the
% modelled order, fusing sEMG buys nothing over a better IMU estimator.
lead=measured_lead(rcflex_control_case('sine',4,8,1,1701,C),C);
assert(lead>.005&&lead<.100,sprintf(...
 'sEMG burst lead over the IMU is %.1f ms; expected 5-100 ms (modelled EMD %.0f ms)',...
 1000*lead,1000*C.semg_emd_s));

fprintf('sEMG checks: sync ratio, ground-truth decomposition, determinism, offset and mains rejection, tremor/intent separation (%.1fx) and %.0f ms burst lead passed.\n',ratio,1000*lead);
end
function env=run_chain(raw,C)
% Envelope feature (channel 1) over a raw stream, tick by tick.
nTick=floor(size(raw,1)/C.semg.sub);s=[];env=zeros(nTick,1);
for j=1:nTick
 [s,f]=rcflex_semg_preprocess(s,raw((j-1)*C.semg.sub+(1:C.semg.sub),:),C);
 env(j)=f(1);
end
env=env(round(end/2):end);  % discard the filter transient
end
function p=burst_power(D,C)
% Mean-square burst-band feature over the second half of a scenario, averaged
% across channels: "how much rhythmic bursting is in this sEMG".
F=semg_features(D,C);b=F(round(end/2):end,C.semg.nch+1:end);
p=mean(b(:).^2);
end
function lead=measured_lead(D,C)
% Cross-correlation lag between the sEMG burst feature and the mechanical
% tremor, both at the control rate. A positive lag means sEMG leads.
%
% The comparand is tremor ACCELERATION, not tremor angle. Muscle force drives
% acceleration, so that is the mechanical quantity the activation is in phase
% with; for a sinusoid the angle is 180 degrees away from it, and correlating
% against the angle instead reports the true lead minus half a tremor period
% (at 8 Hz, +40 ms reads as -22.5 ms). Getting this wrong would make a
% correctly leading signal look like a lagging one.
%
% The search window is also capped at half a tremor period. Beyond that a lead
% and a lag of the opposite sign are indistinguishable in a periodic signal,
% so a wider window invites the same confusion in a different form.
F=semg_features(D,C);dt=1/C.outer_hz;
tTick=(0:size(F,1)-1)'*dt+D.time(1);
tremorAcc=gradient(gradient(D.tremor(:,1),D.time),D.time);
mech=interp1(D.time,tremorAcc,tTick,'linear',0);
a=F(:,C.semg.nch+1);  % roll agonist burst feature
ix=round(numel(tTick)/2):numel(tTick);
a=detrend(a(ix));mech=detrend(mech(ix));
maxLag=max(1,floor(.5/D.frequency/dt));
[c,lags]=xcorr(a,mech,maxLag,'normalized');
[~,j]=max(c);lead=-lags(j)*dt;
end
function F=semg_features(D,C)
F=rcflex_semg_features(D.semg,C,D.time(1));
end
