function [T,TH,CL,DG]=run_fast_movement_experiment(writeOutputs)
%RUN_FAST_MOVEMENT_EXPERIMENT E3: can a rate detector fix the fast-reach gap?
%
% THE PROBLEM. The shipped sEMG intent gate detects voluntary movement from the
% AMPLITUDE of muscle drive. Rejecting tremor-driven activation needs heavy
% smoothing, smoothing costs group delay, and the result is that the gate opens
% 404 ms after a fast voluntary movement begins, against a 350 ms target
% declared in advance. On the 'reach' scenario the shipped system reads ~0.67
% deg attitude RMS against 0.285 deg for the IMU-only estimator: the gate
% degrades the very voluntary movement it exists to protect. (V3 Sec 7.)
%
% THE CANDIDATE. Muscle drive RISES before it is LARGE, so its time derivative
% should cross a threshold earlier than its amplitude does. That is a
% hypothesis, not a result, and it has an obvious cost: differentiation
% amplifies exactly the noise the smoothing removed.
%
% THE THREE ARMS (C.semg_gate_detector -- one switch, one file):
%   A 'amplitude'  the shipped two-speed gate. REFERENCE.
%   B 'rate'       the fast amplitude path replaced by the derivative of the
%                  same filtered drive.
%   C 'hybrid'     both early paths, combined with max().
% The SLOW amplitude path is common to all three and is the only path that may
% ever CLOSE the gate, so the arms differ in early detection and nothing else.
%
% HOW THRESHOLDS ARE SET. Every rate floor here is derived by the SAME
% geometric-mean rule that produced the two amplitude floors
% (calibrate_rcflex_semg_intent.m): the geometric mean of the worst tremor-only
% and the best intent-bearing drive over the 20 benchmark cases. The 'reach'
% scenario is NOT in that set, so no threshold is fitted to the case it is
% judged on. The rate corner is SWEPT and every point is reported, pass or fail.
%
% THE CRITERIA WERE DECLARED BEFORE THE RUN and are the project's existing ones:
%   latency     < 350 ms from movement onset to gate open       (V3 Sec 7)
%   separation  conf <= 0.10 on tremor-only AND conf >= 0.50 on intent-bearing
%               (tests/test_rcflex_semg_intent_gate.m)
% A candidate must meet BOTH. If none does, this script says so and the shipped
% amplitude gate is retained -- the target is not moved to what was achieved.
if nargin<1;writeOutputs=true;end

LATENCY_TARGET_S=.350;
SEP_CLOSED_MAX=.10;
SEP_OPEN_MIN=.50;
RATE_CORNERS_HZ=[2 4 8 16];   % declared sweep; 4 Hz is the fast path's own corner
SEEDS=[4243 9401 9402 9403];  % held out from the benchmark's 9100+k block

fprintf('\n########## E3: fast voluntary movement, detector comparison ##########\n');

% =====================================================================
% 1. Derive every rate floor, by the same rule, from the same 20 cases
% =====================================================================
TH=derive_rate_floors(RATE_CORNERS_HZ);
fprintf('\nRate-path floors, geometric-mean rule over the 20 benchmark cases\n');
fprintf('(the reach scenario is held out of this derivation):\n');
disp(TH);

% SEPARABILITY DIAGNOSTIC, before any arm is built.
% The 20 benchmark cases contain only SLOW voluntary motion (0.2-0.25 Hz), and
% the rate observable is by construction largest for the FASTEST thing present
% -- which in those cases is the tremor. So the derived separation can come out
% INVERTED, and an inverted separation is not a tuning problem: no threshold
% fixes an observable that ranks the two classes the wrong way round.
%
% The question that then matters is narrower and decisive: on the fast movement
% the detector exists for, does the rate rise ABOVE the worst tremor-only rate?
% If it does not, no threshold can separate them and the candidate is dead
% however it is tuned. If it does, a separating threshold exists -- but it could
% only be derived from a case set containing fast voluntary movement, which the
% project's 20-case methodology does not provide, and that is a gap to state
% rather than to fill quietly by fitting to the case under test.
DG=reach_separability(RATE_CORNERS_HZ,SEEDS,TH);
fprintf('\nSeparability of the RATE observable on the held-out fast reach:\n');
disp(DG);

% =====================================================================
% 2. Arms
% =====================================================================
arms=struct('tag',{'A amplitude'},'detector',{'amplitude'},'rate_fc',{NaN},'rate_floor',{NaN});
for k=1:numel(RATE_CORNERS_HZ)
 fc=RATE_CORNERS_HZ(k);fl=TH.rate_floor_deg_s(TH.rate_fc_Hz==fc);
 arms(end+1)=struct('tag',sprintf('B rate %g Hz',fc),'detector','rate',...
  'rate_fc',fc,'rate_floor',fl); %#ok<AGROW>
end
for k=1:numel(RATE_CORNERS_HZ)
 fc=RATE_CORNERS_HZ(k);fl=TH.rate_floor_deg_s(TH.rate_fc_Hz==fc);
 arms(end+1)=struct('tag',sprintf('C hybrid %g Hz',fc),'detector','hybrid',...
  'rate_fc',fc,'rate_floor',fl); %#ok<AGROW>
end

% =====================================================================
% 3. Open-loop gate behaviour: latency, false triggering, separation
% =====================================================================
% Open loop first, deliberately: latency, false triggering and separation are
% properties of the GATE, so measuring them with no plant makes a failure
% attributable to the detector rather than to the estimator or the actuator --
% the same reason tests/test_rcflex_semg_intent_gate.m exists.
%
% The sEMG stream depends only on the scenario and the seed, never on the
% detector, so every scenario is generated and preprocessed ONCE and all twelve
% arms are run over the identical cached features. Besides being ~12x faster,
% this removes stream generation as a source of difference between arms.
tremorOnly={'sine',8,1;'sine',12,1;'sine',4,1;'mixed',8,2;'irregular',8,2};
intentBearing={'intent',8,0;'combined',8,1};
Cgen=arm_params(arms(1));           % any arm: the sEMG source settings are identical
cache=build_cache(Cgen,SEEDS,tremorOnly,intentBearing);

fprintf('\n--- open-loop gate behaviour ---\n');
rows={};
for a=1:numel(arms)
 C=arm_params(arms(a));
 lat=zeros(numel(SEEDS),1);
 for si=1:numel(SEEDS)
  c=run_gate(cache.reach{si},C);
  lat(si)=reach_latency(cache.t,c);
 end
 falseOpen=0;loConf=0;
 for i=1:numel(cache.tremor)
  c=run_gate(cache.tremor{i},C);w=cache.t>1.5;
  falseOpen=max(falseOpen,mean(max(c(w,:),[],2)>SEP_OPEN_MIN));
  loConf=max(loConf,max(mean(c(w,:),1)));
 end
 hiConf=1;
 for i=1:numel(cache.intent)
  c=run_gate(cache.intent{i},C);
  ti=(1:size(c,1))'/C.outer_hz;
  hiConf=min(hiConf,min(mean(c(ti>1.5,:),1)));
 end
 sepOk=loConf<=SEP_CLOSED_MAX&&hiConf>=SEP_OPEN_MIN;
 latOk=mean(lat)<LATENCY_TARGET_S;
 rows(end+1,:)={string(arms(a).tag),string(arms(a).detector),arms(a).rate_fc,arms(a).rate_floor,...
  mean(lat),min(lat),max(lat),std(lat),loConf,hiConf,hiConf/max(loConf,eps),...
  100*falseOpen,double(sepOk),double(latOk),double(sepOk&&latOk)}; %#ok<AGROW>
 fprintf(['%-14s latency %6.0f ms (%3.0f-%3.0f, sd %3.0f) | tremor conf %.3f | intent conf %.3f | ' ...
  'open on tremor %5.1f%% | sep %s | lat %s\n'],...
  arms(a).tag,1000*mean(lat),1000*min(lat),1000*max(lat),1000*std(lat),...
  loConf,hiConf,100*falseOpen,pf(sepOk),pf(latOk));
end
T=cell2table(rows,'VariableNames',{'arm','detector','rate_fc_Hz','rate_floor_deg_s',...
 'latency_mean_s','latency_min_s','latency_max_s','latency_sd_s',...
 'tremor_conf_max','intent_conf_min','separation_ratio','tremor_open_percent',...
 'separation_ok','latency_ok','meets_both'});

% =====================================================================
% 4. Closed loop, for a declared shortlist
% =====================================================================
% SELECTION RULE, fixed in advance so it cannot become cherry-picking: the
% reference arm A, plus the LOWEST-LATENCY arm of each candidate family that
% still satisfies the separation criterion. An arm that has already failed
% separation is not a candidate at all, and running it would only invite its
% movement error being quoted as if it were on offer.
short=[1 pick(T,"rate") pick(T,"hybrid")];
short=short(short>0);
fprintf('\n--- closed loop (IMU-only, reference arm A, and the shortlist) ---\n');
fprintf('shortlist: %s\n',strjoin(cellstr(T.arm(short)),', '));
cl=closed_loop_rows("IMU-only (no gate)",imu_only_params(),SEEDS,tremorOnly);
for a=short
 cl=[cl;closed_loop_rows(T.arm(a),arm_params(arms(a)),SEEDS,tremorOnly)]; %#ok<AGROW>
end
CL=cell2table(cl,'VariableNames',{'arm','scenario','seed','attitude_rms_deg',...
 'suppression_percent','tremor_leak_deg','intent_track_deg'});

% =====================================================================
% 5. Verdict
% =====================================================================
verdict(T,CL,LATENCY_TARGET_S);

if writeOutputs
 writetable(DG,project_path('results','fast_movement_separability.csv'));
 writetable(TH,project_path('results','fast_movement_thresholds.csv'));
 writetable(T,project_path('results','fast_movement_detectors.csv'));
 writetable(CL,project_path('results','fast_movement_closed_loop.csv'));
 fprintf('\nWrote %s\n',project_path('results','fast_movement_thresholds.csv'));
 fprintf('Wrote %s\n',project_path('results','fast_movement_detectors.csv'));
 fprintf('Wrote %s\n',project_path('results','fast_movement_closed_loop.csv'));
 plot_fast_movement_experiment(T,CL);
end
end

% =========================================================================
function ix=pick(T,family)
%PICK Lowest-latency arm of one detector family that keeps the separation.
% Returns 0 when that family has no admissible arm, which is itself a result.
m=T.detector==family&T.separation_ok==1;
if ~any(m);ix=0;return;end
cand=find(m);
[~,j]=min(T.latency_mean_s(cand));
ix=cand(j);
end

function DG=reach_separability(corners,seeds,TH)
%REACH_SEPARABILITY Can ANY rate threshold separate a fast reach from tremor?
% Compares the rate drive during the voluntary phase of 'reach' against the
% worst tremor-only rate already measured at the same corner. Reported as a
% ratio: above 1 means a separating threshold exists, at or below 1 means none
% does and the candidate cannot be rescued by choosing a different number.
%
% This diagnostic deliberately chooses NO threshold. Picking one from the reach
% would be fitting the detector to the case it is judged on, which is the thing
% this experiment is written to avoid.
C=rcflex_control_parameters('ekf_predictive');
C.semg_enabled=true;C=rcflex_prepare_control(C);
fs=C.outer_hz;gain=C.semg.gate_cal(:)';
rows={};
for ci=1:numel(corners)
 c=lowpass2(corners(ci),fs);pk=zeros(numel(seeds),1);
 for si=1:numel(seeds)
  D=rcflex_control_case('reach',4,8,1,seeds(si),C);
  u=rcflex_semg_gate_drive(D.semg,C,c,0);
  r=[zeros(1,2);diff(u)]*fs;
  t=(1:size(r,1))'/fs;
  % The voluntary phase only: the reach runs from t = 2 s to t = 2.8 s.
  w=t>=2&t<=2.8;
  pk(si)=max(max(abs(rad2deg(r(w,:).*gain))));
 end
 lo=TH.tremor_only_max_deg_s(TH.rate_fc_Hz==corners(ci));
 rows(end+1,:)={corners(ci),lo,mean(pk),min(pk),mean(pk)/lo,double(min(pk)>lo)}; %#ok<AGROW>
end
DG=cell2table(rows,'VariableNames',{'rate_fc_Hz','tremor_only_max_deg_s',...
 'reach_peak_mean_deg_s','reach_peak_worst_deg_s','ratio','separable_all_seeds'});
end

function TH=derive_rate_floors(corners)
%DERIVE_RATE_FLOORS The geometric-mean rule, applied at each candidate corner.
% Every corner is derived from the SAME 20 scenarios in the SAME pass, so a
% difference between corners is not confounded by which cases each one saw.
C=rcflex_control_parameters('ekf_predictive');
C.semg_enabled=true;C=rcflex_prepare_control(C);
fs=C.outer_hz;gain=C.semg.gate_cal(:)';
names={'sine','sine','sine','sine','sine','mixed','chirp','irregular','combined','intent','sine','sine','sine','step','sine','sine','sine','sine','sine','sine'};
freq=[4 8 12 6 10 8 8 8 8 8 8 8 8 8 8 5 7 9 11 20];
amp=[1 1 1 3 3 2 2 2 1 0 1 1 1 5 1 1 1 1 1 1];
lp=arrayfun(@(f)lowpass2(f,fs),corners,'UniformOutput',false);
R=nan(20,numel(corners),2);hasIntent=false(20,1);
fprintf('deriving rate floors over 20 cases ');
for k=1:20
 dur=4;if k==10;dur=8;end
 D=rcflex_control_case(names{k},dur,freq(k),amp(k),9100+k,C);
 hasIntent(k)=any(rms(D.intent(D.time>1.5,1:2))>deg2rad(.1));
 for ci=1:numel(corners)
  u=rcflex_semg_gate_drive(D.semg,C,lp{ci},0);
  r=[zeros(1,2);diff(u)]*fs;
  t=(1:size(r,1))'/fs;w=t>1.5;
  d=rad2deg(r(w,:).*gain);
  R(k,ci,1)=rms(d(:,1));R(k,ci,2)=rms(d(:,2));
 end
 fprintf('.');
end
fprintf('\n');
rows={};
for ci=1:numel(corners)
 lo=max(max(R(~hasIntent,ci,:),[],3));hi=min(min(R(hasIntent,ci,:),[],3));
 rows(end+1,:)={corners(ci),lo,hi,hi/max(lo,eps),sqrt(lo*hi)}; %#ok<AGROW>
end
TH=cell2table(rows,'VariableNames',{'rate_fc_Hz','tremor_only_max_deg_s',...
 'intent_bearing_min_deg_s','separation_ratio','rate_floor_deg_s'});
end

function cache=build_cache(C,seeds,tremorOnly,intentBearing)
%BUILD_CACHE Preprocessed sEMG features per scenario, shared by every arm.
cache=struct('reach',{cell(1,numel(seeds))},'tremor',{{}},'intent',{{}},'t',[]);
fprintf('caching scenarios ');
for si=1:numel(seeds)
 cache.reach{si}=features_of('reach',8,1,seeds(si),C);fprintf('.');
end
for i=1:size(tremorOnly,1)
 cache.tremor{end+1}=features_of(tremorOnly{i,1},tremorOnly{i,2},tremorOnly{i,3},seeds(1),C);
 fprintf('.');
end
for i=1:size(intentBearing,1)
 dur=4;if strcmp(intentBearing{i,1},'intent');dur=8;end
 cache.intent{end+1}=features_of(intentBearing{i,1},intentBearing{i,2},intentBearing{i,3},seeds(1),C,dur);
 fprintf('.');
end
fprintf('\n');
cache.t=(1:size(cache.reach{1},1))'/C.outer_hz;
end

function F=features_of(name,fHz,amp,seed,C,dur)
if nargin<6;dur=4;end
D=rcflex_control_case(name,dur,fHz,amp,seed,C);
F=rcflex_semg_features(D.semg,C,0);
end

function c=run_gate(F,C)
n=size(F,1);c=zeros(n,2);s=[];
for k=1:n;[s,c(k,:)]=rcflex_semg_intent_gate(s,F(k,1:C.semg.nch),C);end
end

function C=arm_params(arm)
C=rcflex_control_parameters('ekf_predictive');
C.ekf_intent_model='cv';C.semg_fusion='intent_gate';C.semg_enabled=true;
C.semg_gate_detector=arm.detector;
if ~strcmp(arm.detector,'amplitude')
 C.semg_gate_rate_fc_Hz=arm.rate_fc;
 C.semg_gate_rate_floor_deg_s=arm.rate_floor;
end
C=rcflex_prepare_control(C);
end

function C=imu_only_params()
C=rcflex_control_parameters('ekf_predictive');
C.ekf_intent_model='cv';C.semg_fusion='off';C.semg_enabled=false;
C=rcflex_prepare_control(C);
end

function lag=reach_latency(t,c)
%REACH_LATENCY Time from movement onset to the gate opening, roll axis.
% 'reach' starts its minimum-jerk profile at t = 2 s, and "open" is confidence
% above 0.5 -- exactly the definition tests/test_rcflex_semg_intent_gate.m uses,
% so this number is directly comparable with the 404 ms already on record.
ix=find(c(:,1)>.5&t>2,1);
if isempty(ix);lag=Inf;else;lag=t(ix)-2;end
end

function rows=closed_loop_rows(tag,C,seeds,tremorOnly)
%CLOSED_LOOP_ROWS Movement error on 'reach' over every seed, and tremor
% cancellation on two tremor-only cases. Both matter: an arm that opens early
% and never closes would win the first and lose the second.
rows={};H=rcflex_parameters;
for si=1:numel(seeds)
 C.seed=seeds(si);
 D=rcflex_control_case('reach',4,8,1,seeds(si),C);
 S=run_rcflex_control_fast(D,C,H,.0005);
 m=rcflex_control_metrics(S);Q=rcflex_intent_split_metrics(S);
 rows(end+1,:)={string(tag),"reach",seeds(si),m.attitude_rms_deg,...
  m.suppression_percent,Q.tremor_leak_deg,Q.intent_track_deg}; %#ok<AGROW>
 fprintf('  %-16s reach  seed %d : attitude %.4f deg\n',tag,seeds(si),m.attitude_rms_deg);
end
for i=1:2
 C.seed=seeds(1);
 D=rcflex_control_case(tremorOnly{i,1},4,tremorOnly{i,2},tremorOnly{i,3},seeds(1),C);
 S=run_rcflex_control_fast(D,C,H,.0005);
 m=rcflex_control_metrics(S);Q=rcflex_intent_split_metrics(S);
 rows(end+1,:)={string(tag),string(tremorOnly{i,1}),seeds(1),m.attitude_rms_deg,...
  m.suppression_percent,Q.tremor_leak_deg,Q.intent_track_deg}; %#ok<AGROW>
 fprintf('  %-16s %-6s seed %d : suppression %.2f%%\n',tag,tremorOnly{i,1},seeds(1),m.suppression_percent);
end
end

function verdict(T,CL,target)
fprintf('\n================ E3 VERDICT ================\n');
ref=T(T.detector=="amplitude",:);
fprintf('Declared criteria: latency < %.0f ms AND (tremor conf <= 0.10, intent conf >= 0.50).\n',1000*target);
fprintf('Reference arm A (shipped): latency %.0f ms, tremor conf %.3f, intent conf %.3f.\n',...
 1000*ref.latency_mean_s(1),ref.tremor_conf_max(1),ref.intent_conf_min(1));
win=T(T.meets_both==1,:);
if isempty(win)
 fprintf(['\nNO CANDIDATE MEETS BOTH CRITERIA.\n' ...
  'Every arm that reaches the latency target loses the separation the fusion depends\n' ...
  'on, and every arm that keeps the separation misses the latency target. The target is\n' ...
  'NOT moved to whatever was achieved: the shipped amplitude gate is retained, and fast\n' ...
  'voluntary movement remains the delivered system''s largest known weakness.\n']);
 adm=T(T.separation_ok==1,:);
 if ~isempty(adm)
  [~,j]=min(adm.latency_mean_s);
  fprintf('Best latency among arms that DO keep the separation: %s at %.0f ms (target %.0f ms).\n',...
   adm.arm(j),1000*adm.latency_mean_s(j),1000*target);
 end
else
 fprintf('\n%d candidate(s) meet both criteria:\n',height(win));
 disp(win(:,{'arm','latency_mean_s','tremor_conf_max','intent_conf_min','tremor_open_percent'}));
 fprintf(['Closed-loop cost decides adoption, not latency alone: an arm that opens earlier\n' ...
  'also spends longer open, and that is paid for in tremor suppression.\n']);
end
r=CL(CL.scenario=="reach",:);
fprintf('\nClosed-loop ''reach'' attitude error, mean over %d seeds (lower is better):\n',...
 numel(unique(r.seed)));
disp(groupsummary(r,'arm','mean','attitude_rms_deg'));
s=CL(CL.scenario~="reach",:);
fprintf('Tremor suppression on the tremor-only cases (higher is better):\n');
disp(groupsummary(s,'arm','mean','suppression_percent'));
fprintf('============================================\n');
end

function s=pf(ok);if ok;s='PASS';else;s='FAIL';end;end

function c=lowpass2(f,fs)
k=tan(pi*f/fs);den=1+sqrt(2)*k+k*k;
c=[k*k 2*k*k k*k]/den;c=[c 2*(k*k-1)/den (1-sqrt(2)*k+k*k)/den];
end
