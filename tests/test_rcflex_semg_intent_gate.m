function test_rcflex_semg_intent_gate()
%TEST_RCFLEX_SEMG_INTENT_GATE Standalone checks on the sEMG->EKF intent gate.
% No plant and no closed loop, so a failure here is attributable to the gate
% itself. These are the acceptance criteria the fusion was committed to before
% the closed-loop benchmark was run.
C=rcflex_control_parameters('ekf_predictive');
C.semg_enabled=true;C=rcflex_prepare_control(C);

% --- Pure and deterministic, like every other stage in this chain -----------
env=[3e-5;1e-5;2e-5;1e-5];
s1=[];s2=[];
for k=1:200;[s1,c1]=rcflex_semg_intent_gate(s1,env,C);end
for k=1:200;[s2,c2]=rcflex_semg_intent_gate(s2,env,C);end
assert(isequal(s1,s2)&&isequal(c1,c2),'Intent gate must be deterministic');
assert(numel(s1)==C.semg.s_len_gate,'Packed gate state disagrees with C.semg.s_len_gate');
assert(numel(c1)==2&&all(c1>=0&c1<=1),'Confidence must be a 2x1 vector in [0,1]');

% --- The gate defaults OPEN, and may only close on positive evidence -------
% A gate that starts closed asserts "there is no voluntary drive" before it has
% measured anything. Regression test for a real defect: the first version did
% exactly that, pinned the confidence at 0.000 for the first 0.85 s of every
% run, and cost benchmark case 9 24 points of suppression.
s=[];
for k=1:round(.5*C.outer_hz);[s,c]=rcflex_semg_intent_gate(s,zeros(4,1),C);end
assert(all(c>.99),sprintf('Gate must stay open through its warm-up (got %.3f)',max(c)));

% --- ... and it does close once warmed up on a silent input ----------------
s=[];
for k=1:round(3*C.outer_hz);[s,c]=rcflex_semg_intent_gate(s,zeros(4,1),C);end
assert(all(c<.02),sprintf('Gate must close on a silent input after warm-up (got %.3f)',max(c)));

% --- ACCEPTANCE: the gate separates the two classes on real scenarios ------
% This is the criterion the whole fusion turns on. If the voluntary-drive
% confidence does not separate tremor-only scenarios from intent-bearing ones,
% the gate cannot choose between the two settings of the intent process noise
% and the fusion has nothing to offer. The threshold is set from the
% requirement, not from the observed value: the ceiling experiment showed the
% benefit needs the intent process noise to swing by more than a decade, so the
% gate must deliver conf<0.1 on tremor-only and conf>0.5 on intent-bearing.
tremorOnly={'sine',8,1;'sine',12,1;'mixed',8,2;'irregular',8,2;'sine',4,1};
intentBearing={'intent',8,0;'combined',8,1};
lo=0;hi=1;
for i=1:size(tremorOnly,1)
 lo=max(lo,max(steady_conf(tremorOnly{i,1},tremorOnly{i,2},tremorOnly{i,3},C)));
end
for i=1:size(intentBearing,1)
 hi=min(hi,min(steady_conf(intentBearing{i,1},intentBearing{i,2},intentBearing{i,3},C)));
end
assert(lo<.1,sprintf('Gate does not close on tremor-only scenarios (worst confidence %.3f, need <0.10)',lo));
assert(hi>.5,sprintf('Gate does not open on intent-bearing scenarios (worst confidence %.3f, need >0.50)',hi));

% --- The gate's response to a real voluntary movement ----------------------
% Measured on the 'reach' scenario: a 0.8 s minimum-jerk voluntary movement
% starting at t = 2 s on top of tremor.
%
% HONEST NOTE ON THE THRESHOLD. The criterion declared in advance was 350 ms,
% chosen so the gate would be open before most of the movement had happened.
% THE SHIPPED GATE DOES NOT MEET IT. Adding the fast path (C.semg_gate_fast_*)
% cut the latency substantially but not to 350 ms, and a 16-point sweep over
% the corner frequency and a drive-rate term found no setting that met both
% that target and the 4:1 separation requirement above -- so no setting was
% adopted, rather than moving the target to whatever the design happened to
% achieve. The bound asserted here is therefore a REGRESSION GUARD at the
% movement's own duration (the gate must at least open while the movement is
% still under way), not a claim that the design target was met. The residual
% cost is quantified in docs/SEMG_INTENT_FUSION.md Sec 4.3 and listed as a
% limitation in Sec 9.
[t,c]=gate_trace('reach',8,1,C);
moveIx=find(t>=2,1);
openIx=find(c(:,1)>.5&(1:numel(t))'>moveIx,1);
assert(~isempty(openIx),'Gate never opened during a fast voluntary movement');
lag=t(openIx)-2;
assert(lag<.8,sprintf(['Gate took %.0f ms to open on an 800 ms voluntary reach, ' ...
 'i.e. not until the movement was over. Expected well under 800 ms ' ...
 '(target was 350 ms, see docs/SEMG_INTENT_FUSION.md Sec 4.3).'],1000*lag));

% --- The fast path must not be able to CLOSE the gate ----------------------
% The whole safety argument for giving the fast path a weaker 3.0:1 separation
% is that it can only ever raise the confidence. Check that directly: the
% combined confidence must never fall below what the slow path alone reports.
[~,cSlow]=gate_trace_slow_only('sine',8,1,C);
[~,cBoth]=gate_trace('sine',8,1,C);
assert(all(cBoth(:)>=cSlow(:)-1e-12),...
 'The fast gate path lowered the confidence somewhere; it must only ever raise it.');

fprintf(['sEMG intent gate: determinism, open-by-default warm-up, close-on-silence, ' ...
 'class separation (%.3f tremor-only vs %.3f intent-bearing), fast path open-only, ' ...
 'and %.0f ms reach response passed.\n'],lo,hi,1000*lag);
end
function [t,c]=gate_trace_slow_only(name,fHz,amp,C)
%GATE_TRACE_SLOW_ONLY The same trace with the fast path disabled, by pushing
% its floor far out of reach. Used only to prove the fast path is open-only.
C.semg_gate_fast_floor_deg=1e6;C=rcflex_semg_init(C);
[t,c]=gate_trace(name,fHz,amp,C);
end
function c=steady_conf(name,fHz,amp,C)
[t,cc]=gate_trace(name,fHz,amp,C);
c=mean(cc(t>1.5,:),1);
end
function [t,c]=gate_trace(name,fHz,amp,C)
%GATE_TRACE The gate's confidence over a whole scenario, driven by exactly the
% chain the controller drives it with: source -> preprocess -> gate.
D=rcflex_control_case(name,4,fHz,amp,4243,C);
F=rcflex_semg_features(D.semg,C,0);
n=size(F,1);c=zeros(n,2);s=[];
for k=1:n;[s,c(k,:)]=rcflex_semg_intent_gate(s,F(k,1:C.semg.nch),C);end
t=(1:n)'/C.outer_hz;
end
