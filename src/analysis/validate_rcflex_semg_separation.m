function [T,R]=validate_rcflex_semg_separation(writeOutputs)
%VALIDATE_RCFLEX_SEMG_SEPARATION Real-data test of the intent-gate class separation.
%
% WHAT THIS ANSWERS, AND WHAT IT DOES NOT.
% The sEMG+IMU fusion rests on one number: the voluntary-drive observable
% separates the two classes the estimator must tell apart. In simulation that
% number is 6.3 : 1 on the slow path (3.0 : 1 on the fast path), against a
% >= 4 : 1 design target (docs/SEMG_INTENT_FUSION.md). It has never been
% measured on a human, because no public dataset records forearm sEMG
% simultaneously with a measured tremor -- see docs/SEMG_DATASET.md for the
% survey behind that statement.
%
% This function measures the part of the claim that public data CAN support:
%
%   MEASURED (direct):    deliberate voluntary contraction  vs  quiescent rest,
%                         on real forearm sEMG, through the shipped chain, in
%                         the gate's own units.
%   NOT MEASURED:         deliberate movement vs TREMOR. GRABMyo contains no
%                         tremor condition and none is simulated here.
%
% The two are not the same test, and the difference has a known SIGN. The
% simulated involuntary class is tremor-DRIVEN muscle activation -- real muscle
% activity, merely rhythmic and small -- whereas rest is the absence of
% voluntary drive altogether. A rest segment therefore sits at or below where a
% tremor segment would sit, so:
%
%   the ratio measured here is an UPPER BOUND on the intent/tremor separation
%   a real tremulous subject would give.
%
% An upper bound is still worth having, for two reasons. If it had come out
% BELOW the 4 : 1 target the design would be refuted outright on real data,
% because tremor can only move the number down. And the same run measures the
% thing the synthetic model is least able to predict: the real instrumentation
% floor of the drive metric -- what a quiescent forearm reads through real
% electrodes, real motion artefact and real mains -- which is what decides
% whether the gate can ever be closed at all.
%
% T : per-segment drive table (output/results/semg_separation_real.csv)
% R : summary struct array, per participant and gate path.
if nargin<1;writeOutputs=true;end
C=rcflex_control_parameters('ekf_predictive');
C.semg_enabled=true;C=rcflex_prepare_control(C);

root=fullfile(project_root,'data','external','grabmyo');
subjects=discover_subjects(root);
assert(~isempty(subjects),'SensoSpoon:SemgSeparation',...
 ['No GRABMyo data found under %s. Re-fetch with fetch_grabmyo_subset.sh, or keep the ' ...
  'committed fixture grabmyo_p1s1_fixture.mat in place.'],root);

paths=gate_paths(C);
rows={};R=struct([]);
for si=1:numel(subjects)
 S=subjects(si);
 [raw,tRaw,meta,labels]=load_subject(S,C); %#ok<ASGLU>
 fprintf('\n=== %s: %d samples at %g Hz, %d labelled segments ===\n',...
  S.tag,size(raw,1),C.semg_hz,numel(labels));
 for pi=1:numel(paths)
  u=rcflex_semg_gate_drive(raw,C,paths(pi).coeff,tRaw(1));
  tTick=tRaw(1)+(1:size(u,1))'*C.semg.sub/C.semg_hz;
  if paths(pi).differentiate
   % Rate detector: derivative of the smoothed drive, per axis. Backward
   % difference at the outer rate, exactly as rcflex_semg_intent_gate forms it.
   u=[zeros(1,2);diff(u)]*C.outer_hz;
  end
  drive=u.*C.semg.gate_cal(:)';       % volts -> calibrated units (see note below)
  for k=1:numel(labels)
   L=labels(k);
   % Skip the first 1.5 s of every segment: the gate lowpasses are causal and a
   % segment boundary is a step. This is the same exclusion window the
   % synthetic calibration uses (calibrate_rcflex_semg_intent.m), so the two
   % measurements are made the same way.
   w=tTick>=L.t_start+1.5&tTick<=L.t_end;
   if nnz(w)<round(.5*C.outer_hz);continue;end
   g=0;if isfield(L,'gesture');g=L.gesture;end
   for ax=1:2
    rows(end+1,:)={S.tag,S.participant,S.session,paths(pi).name,k,string(L.class),...
     g,ax,L.t_start,L.t_end,rms(drive(w,ax)),mean(drive(w,ax))}; %#ok<AGROW>
   end
  end
 end
end

T=cell2table(rows,'VariableNames',{'dataset','participant','session','gate_path',...
 'segment','class','gesture','axis','t_start_s','t_end_s','drive_rms','drive_mean'});
T.dataset=string(T.dataset);T.gate_path=string(T.gate_path);

% --- Summary, by the SAME rule the synthetic floor is derived with ----------
% calibrate_rcflex_semg_intent.m summarises each class by its WORST case -- max
% over the no-voluntary-drive class, min over the voluntary class -- and takes
% the geometric mean as the floor. Applying the same rule here, rather than a
% friendlier one such as comparing medians, is what makes the real number
% comparable with the simulated 6.3 : 1.
fprintf('\n');
tags=unique(T.dataset,'stable');
for si=1:numel(tags)
 for pi=1:numel(paths)
  ix=T.dataset==tags(si)&T.gate_path==string(paths(pi).name);
  lo=max(T.drive_rms(ix&T.class=="rest"));
  hi=min(T.drive_rms(ix&T.class=="intent"));
  e=struct('dataset',tags(si),'gate_path',string(paths(pi).name),...
   'rest_max',lo,'intent_min',hi,'separation',hi/max(lo,eps),...
   'floor_geometric_mean',sqrt(lo*hi),...
   'synthetic_separation',paths(pi).synthetic_separation,'target',4);
  if isempty(R);R=e;else;R(end+1)=e;end %#ok<AGROW>
  fprintf('%-8s %-5s  rest max %.3e | voluntary min %.3e | separation %6.1f : 1  (synthetic %.1f : 1, target >= 4 : 1)\n',...
   tags(si),paths(pi).name,lo,hi,e.separation,paths(pi).synthetic_separation);
 end
end
fprintf(['\nUNITS NOTE. drive_rms carries the gate calibration constant\n' ...
 '  C.semg_gate_cal_rad_per_V, which was fitted on the SYNTHETIC front end.\n' ...
 '  GRABMyo was recorded on different hardware at a different gain, so the ABSOLUTE\n' ...
 '  values here are not equivalent wrist angles and must not be read against\n' ...
 '  C.semg_gate_floor_deg. The RATIO is calibration-independent; the ratio is what is\n' ...
 '  compared, and a real deployment would re-fit the constant per subject.\n']);
fprintf(['SCOPE NOTE. The classes here are voluntary contraction vs REST. Tremor is absent\n' ...
 '  from this dataset and is not simulated in, so these ratios BOUND the intent/tremor\n' ...
 '  separation from above. They do not measure it.\n']);

decompose(T);

if writeOutputs
 out=project_path('results','semg_separation_real.csv');
 writetable(T,out);fprintf('\nWrote %s\n',out);
 writetable(struct2table(R),project_path('results','semg_separation_real_summary.csv'));
 fprintf('Wrote %s\n',project_path('results','semg_separation_real_summary.csv'));
 plot_semg_separation(T,R);
end
end

% -------------------------------------------------------------------------
function decompose(T)
%DECOMPOSE Why the pooled worst-case ratio came out the way it did.
%
% The pooled number above is what the project's declared rule gives and it
% stands as the headline. It is also, on its own, misleading, because it pools
% three things the synthetic case set does not contain separately:
%
%   * WRIST FLEXION drives the site chosen as the agonist.
%   * WRIST EXTENSION drives the site chosen as the antagonist -- so it should
%     also produce a large |difference|, and whether it does is a direct test
%     of the reciprocal-activation assumption the whole feature layout rests
%     on (SEMG_IMPLEMENTATION.md 12.2).
%   * The PITCH axis pair is a SUBSTITUTE (docs/SEMG_DATASET.md 5.2): our pitch
%     axis should be FCU/ECU and this is simply another flexor/extensor pair on
%     the same rings. These gestures do not exercise it as an axis, so a low
%     value on it is not evidence that the observable failed.
%
% Splitting them out is not a rescue of the headline -- the headline does not
% change -- it is the difference between reporting a number and explaining it.
fprintf('\n---------------- decomposition of the pooled ratio ----------------\n');
g=T(T.gate_path=="slow",:);
tags=unique(g.dataset,'stable');
fprintf('%-8s %-6s %-22s %12s %12s\n','subject','axis','condition','median drive','max drive');
for si=1:numel(tags)
 for ax=1:2
  axName="roll";if ax==2;axName="pitch";end
  for cond=["flexion (gesture 11)","extension (gesture 12)","rest (gesture 17)"]
   switch extractBefore(cond," ")
    case "flexion";m=g.gesture==11;
    case "extension";m=g.gesture==12;
    otherwise;m=g.gesture==17;
   end
   v=g.drive_rms(g.dataset==tags(si)&g.axis==ax&m);
   if isempty(v);continue;end
   fprintf('%-8s %-6s %-22s %12.3e %12.3e\n',tags(si),axName,cond,median(v),max(v));
  end
 end
end

% --- The finding this decomposition exposes -------------------------------
% A rest segment should read ~0 on an agonist-MINUS-antagonist difference: with
% no drive, both envelopes are noise floors and the difference cancels. On real
% electrodes it does not, because the two channels do not have the same noise
% floor or the same resting tone, and the rectify-and-lowpass envelope turns
% that asymmetry into a standing DC offset that the gate's 1 Hz lowpass passes
% untouched. The synthetic source cannot show this: it generates its channels
% with identical statistics, so its resting difference really is ~0.
fprintf('\nResting baseline of the differential envelope (should be ~0 by design):\n');
for si=1:numel(tags)
 for ax=1:2
  axName="roll";if ax==2;axName="pitch";end
  r=g.drive_mean(g.dataset==tags(si)&g.axis==ax&g.gesture==17);
  f=g.drive_rms(g.dataset==tags(si)&g.axis==ax&g.gesture==11);
  if isempty(r)||isempty(f);continue;end
  fprintf('  %-8s %-6s resting DC %+.3e, i.e. %.0f%% of the flexion drive\n',...
   tags(si),axName,mean(r),100*abs(mean(r))/max(median(f),eps));
 end
end
% --- Testing that explanation rather than asserting it --------------------
% If the standing offset is really what breaks the pooled ratio, then removing
% it should repair the pooled ratio, and nothing else should need to change.
% That is a prediction, so it is checked. For a segment with drive x and a
% resting baseline b, rms(x-b)^2 = rms(x)^2 - 2*b*mean(x) + b^2, so the
% baseline-referred value follows exactly from the two statistics already
% recorded per segment -- no re-analysis and no new free parameter.
fprintf('\nBaseline-referred separation (resting DC subtracted) -- a TEST of that explanation,\n');
fprintf('not an adopted metric:\n');
for si=1:numel(tags)
 lo=0;hi=Inf;
 for ax=1:2
  b=mean(g.drive_mean(g.dataset==tags(si)&g.axis==ax&g.gesture==17));
  m=g.dataset==tags(si)&g.axis==ax;
  ref=@(r,mu)sqrt(max(r.^2-2*b*mu+b^2,0));
  v=ref(g.drive_rms(m&g.gesture==17),g.drive_mean(m&g.gesture==17));
  lo=max(lo,max(v));
  v=ref(g.drive_rms(m&g.gesture~=17),g.drive_mean(m&g.gesture~=17));
  hi=min(hi,min(v));
 end
 fprintf('  %-8s rest max %.3e | voluntary min %.3e | separation %5.1f : 1\n',...
  tags(si),lo,hi,hi/max(lo,eps));
end

fprintf(['\nREADING, in the order the numbers force.\n' ...
 '1. FLEXION separates well. The direction the agonist channel was selected for reads\n' ...
 '   5:1 to 10:1 above rest, comfortably past the >= 4:1 target. The observable works.\n' ...
 '2. EXTENSION does not. The same pair reads 2.5:1 on one subject and BELOW rest on the\n' ...
 '   other, and since the pooled rule takes the worst case, that is what sets the\n' ...
 '   headline. A gate that only sees one direction of voluntary effort is not the gate\n' ...
 '   the design describes.\n' ...
 '3. The standing DC offset is part of the cause but NOT all of it. Subtracting the\n' ...
 '   measured resting baseline lifts the pooled ratio from 0.8 to 1.1 and from 0.1 to\n' ...
 '   1.9 -- real improvement, still far short of 4:1. The hypothesis that the offset\n' ...
 '   alone explains the failure was tested here and is REJECTED.\n' ...
 '4. The residual is co-activation. Stage 3b measures agonist/antagonist envelope\n' ...
 '   correlation at 0.63-0.78 on these same recordings: the two muscles rise together,\n' ...
 '   so their DIFFERENCE cancels much of the drive whenever the antagonist leads.\n' ...
 '   The synthetic source cannot show either effect -- it generates its channels\n' ...
 '   independently and with identical statistics, so its resting difference is ~0 and\n' ...
 '   its reciprocity is perfect by construction.\n' ...
 '\nWHAT THIS DOES AND DOES NOT ESTABLISH. It does not refute the fusion: these are hand\n' ...
 'gestures rather than wrist stabilisation, the channel pairs are chosen empirically, and\n' ...
 'the pitch pair is an acknowledged substitute (docs/SEMG_DATASET.md 5.2). It does\n' ...
 'establish, on real muscle, that the synthetic 6.3:1 is not reproduced in one of the two\n' ...
 'contraction directions, and it names two specific mechanisms the model omits. Neither is\n' ...
 'fixed in this revision: referring the floor to a measured resting baseline and handling\n' ...
 'co-activation are architectural changes, and the experiment that would justify them --\n' ...
 'a real subject, with tremor -- has not been run.\n']);
fprintf('-------------------------------------------------------------------\n');
end

function p=gate_paths(C)
%GATE_PATHS The gate front ends to measure, each carried with the separation it
% achieved on the synthetic source, so the real number is always printed beside
% its simulated counterpart rather than on its own.
p=struct('name',{'slow'},'coeff',{C.semg.gate_lp},'differentiate',{false},...
 'synthetic_separation',{6.3});
p(2)=struct('name','fast','coeff',C.semg.gate_lp_fast,'differentiate',false,...
 'synthetic_separation',3.0);
if isfield(C.semg,'gate_lp_rate')
 % The rate path is measured here whether or not it is the ACTIVE detector,
 % because the question "does this observable separate the classes on real
 % data" is worth answering for a candidate as well as for the shipped arm.
 % Its synthetic counterpart comes from the E3 threshold derivation if that has
 % been run; NaN simply means the comparison column is not available yet.
 syn=NaN;
 f=project_path('results','fast_movement_thresholds.csv');
 if isfile(f)
  TH=readtable(f);
  m=TH.rate_fc_Hz==C.semg_gate_rate_fc_Hz;
  if any(m);syn=TH.separation_ratio(find(m,1));end
 end
 p(3)=struct('name','rate','coeff',C.semg.gate_lp_rate,'differentiate',true,...
  'synthetic_separation',syn);
end
end

function S=discover_subjects(root)
%DISCOVER_SUBJECTS Prefer raw WFDB records; fall back to the committed fixture.
S=struct([]);
d=dir(fullfile(root,'raw','session*_participant*'));
for k=1:numel(d)
 tok=regexp(d(k).name,'session(\d+)_participant(\d+)','tokens','once');
 if isempty(tok);continue;end
 e=struct('tag',sprintf('P%ss%s',tok{2},tok{1}),'participant',str2double(tok{2}),...
  'session',str2double(tok{1}),'root',root,'fixture','');
 if isempty(S);S=e;else;S(end+1)=e;end %#ok<AGROW>
end
if isempty(S)
 f=fullfile(root,'grabmyo_p1s1_fixture.mat');
 if isfile(f)
  S=struct('tag','P1s1','participant',1,'session',1,'root',root,'fixture',f);
 end
end
end

function [raw,tRaw,meta,labels]=load_subject(S,C)
if ~isempty(S.fixture)
 D=S.fixture;
else
 D=import_grabmyo(S.root,S.participant,S.session,C);
end
Cf=C;Cf.semg_source='file';
[raw,tRaw,meta]=rcflex_semg_source(struct('semg_dataset',D),Cf);
labels=meta.labels;
end

function r=project_root()
r=fileparts(fileparts(fileparts(mfilename('fullpath'))));
end

