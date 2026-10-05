function T=run_real_tremor_experiment(varargin)
%RUN_REAL_TREMOR_EXPERIMENT E4: replay recorded human tremor through the loop.
%
%   T = RUN_REAL_TREMOR_EXPERIMENT()                 the full sweep
%   T = RUN_REAL_TREMOR_EXPERIMENT('Records',{...})  a named subset
%   T = RUN_REAL_TREMOR_EXPERIMENT('DryRun',true)    plan only, run nothing
%
% THE QUESTION. Every suppression figure this project has published was measured
% against a tremor the project generated itself: a sine, a chirp, or a sum of a
% few sines. Those are smooth, stationary and have a crest factor of 1.41. Real
% Parkinsonian rest tremor is none of those things. This experiment asks what
% the three controllers do when the hand is driven by a waveform recorded from a
% person instead, and answers it over 60 recordings from 14 patients.
%
% WHAT IS CONTROLLED. Each recorded scenario is matched to its synthetic
% counterpart in everything except waveform shape:
%   * the same RMS tremor amplitude (rcflex_tremor_record scales to a unit
%     sine's RMS, not to unit RMS, precisely so this holds);
%   * the same duration, solver step, sensor seed and metrics window;
%   * for the 14 primary records, a paired synthetic control is also run at
%     THAT record's own dominant frequency, so the comparison isolates waveform
%     structure from frequency rather than confounding the two.
% What is left over between a recorded row and its paired control row is
% attributable to the shape of real tremor and to nothing else.
%
% WHAT IT DOES NOT CLAIM. The recordings are index-fingertip velocity, not hand
% attitude, and they are Parkinsonian REST tremor, not the postural tremor a
% person feeding themselves would have. This is a real-signal robustness test of
% the control algorithm; it is not a clinical trial and it is not evidence about
% the sEMG stream, which remains synthetic throughout. See
% docs/VALIDATION_STATUS.md for the tier this result belongs to.
p=inputParser;
p.addParameter('Records',[]);
p.addParameter('Amplitude',1);
p.addParameter('Duration',4);
p.addParameter('Step',5e-4);
p.addParameter('Control',true);      % also run the paired synthetic controls
p.addParameter('DryRun',false);
% Resume an interrupted sweep from its checkpoint. ONLY valid when no code has
% changed since the checkpoint was written: the carried-through rows were
% produced by whatever version was running then, and mixing two versions in one
% CSV would make the file unreproducible from the tree that holds it. Use it to
% recover from a crash or a kill, never to patch a sweep after an edit.
p.addParameter('Resume',false);
p.parse(varargin{:});
o=p.Results;

% Same three arms as the report's headline comparison, defined the same way as
% in benchmark_rcflex_control so the two experiments cannot drift apart.
arms={'simple_pid','cv_legacy',false,'off','simple_pid'
      'ekf_predictive','cv',false,'off','ekf_cv'
      'ekf_predictive','cv',false,'intent_gate','ekf_cv_semg'};

R=import_tremordb();
if isempty(o.Records)
 recs=cellstr(R.name);
else
 recs=cellstr(string(o.Records));
 assert(all(ismember(recs,cellstr(R.name))),'SensoSpoon:RealTremor', ...
  'Unknown record(s): %s',strjoin(setdiff(recs,cellstr(R.name)),', '));
end
prim=cellstr(R.name(R.minutes_off==0));   % one record per subject

nrun=numel(recs)*size(arms,1);
if o.Control;nrun=nrun+numel(intersect(recs,prim))*size(arms,1);end
fprintf('E4 real-tremor replay: %d records x %d arms%s = %d runs\n', ...
 numel(recs),size(arms,1),ternary(o.Control,' (+ paired synthetic controls)',''),nrun);
if o.DryRun;T=table();return;end

% Progress is checkpointed to a .partial file rather than to the real one. A
% five-hour sweep that dies at run 200 should leave something inspectable, but
% it must not leave a CSV that LOOKS finished -- so the partial carries a
% different name and is removed once the complete file is written.
part=project_path('results','real_tremor_replay.partial.csv');
rows=struct([]);skip=strings(0,3);
if o.Resume
 assert(isfile(part),'SensoSpoon:RealTremor', ...
  'Nothing to resume: no checkpoint at %s.',part);
 K=readtable(part,'TextType','string');
 rows=table2struct(K)';
 skip=[K.arm K.input K.record];
 fprintf('Resuming: %d run(s) carried through from the checkpoint, %d to go.
', ...
  height(K),nrun-height(K));
elseif isfile(part)
 delete(part);
end
done=size(skip,1);t0=tic;
for a=1:size(arms,1)
 C=rcflex_control_parameters(arms{a,1});
 C.ekf_intent_model=arms{a,2};C.ekf_osc2=arms{a,3};C.semg_fusion=arms{a,4};
 C.semg_enabled=strcmp(arms{a,4},'intent_gate');
 for k=1:numel(recs)
  % Seed is keyed to the RECORD, not to the loop index, so every arm meets the
  % identical sensor noise on the identical record and adding a record later
  % cannot renumber the seeds of the ones already run.
  seed=9400+find(strcmp(cellstr(R.name),recs{k}),1);
  H=rcflex_parameters;
  if already(skip,arms{a,5},'recorded',recs{k})
   if o.Control&&ismember(recs{k},prim)&&~already(skip,arms{a,5},'sine_matched',recs{k})
    % The recorded run is banked but its paired control is not; rebuild just
    % enough to recover that record's dominant frequency and run the control.
    Ck=C;Ck.tremor_record=recs{k};Ck=rcflex_prepare_control(Ck);Ck.seed=seed;
    Dm=rcflex_control_case('recorded',o.Duration,NaN,o.Amplitude,seed,Ck);
    rows=run_control(rows,C,Dm,H,o,seed,arms{a,5},recs{k});
    done=done+1;report(done,nrun,t0,arms{a,5},recs{k},'sine_matched',rows(end));
    writetable(struct2table(rows),part);
   end
   continue
  end
  Ck=C;Ck.tremor_record=recs{k};Ck=rcflex_prepare_control(Ck);Ck.seed=seed;
  D=rcflex_control_case('recorded',o.Duration,NaN,o.Amplitude,seed,Ck);
  S=run_rcflex_control_fast(D,Ck,H,o.Step);m=rcflex_control_metrics(S);
  rows=append_row(rows,arms{a,5},'recorded',recs{k},D.tremor_meta,m,H,o);
  done=done+1;report(done,nrun,t0,arms{a,5},recs{k},'recorded',m);
  writetable(struct2table(rows),part);

  % Paired synthetic control at this record's own dominant frequency.
  if o.Control&&ismember(recs{k},prim)&&~already(skip,arms{a,5},'sine_matched',recs{k})
   f0=D.tremor_meta.dominant_Hz;
   Cs=C;Cs.tremor_record='';Cs=rcflex_prepare_control(Cs);Cs.seed=seed;
   Ds=rcflex_control_case('sine',o.Duration,f0,o.Amplitude,seed,Cs);
   Ss=run_rcflex_control_fast(Ds,Cs,H,o.Step);ms=rcflex_control_metrics(Ss);
   rows=append_row(rows,arms{a,5},'sine_matched',recs{k},D.tremor_meta,ms,H,o);
   done=done+1;report(done,nrun,t0,arms{a,5},recs{k},'sine_matched',ms);
   writetable(struct2table(rows),part);
  end
 end
end

T=struct2table(rows);
f=project_path('results','real_tremor_replay.csv');
writetable(T,f);
if isfile(part);delete(part);end
fprintf('\nWrote %s (%d rows) in %.1f min\n',f,height(T),toc(t0)/60);
end

% -------------------------------------------------------------------------
function tf=already(skip,arm,input,record)
% Has this exact (arm, input, record) run been banked by an earlier attempt?
if isempty(skip);tf=false;return;end
tf=any(skip(:,1)==string(arm)&skip(:,2)==string(input)&skip(:,3)==string(record));
end

% -------------------------------------------------------------------------
function rows=run_control(rows,C,D,H,o,seed,arm,record)
% The paired synthetic control for one record, at that record's own dominant
% frequency. Split out so the resume path and the normal path build it the same
% way rather than by two copies of the same six lines.
Cs=C;Cs.tremor_record='';Cs=rcflex_prepare_control(Cs);Cs.seed=seed;
Ds=rcflex_control_case('sine',o.Duration,D.tremor_meta.dominant_Hz,o.Amplitude,seed,Cs);
Ss=run_rcflex_control_fast(Ds,Cs,H,o.Step);ms=rcflex_control_metrics(Ss);
rows=append_row(rows,arm,'sine_matched',record,D.tremor_meta,ms,H,o);
end

% -------------------------------------------------------------------------
function rows=append_row(rows,arm,input,record,meta,m,H,o)
r=struct('arm',string(arm),'input',string(input),'record',string(record), ...
 'subject',string(meta.subject),'target',string(meta.target), ...
 'group',string(meta.group),'hand',string(meta.hand), ...
 'minutes_off',meta.minutes_off,'dominant_Hz',meta.dominant_Hz, ...
 'crest_factor',meta.crest_factor,'in_design_band',double(meta.in_design_band), ...
 'amplitude_deg',o.Amplitude,'duration_s',o.Duration);
f=fieldnames(m);for j=1:numel(f);r.(f{j})=m.(f{j});end
% Same envelope test the benchmark applies, so a row of this CSV can be read
% beside a row of control_benchmark.csv without translating the flags.
r.within_limits=double(m.joint_peak_deg<=15&&m.stroke_peak_mm<=H.vca_stroke_mm&& ...
 m.current_peak_A<=H.current_limit_A+1e-9&&m.side_load_peak_N<=H.shaft_side_load_N&& ...
 m.power_mean_W<=H.coil_continuous_power_W);
if isempty(rows);rows=r;else;rows(end+1)=r;end
end

% -------------------------------------------------------------------------
function report(done,n,t0,arm,rec,input,m)
el=toc(t0);eta=el/done*(n-done);
fprintf('  [%3d/%3d] %-13s %-12s %-14s supp %7.2f%%  rms %6.4f deg   eta %4.1f min\n', ...
 done,n,arm,input,rec,m.suppression_percent,m.attitude_rms_deg,eta/60);
end
function o=ternary(c,a,b);if c;o=a;else;o=b;end;end
