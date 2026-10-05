function S=START_PROJECT(mode,varargin)
%START_PROJECT Single entry point for everything this project produces.
%
%   START_PROJECT('setup')        paths, output folders, Simulink cache. Nothing else.
%   START_PROJECT('test')         every unit and regression test.
%   START_PROJECT('benchmark')    the 20-case x 6-arm controlled sweep, its summary
%                                 and its three figures.
%   START_PROJECT('fusion')       sEMG intent-gate calibration and the E2 fusion
%                                 experiment, with their figures.
%   START_PROJECT('fastmove')     the fast-voluntary-movement detector comparison
%                                 (amplitude vs rate vs hybrid).
%   START_PROJECT('jointtravel')  the authority-limiter joint-travel experiment.
%   START_PROJECT('realdata')     public-dataset (GRABMyo) sEMG validation.
%   START_PROJECT('realtremor')   replay recorded human tremor (PhysioNet tremordb)
%                                 through the three controllers, with its figure.
%   START_PROJECT('validation')   test + jointtravel + fastmove + realdata, i.e. every
%                                 piece of evidence EXCEPT the hour-long sweep.
%   START_PROJECT('figures')      regenerate every report figure from saved results.
%   START_PROJECT('all')          the complete final analysis, in dependency order.
%
% Design demonstrations, unchanged from earlier revisions:
%   'preview' 'simulate' 'control' 'compare' 'soup' 'analysis'
%
% OPTIONS (name/value, any mode):
%   'Force',true      rebuild the geometry assets even if they are current.
%   'Quick',true      where a mode supports it, run the short variant. Modes
%                     that cannot be shortened honestly ignore it and say so.
%   'DryRun',true     list the steps a mode would run and the outputs it would
%                     write, then stop. Nothing is executed. Use it to check the
%                     wiring of a long mode before committing an hour to it.
%
% Every mode prints what it is doing, how long each step took, and where its
% outputs were written. A step that fails stops the mode and reports which step
% failed and why; it does not carry on and leave a half-written CSV looking like
% a finished result.
%
% The underlying functions (benchmark_rcflex_control, summarise_rcflex_benchmark,
% run_semg_intent_experiment, validate_rcflex_semg_dataset, ...) all remain
% callable on their own for debugging. START_PROJECT is the orchestration layer,
% not a replacement for them.
if nargin<1;mode='preview';end
opt=parse_options(varargin);

root=fileparts(mfilename('fullpath'));
addpath(root,genpath(fullfile(root,'src')),fullfile(root,'tests'),fullfile(root,'models'));
for area={'assets','figures','results','cache','models'}
 p=project_path(area{1});if ~isfolder(p);mkdir(p);end
end
Simulink.fileGenControl('set','CacheFolder',project_path('cache'),...
 'CodeGenFolder',fullfile(project_path('cache'),'codegen'),'createDir',true);

modes={'setup','preview','test','simulate','control','compare','soup',...
 'benchmark','fusion','fastmove','jointtravel','realdata','validation',...
 'figures','analysis','all'};
mode=validatestring(mode,modes);
S=[];
if strcmp(mode,'setup')
 fprintf('START_PROJECT: paths and output folders ready.\n  results -> %s\n  figures -> %s\n',...
  project_path('results'),project_path('figures'));
 return
end

% Geometry assets are a prerequisite of every visual mode and cost nothing to
% check, so the check lives here rather than being repeated in each step.
ensure_assets(opt.Force);

steps=plan(mode,opt);
if opt.DryRun
 S=dry_run(mode,steps);
 return
end
S=run_plan(mode,steps);
end

function S=dry_run(mode,steps)
%DRY_RUN List what a mode would do, and check it could do it, without doing it.
% Every step is resolved to a real function and every declared output path is
% printed. This is what makes it cheap to verify that all sixteen modes are
% wired correctly -- running them all end to end takes hours.
fprintf('\nSTART_PROJECT(''%s'') would run %d step(s):\n\n',mode,size(steps,1));
S=struct('mode',mode,'steps',{steps(:,1)},'seconds',[],'outputs',{{}});
outs={};
for k=1:size(steps,1)
 f=functions(steps{k,2});
 fprintf('  %d. %-46s %s\n',k,steps{k,1},f.function);
 for j=1:numel(steps{k,3})
  fprintf('       -> %s%s\n',steps{k,3}{j},tern(isfile(steps{k,3}{j}),' (exists)',' (NOT YET PRESENT)'));
  outs{end+1}=steps{k,3}{j}; %#ok<AGROW>
 end
end
S.outputs=outs(:);
fprintf('\nDry run only -- nothing was executed.\n');
end

function s=tern(c,a,b);if c;s=a;else;s=b;end;end

% =========================================================================
% Plans. One row per step: {label, thunk, {outputs it is expected to write}}
% =========================================================================
function steps=plan(mode,opt)
switch mode
 case 'preview'
  steps={
   'assembled view',   @()preview_rcflex_design('assembled'), {fig('rcflex_assembled.png')}
   'cutaway view',     @()preview_rcflex_design('cutaway'),   {fig('rcflex_cutaway.png')}
   'spoon closeup',    @()preview_spoon_closeup(),            {fig('spoon_curved_neck_closeup.png')}};
 case 'test'
  steps={'validation suite',@()run_validation(),{res('validation_summary.csv')}};
 case 'simulate'
  steps={
   'analytic voltage run',@()assignin('base','S_sim',run_rcflex_analytic('voltage',6)),{}
   'plant plots',         @()plot_rcflex_plant(evalin('base','S_sim')),{fig('rcflex_voltage_response.png')}
   'animation',           @()animate_rcflex_analytic(evalin('base','S_sim'),fig('voltage_test.mp4'),'assembled'),{fig('voltage_test.mp4')}};
 case 'control'
  steps={
   'closed-loop run',  @()assignin('base','S_ctl',run_rcflex_control()),{}
   'numerics check',   @()verify_rcflex_control_numerics(),{res('control_numerical_verification.csv')}
   'control plots',    @()plot_rcflex_control(evalin('base','S_ctl')),{fig('control_response.png')}
   'animation',        @()animate_rcflex_analytic(evalin('base','S_ctl'),fig('control_demo.mp4'),'assembled'),{fig('control_demo.mp4')}};
 case 'compare'
  steps={'comparison animation',@()animate_rcflex_comparison(),{fig('control_comparison.mp4')}};
 case 'soup'
  steps={'soup demo',@()run_rcflex_soup_demo(5),{fig('soup_5ml_comparison.png')}};
 case 'analysis'
  steps={'tremor robustness',@()evaluate_rcflex_robustness(),{res('tremor_robustness.csv')}};

 case 'benchmark'
  % The sweep writes control_benchmark.csv incrementally; the summary and the
  % figures are pure functions of that file, so they always follow it and are
  % never run against a stale one.
  steps={
   'controlled sweep (6 arms x 20 cases)',@()benchmark_rcflex_control(),{res('control_benchmark.csv')}
   'benchmark summary',                   @()summarise_rcflex_benchmark(),{res('control_benchmark_summary.csv')}
   'benchmark figures',                   @()plot_rcflex_benchmark(),...
     {fig('bench_suppression.png'),fig('bench_attitude_error.png'),fig('bench_delta_vs_baseline.png')}
   'results report',                      @()write_report(),{res('CONTROL_RESULTS.md')}};
  if opt.Quick
   steps{1,1}='controlled sweep (sEMG arm only -- QUICK, merges into the existing CSV)';
   steps{1,2}=@()benchmark_rcflex_control("ekf_cv_semg");
  end

 case 'fusion'
  % Calibration first: it produces the constants the gate uses, and the
  % experiment is only meaningful against constants that are current.
  steps={
   'intent-gate calibration',@()calibrate_rcflex_semg_intent(),{res('semg_intent_calibration.csv')}
   'E2 fusion experiment',   @()run_semg_intent_experiment(),{res('semg_intent_e2.csv'),fig('semg_intent_separation.png')}
   'gate trace figure',      @()plot_rcflex_semg_gate(),{fig('semg_gate_reach.png')}};

 case 'fastmove'
  steps={'fast-movement detector comparison',@()run_fast_movement_experiment(),...
   {res('fast_movement_thresholds.csv'),res('fast_movement_separability.csv'),...
    res('fast_movement_detectors.csv'),res('fast_movement_closed_loop.csv'),...
    fig('fast_movement_detectors.png')}};

 case 'jointtravel'
  steps={'joint-travel authority experiment',@()run_joint_travel_experiment(),...
   {res('joint_travel_authority.csv'),fig('joint_travel_authority.png')}};

 case 'realdata'
  steps={
   'six-stage dataset gate',   @()run_dataset_gate(),{res('semg_dataset_validation.csv')}
   'real-data class separation',@()validate_rcflex_semg_separation(),...
     {res('semg_separation_real.csv'),res('semg_separation_real_summary.csv'),fig('semg_separation_real.png')}};

 case 'realtremor'
  % E4. Replays 60 recorded human tremors through the three report arms. The
  % figure is a pure function of the CSV, so it always follows the sweep that
  % produced it rather than an older one left on disk.
  steps={
   'recorded-tremor replay', @()run_real_tremor_experiment(),{res('real_tremor_replay.csv')}
   'recorded-tremor summary',@()summarise_real_tremor(),...
     {res('real_tremor_summary.csv'),res('real_tremor_paired.csv'),res('REAL_TREMOR_RESULTS.md')}
   'real-tremor figure',     @()plot_report_real_tremor(),{rfig('fig_real_tremor.png')}};

 case 'validation'
  steps=[plan('test',opt);plan('jointtravel',opt);plan('fastmove',opt);plan('realdata',opt)];

 case 'figures'
  % Regenerates figures from SAVED results wherever a saved result exists, so
  % this mode is cheap and cannot silently produce a figure from a different
  % run than the CSV beside it. The gate trace and the design views have no
  % intermediate file to read and are recomputed; both are seconds, not hours.
  steps={
   'benchmark figures',  @()plot_rcflex_benchmark(),...
     {fig('bench_suppression.png'),fig('bench_attitude_error.png'),fig('bench_delta_vs_baseline.png')}
   'E2 fusion figure',   @()plot_semg_intent_experiment(),{fig('semg_intent_separation.png')}
   'fast-movement figure',@()plot_fast_movement_experiment(),{fig('fast_movement_detectors.png')}
   'joint-travel figure',@()plot_joint_travel_experiment(),{fig('joint_travel_authority.png')}
   'real-data figure',   @()plot_semg_separation(),{fig('semg_separation_real.png')}
   'gate trace figure',  @()plot_rcflex_semg_gate(),{fig('semg_gate_reach.png')}
   'design views',       @()preview_all(),{fig('rcflex_assembled.png'),fig('rcflex_cutaway.png')}
   'IEEE report figures',@()report_figures(),...
     {rfig('fig_benchmark_suppression.png'),rfig('fig_semg_ablation.png'),...
      rfig('fig_real_tremor.png'),rfig('fig_reach_gate.png')}};

 case 'all'
  % Dependency order: tests gate everything; the cheap experiments run before
  % the hour-long sweep so a mistake surfaces early; figures come last so they
  % are generated from results this run produced.
  steps=[plan('test',opt)
         plan('jointtravel',opt)
         plan('fastmove',opt)
         plan('realdata',opt)
         plan('realtremor',opt)
         plan('fusion',opt)
         plan('benchmark',opt)
         plan('preview',opt)];
 otherwise
  error('SensoSpoon:StartProject','No plan for mode ''%s''.',mode);
end
end

% =========================================================================
% Runner
% =========================================================================
function S=run_plan(mode,steps)
n=size(steps,1);
fprintf('\n================ START_PROJECT(''%s'') : %d step(s) ================\n',mode,n);
produced={};elapsed=zeros(n,1);S=struct('mode',mode,'steps',{steps(:,1)},'seconds',[],'outputs',{{}});
t0=tic;
for k=1:n
 fprintf('\n[%d/%d] %s\n',k,n,steps{k,1});
 fprintf('%s\n',repmat('-',1,66));
 tk=tic;
 try
  steps{k,2}();
 catch err
  elapsed(k)=toc(tk);
  fprintf(2,'\n!! STEP %d/%d FAILED after %.1f s: %s\n',k,n,elapsed(k),steps{k,1});
  fprintf(2,'   %s\n',err.message);
  fprintf(2,'   Mode ''%s'' stopped here. Nothing after this step ran, so any output\n',mode);
  fprintf(2,'   file from a later step is left over from a previous run and is STALE.\n');
  rethrow(err);
 end
 elapsed(k)=toc(tk);
 fprintf('    done in %s\n',dur(elapsed(k)));
 for j=1:numel(steps{k,3})
  f=steps{k,3}{j};
  if isfile(f)
   produced{end+1}=f; %#ok<AGROW>
  else
   % stdout, not stderr: a missing declared output is worth flagging loudly, but
   % it does not stop the mode, and writing it to stderr makes a completed run
   % look like a failed one to a harness that watches stderr.
   fprintf('    [!] expected output NOT FOUND: %s\n',f);
  end
 end
end
S.seconds=elapsed;S.outputs=produced(:);
fprintf('\n================ START_PROJECT(''%s'') complete in %s ================\n',mode,dur(toc(t0)));
if isempty(produced)
 fprintf('No files were declared for this mode.\n');
else
 fprintf('Outputs (%d):\n',numel(produced));
 r=project_path('results');g=project_path('figures');
 for k=1:numel(produced)
  fprintf('  %s\n',shorten(produced{k},r,g));
 end
 fprintf('\n  results -> %s\n  figures -> %s\n',r,g);
end
end

% =========================================================================
% Helpers
% =========================================================================
function p=fig(name);p=project_path('figures',name);end
function p=rfig(name);p=project_path('figures','report',name);end
function p=res(name);p=project_path('results',name);end

function s=shorten(p,r,g)
s=p;
if startsWith(p,r);s=['results/' p(numel(r)+2:end)];
elseif startsWith(p,g);s=['figures/' p(numel(g)+2:end)];end
end

function s=dur(sec)
if sec<90;s=sprintf('%.1f s',sec);else;s=sprintf('%.1f min',sec/60);end
end

function opt=parse_options(args)
opt=struct('Force',false,'Quick',false,'DryRun',false);
for k=1:2:numel(args)
 assert(k+1<=numel(args),'SensoSpoon:StartProject','Option ''%s'' has no value.',args{k});
 name=validatestring(args{k},fieldnames(opt));
 opt.(name)=logical(args{k+1});
end
end

function ensure_assets(force)
H=rcflex_parameters();asset=project_path('assets','assembly.mat');
rebuild=force||~isfile(asset);
if ~rebuild;d=load(asset,'H');rebuild=~isequaln(d.H,H);end
if rebuild
 fprintf('Geometry assets are out of date -- rebuilding.\n');
 generate_rcflex_assets(H);
end
end

function preview_all()
preview_rcflex_design('assembled');preview_rcflex_design('cutaway');
preview_spoon_closeup();preview_spoon_dimensions();
end

function write_report()
%WRITE_REPORT Regenerate output/results/CONTROL_RESULTS.md.
% write_rcflex_control_report reads two artefacts the BENCHMARK does not
% produce -- the single native closed-loop run and its numerical verification --
% and asserts that the two were produced by the same configuration. Rather than
% error on a fresh checkout, or worse, skip silently and leave a stale report
% beside a fresh CSV, the prerequisites are produced here when they are absent.
% That is the orchestration START_PROJECT exists to do.
need=~isfile(project_path('results','control_native.mat'))||...
     ~isfile(project_path('results','control_numerical_verification.mat'));
if need
 fprintf('  (native closed-loop artefacts missing -- generating them first)\n');
 run_rcflex_control();verify_rcflex_control_numerics();
end
try
 write_rcflex_control_report();
catch err
 if strcmp(err.identifier,'MATLAB:assertion:failed')
  % The stored verification no longer matches the stored native run. Regenerate
  % both and try once more, then let any second failure through unchanged.
  fprintf('  (stored native run and its verification disagree -- regenerating both)\n');
  run_rcflex_control();verify_rcflex_control_numerics();
  write_rcflex_control_report();
 else
  rethrow(err);
 end
end
fprintf('Wrote %s\n',project_path('results','CONTROL_RESULTS.md'));
end

function run_dataset_gate()
%RUN_DATASET_GATE The six-stage real-sEMG gate over every GRABMyo participant
% present, writing one CSV per participant plus a pooled one. Kept here rather
% than in the gate itself because the gate scores ONE dataset; choosing which
% datasets exist is an orchestration question.
C=rcflex_control_parameters('ekf_predictive');
C.semg_enabled=true;C=rcflex_prepare_control(C);
root=fullfile(fileparts(mfilename('fullpath')),'data','external','grabmyo');
d=dir(fullfile(root,'raw','session*_participant*'));
all=table();
if isempty(d)
 f=fullfile(root,'grabmyo_p1s1_fixture.mat');
 assert(isfile(f),'SensoSpoon:StartProject',...
  ['No GRABMyo data under %s. Run data/external/grabmyo/fetch_grabmyo_subset.sh, ' ...
   'or restore the committed fixture grabmyo_p1s1_fixture.mat.'],root);
 R=validate_rcflex_semg_dataset(f,C,false);
 R.dataset=repmat("P1s1",height(R),1);all=R;
 writetable(R,project_path('results','semg_dataset_validation_p1.csv'));
else
 for k=1:numel(d)
  tok=regexp(d(k).name,'session(\d+)_participant(\d+)','tokens','once');
  if isempty(tok);continue;end
  p=str2double(tok{2});s=str2double(tok{1});
  S=import_grabmyo(root,p,s,C);
  R=validate_rcflex_semg_dataset(S,C,false);
  R.dataset=repmat(sprintf("P%ds%d",p,s),height(R),1);
  writetable(R,project_path('results',sprintf('semg_dataset_validation_p%d.csv',p)));
  all=[all;R]; %#ok<AGROW>
 end
end
writetable(all,project_path('results','semg_dataset_validation.csv'));
end
