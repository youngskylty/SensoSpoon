function [raw,tRaw,meta]=rcflex_semg_source(D,C)
%RCFLEX_SEMG_SOURCE The one place a raw sEMG stream is obtained.
% Every sEMG consumer goes through here, so the synthetic generator, a recorded
% dataset and the physical sensor are interchangeable behind a single switch
% (C.semg_source) without any change to preprocessing, feature extraction, the
% characterisation script or the controller.
%
% Selected by C.semg_source:
%   'synthetic'  generated from the scenario's intent/tremor ground truth
%                (rcflex_semg_truth + rcflex_semg_sample). Implemented.
%   'file'       a recorded dataset. NOT implemented -- deliberately deferred.
%   'device'     a live/logged capture from the physical front end. NOT
%                implemented.
%
% THE RAW STREAM CONTRACT. Whatever the source, it must return:
%
%   raw   n x C.semg_channels, ELECTRODE-REFERRED volts -- i.e. the ADC reading
%         divided back through the instrumentation gain, so downstream code
%         works in physical units and is independent of the gain actually used.
%         Channel order [roll agonist, roll antagonist, pitch agonist,
%         pitch antagonist].
%   tRaw  n x 1 sample times (s), uniformly spaced at exactly 1/C.semg_hz and
%         starting at the scenario start. Uniform spacing is not a convenience:
%         the whole synchronisation scheme is that C.semg_hz is an integer
%         multiple of C.outer_hz, so tick j consumes samples (j-1)*sub+1 : j*sub
%         with no resampling. A source that cannot deliver a uniform grid at
%         that rate must resample to it HERE, before returning.
%   meta  struct describing provenance; must carry a .source field. Anything
%         else is source-specific and is recorded, not interpreted.
%
% A source is responsible for delivering raw electrode volts and nothing more.
% All filtering, rectification and feature extraction belong downstream, in
% rcflex_semg_preprocess.m, so that every source is processed identically and a
% comparison between them is meaningful.
source='synthetic';
if isfield(C,'semg_source')&&~isempty(C.semg_source);source=C.semg_source;end

switch source
 case 'synthetic'
  assert(isfield(D,'intent')&&isfield(D,'tremor'),...
   'SensoSpoon:Semg','The synthetic source needs a case with intent and tremor truth (rcflex_control_case).');
  seed=C.seed;
  if isfield(D,'seed');seed=D.seed;end
  % rcflex_semg_truth works on the ANALOGUE (oversampled) grid; sampling and
  % decimation to C.semg_hz happen inside rcflex_semg_sample, after the
  % anti-alias filter. Only the decimated stream leaves this function, so every
  % consumer still sees a plain C.semg_hz grid.
  [clean,tAnalog,activation]=rcflex_semg_truth(D.time,D.intent,D.tremor,C,seed+8100);
  [raw,tRaw]=rcflex_semg_sample(clean,tAnalog,C,seed+8200);
  meta=struct('source','synthetic','seed',seed,'ground_truth',true,...
   'analog_time',tAnalog,'clean',clean,'activation',activation,...
   'oversample',C.semg_oversample);

 case 'file'
  % A recorded dataset. Deliberately minimal: one struct (or a .mat holding
  % one), validated and regridded onto the required rate. See
  % rcflex_semg_dataset_spec.m for the field contract -- that function is the
  % single definition of the format, and is what a recording script should be
  % written against.
  src=[];
  if isfield(C,'semg_file')&&~isempty(C.semg_file);src=C.semg_file;end
  if isfield(D,'semg_dataset')&&~isempty(D.semg_dataset);src=D.semg_dataset;end
  assert(~isempty(src),'SensoSpoon:Semg',...
   'C.semg_source=''file'' needs C.semg_file (a path or struct) or D.semg_dataset.');
  [raw,tRaw,meta]=load_dataset(src,C);

 case 'device'
  error('SensoSpoon:SemgSourceNotImplemented',[ ...
   'C.semg_source=''device'' is not implemented yet (no physical front end built).\n' ...
   'See docs/SEMG_SUBSYSTEM.md for the hardware architecture this source is expected to read from.']);

 otherwise
  error('SensoSpoon:SemgSource','Unknown C.semg_source: %s',source);
end

assert(size(raw,2)==C.semg_channels,'SensoSpoon:Semg','Source returned %d channels, expected %d',size(raw,2),C.semg_channels);
assert(numel(tRaw)==size(raw,1),'SensoSpoon:Semg','Source returned mismatched sample/time counts');
if numel(tRaw)>1
 assert(abs((tRaw(2)-tRaw(1))-1/C.semg_hz)<1e-9,...
  'SensoSpoon:Semg','Source grid is %g Hz, not the required %g Hz',1/(tRaw(2)-tRaw(1)),C.semg_hz);
end
end
function [raw,tRaw,meta]=load_dataset(src,C)
%LOAD_DATASET Read, validate and regrid a recorded sEMG dataset.
S=src;
if ischar(S)||isstring(S)
 S=char(S);
 assert(isfile(S),'SensoSpoon:Semg','sEMG dataset not found: %s',S);
 d=load(S);f=fieldnames(d);
 if isscalar(f)&&isstruct(d.(f{1}));S=d.(f{1});else;S=d;end
end
assert(isstruct(S),'SensoSpoon:Semg','sEMG dataset must be a struct, or a path to a .mat holding one');

rcflex_semg_dataset_spec(S,C);   % validates, and errors with the field contract

fs=double(S.fs);x=double(S.semg);

% Refuse data that cannot represent the band the pipeline assumes. Accepting it
% silently would produce features computed over a partly empty band, which
% presents as weak physiology rather than as a wrong recording -- the most
% expensive kind of mistake at this stage.
assert(fs>=2*C.semg_band_hi_Hz,'SensoSpoon:Semg',[ ...
 'Dataset rate %g Hz cannot represent the %g-%g Hz band this pipeline assumes (needs >= %g Hz). ' ...
 'Re-record, or narrow C.semg_band_hi_Hz and re-derive the features.'],...
 fs,C.semg_band_lo_Hz,C.semg_band_hi_Hz,2*C.semg_band_hi_Hz);

% Regrid onto exactly C.semg_hz. This must happen here, not in the caller: the
% fixed-block synchronisation contract is what the whole pipeline rests on.
if abs(fs-C.semg_hz)<1e-9
 raw=x;
else
 [p,q]=rat(C.semg_hz/fs,1e-9);
 raw=resample(x,p,q);
end
n=size(raw,1);tRaw=(0:n-1)'/C.semg_hz;
if isfield(S,'t0');tRaw=tRaw+double(S.t0);end

meta=struct('source','file','ground_truth',false,'fs_original',fs,'n_original',size(x,1));
for f={'subject','notes','channels','session','device'}
 if isfield(S,f{1});meta.(f{1})=S.(f{1});end
end
% Carried through for the validation stages that need them. Absent fields are
% not an error: the corresponding stage reports itself SKIPPED, naming what is
% missing, rather than silently producing a number from nothing.
for f={'imu_accel','imu_fs','labels'}
 if isfield(S,f{1});meta.(f{1})=S.(f{1});end
end
end
