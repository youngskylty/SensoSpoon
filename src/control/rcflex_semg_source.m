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
  [clean,tRaw,activation]=rcflex_semg_truth(D.time,D.intent,D.tremor,C,seed+8100);
  raw=rcflex_semg_sample(clean,tRaw,C,seed+8200);
  meta=struct('source','synthetic','seed',seed,'clean',clean,'activation',activation,...
   'ground_truth',true);

 case 'file'
  error('SensoSpoon:SemgSourceNotImplemented',[ ...
   'C.semg_source=''file'' is not implemented yet (deferred until a dataset is chosen).\n' ...
   'To add it, return raw electrode-referred volts on a uniform %g Hz grid with %d channels,\n' ...
   'resampling here if the recording rate differs, and set meta.ground_truth=false.'],...
   C.semg_hz,C.semg_channels);

 case 'device'
  error('SensoSpoon:SemgSourceNotImplemented',[ ...
   'C.semg_source=''device'' is not implemented yet (no physical front end built).\n' ...
   'See docs/SEMG_SENSOR.md for the hardware architecture this source is expected to read from.']);

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
