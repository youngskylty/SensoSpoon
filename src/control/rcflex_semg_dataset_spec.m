function spec=rcflex_semg_dataset_spec(S,C)
%RCFLEX_SEMG_DATASET_SPEC The recorded-sEMG dataset contract, in one place.
% Called with no arguments it RETURNS the contract as a struct, so a recording
% script or a dataset-conversion script can be written against it directly.
% Called with (S,C) it VALIDATES S and errors with the contract if anything is
% wrong.
%
% Keeping the format defined in exactly one function is deliberate: the loader,
% the validation script, the unit test and any future recording tool all read
% the same definition, so a field cannot be added in one place and forgotten in
% another.
%
% ---------------------------------------------------------------------------
% REQUIRED
%   semg    n x nch double, ELECTRODE-REFERRED VOLTS.
%           Not ADC counts, not millivolts, not gain-included. If your
%           acquisition gives counts, convert with
%               volts = counts * (adc_range/2^bits) / analogue_gain
%           Channel order must be
%               [roll agonist, roll antagonist, pitch agonist, pitch antagonist]
%           i.e. [FCR, ECRL/B, FCU, ECU] for the forearm placement in
%           docs/SEMG_SUBSYSTEM.md 1.9.
%   fs      scalar sample rate in Hz. Must be >= 2*C.semg_band_hi_Hz (900 Hz);
%           anything else is regridded to C.semg_hz by the loader.
%
% OPTIONAL -- each unlocks specific validation stages
%   t0          scalar start time (s), default 0.
%   channels    1 x nch cellstr of labels, for provenance and to catch a
%               mis-ordered file. Recommended.
%   imu_accel   m x 2 mechanical reference, tremor ACCELERATION for
%               [roll pitch]. Needed by the lead and frequency-agreement
%               stages, which are the two that decide whether sEMG carries
%               anything the IMU does not already have. Units are arbitrary but
%               must be consistent -- both stages are scale-invariant.
%   imu_fs      scalar rate for imu_accel. Required if imu_accel is present.
%   labels      segment table/struct array with fields t_start, t_end, class.
%               class is one of 'rest', 'tremor', 'intent', 'both'.
%               Needed by the SNR and discrimination stages. Recording these as
%               cued trial boundaries is far more reliable than labelling after
%               the fact.
%   subject, session, device, notes   free-form provenance, carried into meta.
% ---------------------------------------------------------------------------
%
% MINIMUM USEFUL DATASET. semg + fs alone runs acquisition health and
% cross-talk. Adding labels with at least one 'tremor' and one 'rest' segment
% adds the SNR go/no-go. Adding imu_accel adds the lead and frequency stages.
% Nothing here requires a full labelled corpus to start.
spec=struct(...
 'required',{{'semg','fs'}},...
 'optional',{{'t0','channels','imu_accel','imu_fs','labels','subject','session','device','notes'}},...
 'classes',{{'rest','tremor','intent','both'}},...
 'units','electrode-referred volts',...
 'channel_order',{{'roll_agonist','roll_antagonist','pitch_agonist','pitch_antagonist'}});
if nargin==0;return;end

ctx=sprintf(['\n\nThe dataset contract is defined in rcflex_semg_dataset_spec.m;\n' ...
 'call it with no arguments to get the field list, or ''help rcflex_semg_dataset_spec''.']);

assert(isstruct(S),'SensoSpoon:SemgDataset','Dataset must be a struct.%s',ctx);
for f=spec.required
 assert(isfield(S,f{1}),'SensoSpoon:SemgDataset','Dataset is missing required field ''%s''.%s',f{1},ctx);
end
assert(isnumeric(S.semg)&&ismatrix(S.semg)&&~isempty(S.semg),...
 'SensoSpoon:SemgDataset','semg must be a non-empty numeric n x nch matrix.%s',ctx);
assert(isnumeric(S.fs)&&isscalar(S.fs)&&S.fs>0,...
 'SensoSpoon:SemgDataset','fs must be a positive scalar.%s',ctx);
assert(size(S.semg,2)==C.semg_channels,...
 'SensoSpoon:SemgDataset','semg has %d columns; this pipeline expects %d (%s).%s',...
 size(S.semg,2),C.semg_channels,strjoin(spec.channel_order,', '),ctx);
assert(size(S.semg,1)>size(S.semg,2),...
 'SensoSpoon:SemgDataset',...
 'semg is %d x %d -- it looks transposed. Rows are samples, columns are channels.%s',...
 size(S.semg,1),size(S.semg,2),ctx);
assert(all(isfinite(S.semg(:))),'SensoSpoon:SemgDataset','semg contains NaN or Inf.%s',ctx);

% A units sanity check. Electrode-referred sEMG is sub-10 mV; a stream in the
% volts is almost certainly gain-included or in ADC counts, and would silently
% produce plausible-looking but wrong features.
pk=max(abs(S.semg(:)));
assert(pk<0.1,'SensoSpoon:SemgDataset',[ ...
 'Peak |semg| is %.3g, which is too large for electrode-referred volts (expect < 0.01). ' ...
 'This is usually gain-included data or raw ADC counts -- divide by the analogue gain first.%s'],pk,ctx);
if pk<1e-5
 warning('SensoSpoon:SemgDataset',...
  'Peak |semg| is %.3g V, unusually small for sEMG. Check the units and the electrode contact.',pk);
end

if isfield(S,'channels')
 assert(numel(S.channels)==C.semg_channels,'SensoSpoon:SemgDataset',...
  'channels has %d entries, expected %d.%s',numel(S.channels),C.semg_channels,ctx);
end
if isfield(S,'imu_accel')
 assert(isfield(S,'imu_fs')&&isscalar(S.imu_fs)&&S.imu_fs>0,...
  'SensoSpoon:SemgDataset','imu_accel requires a positive scalar imu_fs.%s',ctx);
 assert(size(S.imu_accel,2)>=2,'SensoSpoon:SemgDataset',...
  'imu_accel needs at least 2 columns ([roll pitch] tremor acceleration).%s',ctx);
end
if isfield(S,'labels')&&~isempty(S.labels)
 L=S.labels;if istable(L);L=table2struct(L);end
 for k=1:numel(L)
  for f={'t_start','t_end','class'}
   assert(isfield(L(k),f{1}),'SensoSpoon:SemgDataset',...
    'labels(%d) is missing ''%s''.%s',k,f{1},ctx);
  end
  assert(any(strcmpi(char(L(k).class),spec.classes)),'SensoSpoon:SemgDataset',...
   'labels(%d).class is ''%s''; expected one of: %s.%s',...
   k,char(L(k).class),strjoin(spec.classes,', '),ctx);
  assert(L(k).t_end>L(k).t_start,'SensoSpoon:SemgDataset',...
   'labels(%d) has t_end <= t_start.%s',k,ctx);
 end
end
end
