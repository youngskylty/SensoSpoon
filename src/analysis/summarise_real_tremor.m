function [H,P]=summarise_real_tremor()
%SUMMARISE_REAL_TREMOR Headline table for the recorded-tremor replay (E4).
%
%   [H,P] = SUMMARISE_REAL_TREMOR()
%
% H is the per-arm headline. P is the paired synthetic-vs-recorded comparison.
% Both are written to output/results/, and the markdown the report quotes is
% written by this same function, so the table and the CSV cannot disagree.
%
% THREE AVERAGES ARE REPORTED TOGETHER, for the same reason the benchmark
% reports three: an exclusion that is applied silently is an exclusion that
% flatters the result.
%   all recorded          every one of the records, whatever its frequency
%   in design band        records whose dominant tremor is inside the declared
%                         4-20 Hz envelope. A record below that band is a
%                         statement about the design brief, not about waveform
%                         shape, and averaging the two together hides both
%   paired                the subset carrying a synthetic control at the SAME
%                         dominant frequency, which is the only comparison
%                         entitled to attribute a difference to waveform shape
f=project_path('results','real_tremor_replay.csv');
assert(isfile(f),'SensoSpoon:RealTremor', ...
 'No replay at %s. Run START_PROJECT(''realtremor'') first.',f);
T=readtable(f,'TextType','string');

arms=unique(T.arm,'stable');
rec=T(T.input=="recorded",:);
ctl=T(T.input=="sine_matched",:);
pairRec=intersect(unique(rec.record),unique(ctl.record));

rows=struct([]);prows=struct([]);
for a=1:numel(arms)
 ra=rec(rec.arm==arms(a),:);
 inb=ra(ra.in_design_band==1,:);
 % min() of an empty column returns empty, which would silently produce a
 % malformed row rather than an error, so the empty case is made explicit.
 if isempty(inb)
  mband=NaN;wband=NaN;
 else
  mband=mean(inb.suppression_percent);wband=min(inb.suppression_percent);
 end
 r=struct('arm',arms(a), ...
  'n_records',height(ra), ...
  'mean_all_recorded',mean(ra.suppression_percent), ...
  'n_in_design_band',height(inb), ...
  'mean_in_design_band',mband, ...
  'worst_in_design_band',wband, ...
  'n_meets_90_in_band',sum(inb.suppression_percent>=90), ...
  'attitude_rms_mean_deg',mean(ra.attitude_rms_deg), ...
  'attitude_rms_worst_deg',max(ra.attitude_rms_deg), ...
  'n_within_limits',sum(ra.within_limits));
 if isempty(rows);rows=r;else;rows(end+1)=r;end %#ok<AGROW>

 % Paired: same record, same seed, same amplitude, same duration; the synthetic
 % control runs at that record's own dominant frequency, so frequency is held
 % and only the waveform shape differs.
 sr=nan(numel(pairRec),1);sc=sr;
 ca=ctl(ctl.arm==arms(a),:);
 for k=1:numel(pairRec)
  vr=ra.suppression_percent(ra.record==pairRec(k));
  vc=ca.suppression_percent(ca.record==pairRec(k));
  assert(isscalar(vr)&&isscalar(vc),'SensoSpoon:RealTremor', ...
   'Expected exactly one recorded and one control row for %s / %s.',arms(a),pairRec(k));
  sr(k)=vr;sc(k)=vc;
 end
 d=sr-sc;
 p=struct('arm',arms(a),'n_paired',numel(pairRec), ...
  'mean_synthetic',mean(sc),'mean_recorded',mean(sr), ...
  'mean_delta_points',mean(d),'worst_delta_points',min(d), ...
  'n_recorded_worse',sum(d<0));
 if isempty(prows);prows=p;else;prows(end+1)=p;end %#ok<AGROW>
end
H=struct2table(rows);P=struct2table(prows);
writetable(H,project_path('results','real_tremor_summary.csv'));
writetable(P,project_path('results','real_tremor_paired.csv'));

% --- the markdown the report quotes -------------------------------------
u=unique(rec(:,{'record','subject','group','dominant_Hz','crest_factor','in_design_band'}),'rows');
w=fopen(project_path('results','REAL_TREMOR_RESULTS.md'),'w');c=onCleanup(@()fclose(w));
fprintf(w,'# Recorded human tremor: replay results (E4)\n\n');
fprintf(w,['Tremor input taken from PhysioNet `tremordb` (Beuter, Titcombe & Glass), every\n' ...
 'stimulation-off, medication-off record: %d recordings from %d patients. The recordings\n' ...
 'supply waveform SHAPE only and are RMS-matched to a unit sine, because fingertip\n' ...
 'translation cannot be converted into hand attitude without inventing a moment arm.\n' ...
 'Amplitude, duration, solver step, sensor seed and metrics window are identical to the\n' ...
 'synthetic benchmark.\n\n'],height(u),numel(unique(u.subject)));
fprintf(w,['**This is evidence about the control algorithm meeting a real waveform. It is not\n' ...
 'evidence about human physiology, and it says nothing about the sEMG stream, which is\n' ...
 'synthetic throughout and is generated from the same ground truth as before.**\n\n']);

fprintf(w,'## The input, against the synthetic model it replaces\n\n');
fprintf(w,'| | Synthetic sine | Recorded tremor |\n|---|---:|---:|\n');
fprintf(w,'| Crest factor, median (range) | 1.414 | %.2f (%.2f-%.2f) |\n', ...
 median(u.crest_factor),min(u.crest_factor),max(u.crest_factor));
fprintf(w,'| Dominant frequency, median (range) Hz | as commanded | %.2f (%.2f-%.2f) |\n', ...
 median(u.dominant_Hz),min(u.dominant_Hz),max(u.dominant_Hz));
fprintf(w,'| Records inside the declared 4-20 Hz envelope | - | %d of %d |\n\n', ...
 sum(u.in_design_band),height(u));

fprintf(w,'## Headline, by arm\n\n');
fprintf(w,['| Arm | Mean, all %d records (%%) | Mean, in 4-20 Hz band (%%) | Worst in band (%%) ' ...
 '| Meets 90%% | Attitude RMS mean (deg) |\n|---|---:|---:|---:|---:|---:|\n'],height(u));
for k=1:height(H)
 fprintf(w,'| `%s` | %.2f | %.2f | %.2f | %d of %d | %.4f |\n',H.arm(k), ...
  H.mean_all_recorded(k),H.mean_in_design_band(k),H.worst_in_design_band(k), ...
  H.n_meets_90_in_band(k),H.n_in_design_band(k),H.attitude_rms_mean_deg(k));
end

fprintf(w,'\n## Paired against a synthetic sine at the same frequency\n\n');
fprintf(w,['Each row holds the record, the seed, the amplitude and the dominant frequency fixed\n' ...
 'and changes only the waveform, so the delta is attributable to shape alone.\n\n']);
fprintf(w,'| Arm | Synthetic (%%) | Recorded (%%) | Change (points) | Worst change | Records where recorded is worse |\n|---|---:|---:|---:|---:|---:|\n');
for k=1:height(P)
 fprintf(w,'| `%s` | %.2f | %.2f | %+.2f | %+.2f | %d of %d |\n',P.arm(k), ...
  P.mean_synthetic(k),P.mean_recorded(k),P.mean_delta_points(k), ...
  P.worst_delta_points(k),P.n_recorded_worse(k),P.n_paired(k));
end

fprintf(w,'\n## Every record\n\n');
fprintf(w,'| Record | Subject | Group | Dominant Hz | Crest | In band |');
for k=1:numel(arms);fprintf(w,' %s (%%) |',arms(k));end
fprintf(w,'\n|---|---|---|---:|---:|---|');
for k=1:numel(arms);fprintf(w,'---:|');end
fprintf(w,'\n');
u=sortrows(u,{'group','subject','dominant_Hz'});
for i=1:height(u)
 fprintf(w,'| %s | %s | %s | %.2f | %.2f | %d |',u.record(i),u.subject(i),u.group(i), ...
  u.dominant_Hz(i),u.crest_factor(i),u.in_design_band(i));
 for k=1:numel(arms)
  v=rec.suppression_percent(rec.arm==arms(k)&rec.record==u.record(i));
  fprintf(w,' %.2f |',v);
 end
 fprintf(w,'\n');
end
fprintf('Wrote %s\n',project_path('results','REAL_TREMOR_RESULTS.md'));
disp(H);disp(P);
end
