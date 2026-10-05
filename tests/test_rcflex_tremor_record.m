function test_rcflex_tremor_record()
%TEST_RCFLEX_TREMOR_RECORD Cover for the recorded-human-tremor input path.
%
% The recorded-tremor scenario exists so that suppression can be measured
% against a waveform a person actually produced rather than one this project
% generated. Three properties have to hold for that comparison to mean
% anything, and each is asserted below:
%
%   (1) THE NEW PATH IS INERT BY DEFAULT. With C.tremor_record empty, every
%       synthetic scenario must be bit-identical to what it was before the
%       branch existed. Otherwise the whole published benchmark silently moves.
%   (2) RECORDED AND SYNTHETIC ARE RMS-MATCHED. 'recorded at 1 deg' and 'sine
%       at 1 deg' must present the same tremor ENERGY, or a comparison between
%       them measures the scaling rather than the waveform.
%   (3) THE RECORDS ARE WHAT THE DATABASE SAYS THEY ARE. Sample counts, units
%       and sampling rates are checked against file_description.txt, so a
%       truncated or half-downloaded record fails here rather than quietly
%       producing a plausible-looking result.
%
% It also asserts that real tremor is measurably NOT a sine, which is the
% premise of the whole experiment: if the conditioned records had a crest
% factor of 1.41 there would be nothing to test.

%% (3) every record available here agrees with the published metadata
% Runs against the full downloaded set when it is present and against the
% committed fixture otherwise, so this test never depends on a download. The
% fixture deliberately spans the crest-factor range and both velocity units, so
% the cheap path still exercises what the expensive one does.
R=import_tremordb();
assert(height(R)>0,'No tremordb records found. Run fetch_tremordb_subset.sh.');
full=height(R)>10;
% KNOWN THIRD-PARTY DISCREPANCY. file_description.txt disagrees with the signal
% it describes for exactly one record: v5rof.let holds 3380 samples where the
% table says 3400, a shortfall of 0.2 s in a 33.8 s recording. The file's
% SHA-256 matches the SHA256SUMS.txt that PhysioNet publishes alongside it, so
% the FILE is authentic and the TABLE is wrong -- this is not a truncated
% download. It is listed by name, with the exact delta, rather than absorbed
% into a percentage tolerance, so that a genuinely truncated download still
% fails this test instead of slipping under a threshold.
known={'v5rof.let',-20};
crest=zeros(height(R),1);
% The reference is the ANALYTIC RMS of a unit sine, which is what the
% conditioning normalises to. A sine SAMPLED on this grid reads about 6e-5 low,
% because 0:1/2000:4 duplicates the endpoint and so carries one extra zero
% crossing; comparing against the sampled value would test that artefact rather
% than the normalisation.
t=(0:1/2000:4)';
ref=1/sqrt(2);
assert(abs(rms(sin(2*pi*8*t))-ref)<1e-3,'Sampled-sine sanity check drifted.');
for i=1:height(R)
 nm=char(R.name(i));
 allow=0;
 j=find(strcmp(known(:,1),nm),1);
 if ~isempty(j);allow=known{j,2};end
 % One pass per record: rcflex_tremor_record carries the sample-count facts
 % through in its metadata, so the integrity check and the conditioning check
 % share a single read of the file instead of each paying for their own.
 [sh,m]=rcflex_tremor_record(nm,t);
 assert(m.samples-m.samples_expected==allow, ...
  ['Record %s has %d samples but file_description.txt says %d (expected offset %d). ' ...
   'Re-run fetch_tremordb_subset.sh and check the file against SHA256SUMS.txt.'], ...
  nm,m.samples,m.samples_expected,allow);
 assert(m.fs==100,'Record %s reports fs=%g, expected 100 Hz.',nm,m.fs);
 assert(all(isfinite(sh(:))),'Record %s conditioned to non-finite samples.',nm);
 assert(size(sh,2)==2,'Shape must be [roll pitch] for %s.',nm);
 assert(abs(rms(sh(:,1))-ref)<1e-9, ...
  'Record %s roll RMS %.6f does not match a unit sine''s %.6f.',nm,rms(sh(:,1)),ref);
 assert(abs(rms(sh(:,2))-ref)<1e-9, ...
  'Record %s pitch RMS %.6f does not match a unit sine''s %.6f.',nm,rms(sh(:,2)),ref);
 assert(~isequal(sh(:,1),sh(:,2)),'Roll and pitch must not be identical for %s.',nm);
 crest(i)=m.crest_factor;
end

%% the premise: conditioned real tremor is not a sine
assert(median(crest)>1.8, ...
 ['Median crest factor %.2f is close to a sine''s 1.414. If real records look ' ...
  'like sines the experiment has nothing to measure -- check the conditioning.'],median(crest));

%% (1) the branch is inert when no record is selected
C=rcflex_control_parameters('ekf_predictive');C.ekf_intent_model='cv';
C.semg_fusion='off';C.semg_enabled=false;
assert(isempty(C.tremor_record),'C.tremor_record must default to empty.');
C=rcflex_prepare_control(C);C.seed=9102;H=rcflex_parameters;
D=rcflex_control_case('sine',4,8,1,C.seed,C);
S=run_rcflex_control_fast(D,C,H,5e-4);m=rcflex_control_metrics(S);
T=readtable(project_path('results','control_benchmark.csv'),'TextType','string');
q=T(T.arm=="ekf_cv"&T.case_id==2,:);
assert(height(q)==1,'Expected one ekf_cv row for benchmark case 2.');
d=abs(m.suppression_percent-q.suppression_percent);
assert(d<1e-9, ...
 ['Synthetic case 2 moved by %.3e points after the recorded-tremor branch was ' ...
  'added. The branch must be inert when C.tremor_record is empty.'],d);

%% a recorded scenario without a record must fail loudly, not silently
Cr=C;Cr.tremor_record='';
ok=false;
try
 rcflex_control_case('recorded',4,NaN,1,1,Cr);
catch e
 ok=strcmp(e.identifier,'SensoSpoon:ControlCase');
end
assert(ok,'Scenario ''recorded'' with no C.tremor_record must raise SensoSpoon:ControlCase.');

if full;src='downloaded set';else;src='committed fixture';end
fprintf(['test_rcflex_tremor_record: %d records (%s), median crest %.2f ' ...
 '(sine = 1.41), synthetic path unchanged.\n'],height(R),src,median(crest));
end
