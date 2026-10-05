function R=import_tremordb(record)
%IMPORT_TREMORDB Read one record of the PhysioNet tremordb rest-tremor database.
%
%   R = IMPORT_TREMORDB('g2rof.rit')  reads one recording.
%   R = IMPORT_TREMORDB()             returns a table of every record on disk.
%
% Dataset: "Effect of Deep Brain Stimulation on Parkinsonian Tremor"
% (Beuter, Titcombe & Glass, PhysioNet v1.0.0, Open Access). Index-fingertip
% velocity from a laser transducer, 100 Hz, ~60 s per record. Only the
% stimulation-off, medication-off condition is held locally; see
% data/external/tremordb/fetch_tremordb_subset.sh for exactly what was fetched
% and why.
%
% This function does ONE job: get the samples off disk in SI units with their
% metadata attached. It deliberately does no filtering, integration or
% normalisation -- that is rcflex_tremor_record's job -- so that the raw signal
% stays inspectable and the conditioning stays testable on its own.
%
% Fields of R:
%   name       record file name, e.g. 'g2rof.rit'
%   subject    's6', 'g12', ... ; leading letter is the stimulation target
%   target     'GPi' | 'Vim' | 'STN'
%   group      'high' | 'low' amplitude tremor group (subjects 1-8 / 9-16)
%   hand       'left' | 'right' (from the .let / .rit extension)
%   condition  'rof' | 'r15of' | ... minutes since stimulation was stopped
%   minutes_off  0, 15, 30, 45 or 60
%   fs         sampling rate, Hz
%   v          fingertip velocity, m/s (column)
%   units_file the units the file itself stores, for audit
root=fullfile(project_path('root'),'data','external','tremordb');
meta=read_description(fullfile(root,'file_description.txt'));
% The raw records are re-fetchable and so are NOT committed (.gitignore excludes
% data/external/*/raw/). A single committed fixture stands in for them when they
% are absent, so that the test suite -- which must never depend on a download --
% still has real samples to check the conditioning against. This mirrors what
% grabmyo_p1s1_fixture.mat already does for the sEMG side. When the full set has
% been fetched the fixture is ignored entirely.
[names,fix]=list_records(root);
assert(~isempty(names),'SensoSpoon:Tremordb', ...
 ['No tremordb records at %s.\n' ...
  'Run data/external/tremordb/fetch_tremordb_subset.sh to download them, ' ...
  'or restore the committed fixture tremordb_fixture.mat.'],root);

if nargin<1||isempty(record)
 R=table();
 for i=1:numel(names)
  R=[R;struct2table(describe(names{i},meta),'AsArray',true)]; %#ok<AGROW>
 end
 R=sortrows(R,{'group','subject','minutes_off'});
 return
end

record=char(record);
assert(ismember(record,names),'SensoSpoon:Tremordb', ...
 'Record %s is not available here. Available: %s',record,strjoin(names,', '));
R=describe(record,meta);
f=fullfile(root,'raw',record);
if isfile(f)
 raw=readmatrix(f,'FileType','text');
else
 F=load(fix,'fixture');
 raw=F.fixture(strcmp({F.fixture.name},record)).v_raw;
end
raw=raw(:);raw=raw(~isnan(raw));
assert(~isempty(raw),'SensoSpoon:Tremordb','Record %s contained no samples.',record);
% The file stores physical velocity already; the VELOCITY column of
% file_description.txt says in which unit, and it differs between records.
switch R.units_file
 case 'mm/s';R.v=raw*1e-3;
 case 'm/s';R.v=raw;
 otherwise;error('SensoSpoon:Tremordb','Unknown velocity unit ''%s'' for %s.',R.units_file,record);
end
R.n=numel(R.v);
R.duration_s=R.n/R.fs;
end

% -------------------------------------------------------------------------
function [names,fix]=list_records(root)
% Records available to this installation: the downloaded set when present,
% otherwise whatever the committed fixture carries.
fix=fullfile(root,'tremordb_fixture.mat');
d=dir(fullfile(root,'raw','*.*'));d=d(~[d.isdir]);
if ~isempty(d)
 names=sort({d.name}');
 return
end
names={};
if isfile(fix)
 F=load(fix,'fixture');
 names=sort({F.fixture.name}');
end
end

% -------------------------------------------------------------------------
function r=describe(name,meta)
% Everything derivable from the file name, plus the row of file_description.txt.
r.name=string(name);
[~,stem,ext]=fileparts(name);
switch lower(ext)
 case '.let';r.hand="left";
 case '.rit';r.hand="right";
 otherwise;error('SensoSpoon:Tremordb','Unexpected extension ''%s'' on %s.',ext,name);
end
tok=regexp(stem,'^([a-z])(\d+)(r(?:\d+)?of)$','tokens','once');
assert(~isempty(tok),'SensoSpoon:Tremordb', ...
 'Record name %s does not match <target><n><condition>.',name);
r.subject=string([tok{1} tok{2}]);
switch tok{1}
 case 'g';r.target="GPi";
 case 'v';r.target="Vim";
 case 's';r.target="STN";
 otherwise;error('SensoSpoon:Tremordb','Unknown stimulation target ''%s''.',tok{1});
end
% Subjects 1-8 are the high-amplitude tremor group, 9-16 the low-amplitude one.
num=str2double(tok{2});
if num<=8;r.group="high";else;r.group="low";end
r.condition=string(tok{3});
m=regexp(tok{3},'^r(\d+)of$','tokens','once');
if isempty(m);r.minutes_off=0;else;r.minutes_off=str2double(m{1});end
key=lower(string(name));
assert(isKey(meta,key),'SensoSpoon:Tremordb', ...
 'Record %s is not listed in file_description.txt.',name);
row=meta(key);
r.fs=row.fs;r.range_V=row.range;r.units_file=row.units;
r.samples_expected=row.samples;
r.v=[];r.n=0;r.duration_s=0;
end

% -------------------------------------------------------------------------
function m=read_description(f)
% file_description.txt is the authority for units and sampling rate, and the
% units are NOT the same for every record, so this is parsed rather than
% assumed.
assert(isfile(f),'SensoSpoon:Tremordb','Missing %s',f);
m=containers.Map('KeyType','char','ValueType','any');
fid=fopen(f,'r');c=onCleanup(@()fclose(fid));
while true
 L=fgetl(fid);
 if ~ischar(L);break;end
 t=regexp(strtrim(L),'^\S+\s+(\S+\.(?:let|rit))\s+([\d.]+)\s+(\S+)\s+\S+\s+(\d+)\s+(\d+)$','tokens','once');
 if isempty(t);continue;end
 s.range=str2double(t{2});s.units=t{3};s.fs=str2double(t{4});s.samples=str2double(t{5});
 m(lower(t{1}))=s;
end
end
