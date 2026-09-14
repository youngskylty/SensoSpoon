function S=import_grabmyo(root,participant,session,C,channelMap)
%IMPORT_GRABMYO Convert a GRABMyo subset into the rcflex sEMG dataset format.
% GRABMyo (PhysioNet, CC BY 4.0) is the real-data test case for the six-stage
% validation gate. See docs/SEMG_DATASET.md for provenance and what it can and
% cannot answer.
%
% root        : folder holding raw/session{i}_participant{j}/...
% participant : integer participant id
% session     : integer session id (default 1)
% C           : control parameters (for C.semg_channels)
% channelMap  : 1x4 forearm channel indices (1..16) mapping GRABMyo F-channels
%               onto [roll agonist, roll antagonist, pitch agonist,
%               pitch antagonist]. Omit to derive them empirically (below).
%
% S : struct meeting rcflex_semg_dataset_spec, with labels built from the
%     gesture of each trial:
%       gesture 11 (wrist flexion)   -> 'intent'
%       gesture 12 (wrist extension) -> 'intent'
%       gesture 17 (rest)            -> 'rest'
%     No class is 'tremor': GRABMyo has no tremor subjects, so the gate's
%     tremor-dependent stages must and do report SKIPPED rather than scoring
%     voluntary contraction as if it were tremor.
%
% BIPOLAR DERIVATION. GRABMyo records 16 forearm electrodes MONOPOLAR against a
% common reference, while this pipeline's dataset contract specifies bipolar
% differential channels. The difference is not cosmetic: every monopolar channel
% carries the same reference potential, so any two of them are strongly
% correlated by construction, and feeding them in directly would report enormous
% "cross-talk" that is an artefact of the montage rather than a property of the
% muscles.
%
% The array is two rings of eight, 2 cm apart longitudinally, so F(k) and F(k+8)
% are a longitudinally adjacent pair at the same circumferential position with
% 20 mm spacing along the fibre direction -- exactly the bipolar montage
% docs/SEMG_SUBSYSTEM.md 1.9 specifies. Bipolar channel k is therefore
% F(k) - F(k+8), giving 8 bipolar sites around the circumference.
%
% CHANNEL SELECTION. Those 8 sites are circumferential positions, not named
% muscles, and GRABMyo publishes no site-to-muscle table. Rather than guess from
% the electrode-placement figure, the mapping is derived from the data: the site
% most selectively active during wrist FLEXION (relative to rest, and relative
% to extension) is taken as the flexor/agonist, and the most extension-selective
% site as the extensor/antagonist. The second axis takes the next-best pair.
%
% That second pair is a substitute, not a match: our pitch axis should be
% FCU/ECU, and this is simply another flexor/extensor pair on the same rings.
if nargin<3||isempty(session);session=1;end
if nargin<4||isempty(C);C=rcflex_control_parameters;C.semg_enabled=true;end
if nargin<5;channelMap=[];end

gestures=[11 12 17];classes={'intent','intent','rest'};
trials=1:3;

blocks={};labels=struct('t_start',{},'t_end',{},'class',{});
fs=[];tOff=0;
for g=1:numel(gestures)
 for t=trials
  name=sprintf('session%d_participant%d_gesture%d_trial%d',session,participant,gestures(g),t);
  f=fullfile(root,'raw',sprintf('session%d_participant%d',session,participant),name);
  if ~isfile([f '.hea']);continue;end
  [x,fsRec,labelsRec]=read_wfdb16([f '.hea']);
  if isempty(fs);fs=fsRec;end
  assert(fsRec==fs,'SensoSpoon:Grabmyo','Inconsistent sample rate in %s',name);
  mono=x(:,startsWith(labelsRec,'F'));
  assert(size(mono,2)==16,'SensoSpoon:Grabmyo','Expected 16 forearm electrodes, got %d',size(mono,2));
  forearm=mono(:,1:8)-mono(:,9:16);   % bipolar, 20 mm along the fibre direction
  blocks{end+1}=struct('data',forearm,'gesture',gestures(g),'class',classes{g}); %#ok<AGROW>
  dur=size(forearm,1)/fs;
  % Skip the first 0.5 s of every trial: the gesture is still being formed, and
  % a highpass transient sits at the join once the trials are concatenated.
  labels(end+1)=struct('t_start',tOff+0.5,'t_end',tOff+dur,'class',classes{g}); %#ok<AGROW>
  tOff=tOff+dur;
 end
end
assert(~isempty(blocks),'SensoSpoon:Grabmyo','No GRABMyo records found under %s',root);

allForearm=cell2mat(cellfun(@(b)b.data,blocks(:),'UniformOutput',false));
cls=arrayfun(@(k)string(blocks{k}.class),1:numel(blocks));
gest=cellfun(@(b)b.gesture,blocks);

if isempty(channelMap)
 channelMap=derive_channels(blocks,gest);
end

S=struct();
S.semg=allForearm(:,channelMap);
S.fs=fs;
S.labels=labels;
S.channels={sprintf('bip%d=F%d-F%d (flexion-dominant)',channelMap(1),channelMap(1),channelMap(1)+8),...
            sprintf('bip%d=F%d-F%d (extension-dominant)',channelMap(2),channelMap(2),channelMap(2)+8),...
            sprintf('bip%d=F%d-F%d (flexion, 2nd)',channelMap(3),channelMap(3),channelMap(3)+8),...
            sprintf('bip%d=F%d-F%d (extension, 2nd)',channelMap(4),channelMap(4),channelMap(4)+8)};
S.subject=sprintf('GRABMyo participant%d session%d',participant,session);
S.device='OT Bioelettronica EMG-USB2+, 2048 Hz, 2x8 monopolar forearm rings, 2 cm apart';
S.notes=sprintf(['GRABMyo (PhysioNet, CC BY 4.0). Healthy subject, no tremor, no IMU. ' ...
 'Bipolar derivations F(k)-F(k+8), 20 mm along the fibre direction. ' ...
 'Sites chosen empirically from the wrist flexion/extension contrast: bip %d %d %d %d. ' ...
 'Classes are intent/rest only.'],channelMap);
fprintf('Imported GRABMyo participant%d session%d: %d samples, %g Hz, %d trials, channels F%s\n',...
 participant,session,size(S.semg,1),fs,numel(blocks),mat2str(channelMap));
fprintf('  classes: %s\n',strjoin(unique(cls),', '));
end
% -------------------------------------------------------------------------
function map=derive_channels(blocks,gest)
%DERIVE_CHANNELS Pick flexor/extensor sites from the flexion/extension contrast.
p=@(sel)mean(cell2mat(cellfun(@(b)rms_cols(b.data),blocks(sel)','UniformOutput',false)),1);
pFlex=p(gest==11);pExt=p(gest==12);pRest=p(gest==17);
% Activation above rest, per channel, for each contraction.
aFlex=pFlex./max(pRest,eps);aExt=pExt./max(pRest,eps);
% Preference for flexion over extension, and vice versa.
prefFlex=aFlex./max(aExt,eps);prefExt=aExt./max(aFlex,eps);
[~,iF]=max(prefFlex);[~,iE]=max(prefExt);
prefFlex(iF)=-inf;prefExt(iE)=-inf;
[~,iF2]=max(prefFlex);[~,iE2]=max(prefExt);
map=[iF iE iF2 iE2];
end
function r=rms_cols(x);r=sqrt(mean(x.^2,1));end
% -------------------------------------------------------------------------
function [x,fs,labels]=read_wfdb16(heaFile)
%READ_WFDB16 Minimal reader for WFDB format-16 records.
% Implemented directly rather than via the WFDB toolbox so the import has no
% extra dependency. Format 16 is little-endian int16, channel-interleaved.
fid=fopen(heaFile,'r');assert(fid>0,'Cannot open %s',heaFile);
raw=textscan(fid,'%s','Delimiter','\n','CommentStyle','#');fclose(fid);
lines=raw{1};lines=lines(~cellfun(@isempty,lines));
hdr=strsplit(strtrim(lines{1}));
nSig=str2double(hdr{2});fs=str2double(hdr{3});nSamp=str2double(hdr{4});

gain=zeros(1,nSig);baseline=zeros(1,nSig);labels=strings(1,nSig);datFile='';
for k=1:nSig
 f=strsplit(strtrim(lines{k+1}));
 if isempty(datFile);datFile=f{1};end
 % Field 3 is  gain(baseline)/units, with baseline and units both optional.
 % Parsed in pieces rather than with one optional-group regexp: MATLAB's
 % regexp(...,'once') silently DROPS trailing unmatched optional groups, so a
 % single pattern returns a short token list and indexing it errors.
 gain(k)=str2double(regexp(f{3},'^[-\d.eE+]+','match','once'));
 bt=regexp(f{3},'\(([-\d]+)\)','tokens','once');
 if ~isempty(bt);baseline(k)=str2double(bt{1});
 else;baseline(k)=str2double(f{5});end   % WFDB: absent baseline = ADC zero
 if gain(k)==0;gain(k)=200;end           % WFDB: 0 means default gain
 labels(k)=string(f{end});
end

fid=fopen(fullfile(fileparts(heaFile),datFile),'r');
assert(fid>0,'Cannot open %s',datFile);
d=fread(fid,[nSig nSamp],'int16=>double',0,'ieee-le');fclose(fid);
% Header gain is in ADU per mV; convert to volts so the dataset spec's
% electrode-referred-volts contract is satisfied.
x=((d.'-baseline)./gain)/1000;
end
