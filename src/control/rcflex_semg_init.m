function C=rcflex_semg_init(C)
%RCFLEX_SEMG_INIT Fixed sEMG preprocessing coefficients, computed once.
% Called from rcflex_prepare_control.m, mirroring how C.intent_sos and C.ekf
% are prepared. Everything the causal chain needs -- coefficients, the packed
% state layout, and the initial state -- is built here so
% rcflex_semg_preprocess.m stays a pure function and the initial state has a
% single definition that call sites cannot drift apart from.
%
% Produces C.semg with fields:
%   nch, sub    channel count and sEMG samples per outer-loop tick
%   hp,notch,env,band   biquad coefficient rows [b0 b1 b2 a1 a2]
%   ix_*        index ranges within the packed state
%   s_len, s0   packed state length and initial value
%   n_feat      width of the feature vector returned per tick
assert(mod(C.semg_hz,C.outer_hz)==0,...
 'SensoSpoon:Semg','C.semg_hz (%g) must be an integer multiple of C.outer_hz (%g) so the two streams stay sample-synchronous.',...
 C.semg_hz,C.outer_hz);
nch=C.semg_channels;sub=C.semg_hz/C.outer_hz;

% Stage 1: 2-pole highpass at C.semg_hp_Hz. Removes the electrode half-cell
% offset and the bulk of the low-frequency motion artefact. Must come first --
% the offset is far larger than the EMG and would otherwise dominate every
% downstream stage including the rectifier.
hp=highpass2(C.semg_hp_Hz,C.semg_hz);
% Stage 2: notch at the mains frequency, which lies inside the EMG band and so
% survives any amount of band limiting.
notch=notch2(C.semg_mains_Hz,C.semg_notch_bw_Hz,C.semg_hz);
% Stages 3a/3b run on the RECTIFIED signal, which is where the activation
% envelope lives. Two envelopes are extracted because the two things we want to
% tell apart occupy different modulation bands:
%   env  : 2-pole lowpass at C.semg_env_Hz -- slow voluntary effort level.
%   band : 2-pole bandpass over C.semg_burst_band_Hz -- the rhythmic burst
%          modulation at the tremor frequency. This is the channel that should
%          carry tremor timing, and the one that leads the IMU by the
%          electromechanical delay.
env=lowpass2(C.semg_env_Hz,C.semg_hz);
band=bandpass2(C.semg_burst_band_Hz,C.semg_hz);

% Packed state: two direct-form-II transposed delays per biquad per channel.
nBq=4;
ixZ=1:(2*nBq*nch);
s_len=ixZ(end);
s0=zeros(s_len,1);

% --- Frequency-observable state (rcflex_semg_omega.m) -----------------------
% Kept SEPARATE from the preprocessing state above: the preprocessing chain is
% used by every consumer including the offline characterisation scripts, while
% this is only touched when C.semg_fusion is on. Separate packing means the
% fusion experiment cannot perturb the feature path it is being compared
% against.
fuseD=16;fuseTau=.15;
if isfield(C,'semg_fuse_D');fuseD=C.semg_fuse_D;end
if isfield(C,'semg_fuse_tau_s');fuseTau=C.semg_fuse_tau_s;end
nBuf=2*fuseD+1;
ixFuseBuf=1:(nBuf*nch);
ixFuseNum=ixFuseBuf(end)+1;ixFuseDen=ixFuseNum+1;ixFusePow=ixFuseDen+1;
s0Fuse=zeros(ixFusePow,1);

% --- Intent-gate state (rcflex_semg_intent_gate.m) --------------------------
% Packed separately from both the preprocessing chain and the omega-fusion
% accumulators, for the same reason those two are separate: the feature path is
% shared by every consumer including the offline characterisation, and a fusion
% experiment must not be able to perturb the path it is being compared against.
gateFc=1;gateAtt=.05;gateRel=1;gateFloorDeg=.75;gateExp=2;gateWarm=1;gateCal=[0;0];
gateFcFast=4;gateFloorFastDeg=2.2;
if isfield(C,'semg_gate_fc_Hz');gateFc=C.semg_gate_fc_Hz;end
if isfield(C,'semg_gate_fast_fc_Hz');gateFcFast=C.semg_gate_fast_fc_Hz;end
if isfield(C,'semg_gate_fast_floor_deg');gateFloorFastDeg=C.semg_gate_fast_floor_deg;end
if isfield(C,'semg_gate_attack_s');gateAtt=C.semg_gate_attack_s;end
if isfield(C,'semg_gate_release_s');gateRel=C.semg_gate_release_s;end
if isfield(C,'semg_gate_floor_deg');gateFloorDeg=C.semg_gate_floor_deg;end
if isfield(C,'semg_gate_exponent');gateExp=C.semg_gate_exponent;end
if isfield(C,'semg_gate_warmup_s');gateWarm=C.semg_gate_warmup_s;end
if isfield(C,'semg_gate_cal_rad_per_V');gateCal=C.semg_gate_cal_rad_per_V(:);end
ixGateZ=1:4;                       % slow path: 2 biquad states x 2 axes
ixGatePow=ixGateZ(end)+(1:2);      % slow path: one mean-square accumulator per axis
ixGateZf=ixGatePow(end)+(1:4);     % fast path: 2 biquad states x 2 axes
ixGatePowf=ixGateZf(end)+(1:2);    % fast path: one mean-square accumulator per axis
ixGateK=ixGatePowf(end)+1;         % tick counter, for the warm-up window
s0Gate=zeros(ixGateK,1);

C.semg=struct('nch',nch,'sub',sub,'hp',hp,'notch',notch,'env',env,'band',band,...
 'n_bq',nBq,'ix_z',ixZ,'s_len',s_len,'s0',s0,'n_feat',2*nch,...
 'fuse_D',fuseD,'fuse_alpha',1-exp(-1/(C.outer_hz*fuseTau)),...
 'ix_fuse_buf',ixFuseBuf,'ix_fuse_num',ixFuseNum,'ix_fuse_den',ixFuseDen,...
 'ix_fuse_pow',ixFusePow,'s_len_fuse',ixFusePow,'s0_fuse',s0Fuse,...
 'gate_lp',lowpass2(gateFc,C.outer_hz),...
 'gate_lp_fast',lowpass2(gateFcFast,C.outer_hz),...
 'gate_alpha_att',1-exp(-1/(C.outer_hz*gateAtt)),...
 'gate_alpha_rel',1-exp(-1/(C.outer_hz*gateRel)),...
 'gate_floor',deg2rad(gateFloorDeg)^2,'gate_floor_fast',deg2rad(gateFloorFastDeg)^2,...
 'gate_exponent',gateExp,'gate_cal',gateCal,...
 'gate_warmup_n',round(gateWarm*C.outer_hz),...
 'ix_gate_z',ixGateZ,'ix_gate_pow',ixGatePow,...
 'ix_gate_zf',ixGateZf,'ix_gate_powf',ixGatePowf,'ix_gate_k',ixGateK,...
 's_len_gate',ixGateK,'s0_gate',s0Gate);
end
function c=lowpass2(f,fs)
k=tan(pi*f/fs);den=1+sqrt(2)*k+k*k;
c=[k*k 2*k*k k*k]/den;c=[c 2*(k*k-1)/den (1-sqrt(2)*k+k*k)/den];
end
function c=highpass2(f,fs)
k=tan(pi*f/fs);den=1+sqrt(2)*k+k*k;
c=[1 -2 1]/den;c=[c 2*(k*k-1)/den (1-sqrt(2)*k+k*k)/den];
end
function c=notch2(f0,bw,fs)
k=tan(pi*f0/fs);d=bw/f0*k;den=1+d+k*k;
c=[(1+k*k) 2*(k*k-1) (1+k*k)]/den;c=[c 2*(k*k-1)/den (1-d+k*k)/den];
end
function c=bandpass2(band,fs)
% Constant-skirt 2-pole bandpass from the geometric-mean centre frequency and
% the fractional bandwidth the band implies.
f0=sqrt(band(1)*band(2));bw=band(2)-band(1);
k=tan(pi*f0/fs);d=bw/f0*k;den=1+d+k*k;
c=[d 0 -d]/den;c=[c 2*(k*k-1)/den (1-d+k*k)/den];
end
