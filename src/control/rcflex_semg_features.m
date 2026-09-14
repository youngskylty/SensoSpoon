function [F,tTick]=rcflex_semg_features(raw,C,t0)
%RCFLEX_SEMG_FEATURES Offline feature extraction over a whole raw sEMG stream.
% Runs exactly the same causal, per-tick chain the controller runs
% (rcflex_semg_preprocess), tick by tick, over a stream obtained from any
% source. Having one implementation matters: the point of the source
% indirection is that synthetic, recorded and physically captured data are
% comparable, and they are only comparable if they are processed by identical
% code. Analysis scripts and tests must call this rather than re-implementing
% the loop.
%
% raw   : n x C.semg_channels electrode-referred volts at C.semg_hz, as
%         returned by rcflex_semg_source.
% t0    : optional stream start time (s), default 0.
% F     : nTick x C.semg.n_feat features, [env(nch) burst(nch)] per row.
% tTick : nTick x 1 times at the outer-loop rate. Each row is the feature
%         evaluated at the LAST sEMG sample of its tick, so tTick(j) is the
%         instant the controller would have had that feature available -- no
%         lookahead, and directly comparable with IMU-rate signals.
if nargin<3;t0=0;end
assert(isfield(C,'semg'),'SensoSpoon:Semg','Call rcflex_prepare_control first (C.semg missing)');
sub=C.semg.sub;
nTick=floor(size(raw,1)/sub);
F=zeros(nTick,C.semg.n_feat);
s=[];
for j=1:nTick
 [s,f]=rcflex_semg_preprocess(s,raw((j-1)*sub+(1:sub),:),C);
 F(j,:)=f';
end
tTick=t0+(1:nTick)'*sub/C.semg_hz;
end
