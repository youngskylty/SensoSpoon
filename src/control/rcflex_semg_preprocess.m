function [s,feat]=rcflex_semg_preprocess(s,block,C)
%RCFLEX_SEMG_PREPROCESS Causal sEMG preprocessing for one outer-loop tick.
% Pure function, no persistent variables -- the same determinism contract
% rcflex_intent_ekf_step.m works under, and for the same reason
% (tests/test_rcflex_control.m asserts it).
%
% s     : packed filter state, C.semg.s_len elements. Pass [] to initialise
%         from C.semg.s0.
% block : C.semg.sub x C.semg.nch electrode-referred samples covering exactly
%         this control tick. The sub-sample count is fixed by construction
%         (C.semg_hz is an integer multiple of C.outer_hz), so the sEMG and IMU
%         streams stay aligned without any resampling or timestamp matching.
% feat  : [env(nch); burst(nch)] at the outer-loop rate, evaluated at the LAST
%         sEMG sample of the tick so it is causal and carries no lookahead.
%           env   -- slow activation envelope, i.e. voluntary effort level.
%           burst -- tremor-band modulation of the envelope, i.e. rhythmic
%                    bursting. Signed, and leads the IMU by the
%                    electromechanical delay.
%
% Chain, per sEMG sample: highpass -> mains notch -> full-wave rectify ->
% {envelope lowpass, burst bandpass}. Rectification is what moves the
% modulation down to baseband; it is the standard linear-envelope detector
% (De Luca 1997) and is deliberately kept simple here -- this stage exists to
% make the signal available and characterisable, not to be the final design.
E=C.semg;nch=E.nch;sub=E.sub;
if isempty(s);s=E.s0;end
Z=reshape(s(E.ix_z),2,E.n_bq,nch);

envOut=zeros(1,nch);burstOut=zeros(1,nch);
for k=1:sub
 for ch=1:nch
  u=block(k,ch);
  [u,Z(:,1,ch)]=biquad(E.hp,u,Z(:,1,ch));
  [u,Z(:,2,ch)]=biquad(E.notch,u,Z(:,2,ch));
  r=abs(u);
  [envOut(ch),Z(:,3,ch)]=biquad(E.env,r,Z(:,3,ch));
  [burstOut(ch),Z(:,4,ch)]=biquad(E.band,r,Z(:,4,ch));
 end
end

s(E.ix_z)=Z(:);
feat=[envOut(:);burstOut(:)];
end
function [y,z]=biquad(c,u,z)
% Direct form II transposed: one multiply-accumulate per state, and the state
% is the filter's entire memory, which is what lets the whole chain be packed
% into a plain vector.
y=c(1)*u+z(1);
z(1)=c(2)*u-c(4)*y+z(2);
z(2)=c(3)*u-c(5)*y;
end
