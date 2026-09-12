function test_rcflex_intent_ekf()
% Unit tests for the 13-state intent/tremor EKF, run in isolation (no
% plant, no attitude_step) so a failure here is attributable to the
% estimator itself, not to the closed loop around it.
C=rcflex_control_parameters('ekf_predictive');C=rcflex_prepare_control(C);

% --- Frequency convergence, including the out-of-band 20 Hz case that a
% fixed 4-12 Hz notch/BMFLC basis cannot represent at all ---
dt=1/C.outer_hz;
for fHz=[4 8 12 20]
 s=[];t=(0:dt:3)';omegaHist=zeros(size(t));
 for k=1:numel(t)
  hand=deg2rad(3)*sin(2*pi*fHz*t(k))*[1;1];
  [s,~,info]=rcflex_intent_ekf_step(s,hand,C);omegaHist(k)=info(1);
 end
 estHz=median(omegaHist(t>=2));
 assert(abs(estHz-fHz)<0.2,sprintf('omega did not converge for %g Hz (got %.3f Hz)',fHz,estHz));
end

% --- Chirp tracking with bounded lag, 4 -> 12 Hz over 4 s ---
s=[];t=(0:dt:4)';f0=4;f1=12;kchirp=(f1-f0)/t(end);
trueFreq=zeros(size(t));estFreq=zeros(size(t));
for k=1:numel(t)
 f=f0+kchirp*t(k);trueFreq(k)=f;phase=2*pi*(f0*t(k)+kchirp*t(k)^2/2);
 hand=deg2rad(3)*sin(phase)*[1;1];
 [s,~,info]=rcflex_intent_ekf_step(s,hand,C);estFreq(k)=info(1);
end
lagErr=abs(estFreq(t>1)-trueFreq(t>1));
assert(max(lagErr)<1.5,sprintf('Chirp tracking lag too large (max %.2f Hz)',max(lagErr)));

% --- Pure tremor must not leak into the intent (voluntary) estimate ---
s=[];t=(0:dt:2)';intentErr=zeros(size(t));
for k=1:numel(t)
 hand=deg2rad(2)*sin(2*pi*6*t(k))*[1;1];
 [s,ref]=rcflex_intent_ekf_step(s,hand,C);intentErr(k)=norm(ref);
end
assert(rad2deg(max(intentErr(t>1)))<0.05,'Tremor leaked into the intent estimate');

% --- NIS spikes within 50 ms of a 5 deg step, recovers within 300 ms ---
s=[];t=(0:dt:1)';nisHist=zeros(size(t));stepAt=0.5;stepAngle=deg2rad(5);
for k=1:numel(t)
 hand=[0;0];if t(k)>=stepAt;hand=stepAngle*[1;1];end
 [s,~,info]=rcflex_intent_ekf_step(s,hand,C);nisHist(k)=info(2);
end
window=t>=stepAt&t<stepAt+.05;
assert(any(nisHist(window)>C.ekf_nis_hi),'NIS did not spike within 50 ms of a step');
recoverIx=find(t>=stepAt+.3,1);
assert(nisHist(recoverIx)<C.ekf_nis_hi,'NIS did not recover within 300 ms of a step');

% --- Covariance stays symmetric positive-definite over a long run
% (10000 steps @ 800 Hz = 12.5 s); deterministic multi-tone drive, no RNG ---
s=[];t=(0:dt:12.5)';
for k=1:numel(t)
 hand=deg2rad(3)*sin(2*pi*7*t(k))*[1;1]+1e-4*sin(2*pi*131*t(k))*[1;-1];
 s=rcflex_intent_ekf_step(s,hand,C);
end
n=13;P=reshape(s(n+1:n+n*n),n,n);
assert(norm(P-P','fro')<1e-9,'Covariance lost symmetry');
assert(min(eig(P))>0,'Covariance lost positive-definiteness');

% --- Determinism: pure function, no persistent state ---
s1=[];s2=[];hand=deg2rad(2)*[1;-1];
for k=1:50;s1=rcflex_intent_ekf_step(s1,hand,C);end
for k=1:50;s2=rcflex_intent_ekf_step(s2,hand,C);end
assert(isequal(s1,s2),'EKF step must be deterministic');

fprintf('EKF checks: frequency convergence (incl. 20 Hz), chirp tracking, intent leakage, NIS step response, covariance health and determinism passed.\n');
end
