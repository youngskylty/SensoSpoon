function T=characterize_rcflex_semg(scenarios,writeOutputs)
%CHARACTERIZE_RCFLEX_SEMG What does sEMG actually tell us about intent vs tremor?
% This is the evidence-gathering stage that sits between "sEMG is sensed" and
% "sEMG influences the estimator". It deliberately answers the question with
% measurements rather than assumption, because the next design decision -- what
% the estimator should take from sEMG -- is supposed to follow from these
% numbers and not precede them.
%
% For each scenario it reports, per axis:
%
%   env_intent_r   Correlation of the slow envelope feature with the voluntary
%                  intent magnitude. High => sEMG amplitude is a usable proxy
%                  for voluntary effort.
%   env_tremor_r   Correlation of the same envelope with tremor magnitude. If
%                  this is also high, the envelope does NOT discriminate and
%                  amplitude alone is not a fusion signal.
%   burst_tremor_r Correlation of the burst-band feature with tremor
%                  acceleration -- the discriminating feature if there is one.
%   burst_power    Mean-square burst feature. Compare a tremor scenario against
%                  the tremor-free 'intent' scenario: the ratio is the
%                  detection margin.
%   lead_ms        Cross-correlation lead of the burst feature over the
%                  mechanical tremor. This is the quantity that decides whether
%                  sEMG can contribute PHASE information the IMU does not
%                  already have, which is the only reason to prefer fusion over
%                  simply improving the IMU-only estimator.
%   burst_freq_Hz  Dominant frequency of the burst feature, against the
%                  scenario's true tremor frequency. Tests whether sEMG offers
%                  an independent frequency observable.
%   snr_db         Burst-feature power in the tremor scenario over the same
%                  feature in the tremor-free scenario, in dB.
%
% Ground truth is available here (rcflex_control_case.m keeps intent and
% tremor separate) and is used ONLY for scoring -- never fed to any estimator.
if nargin<1||isempty(scenarios);scenarios={'sine','combined','intent','mixed','chirp'};end
if nargin<2;writeOutputs=true;end

C=rcflex_control_parameters;C.semg_enabled=true;C=rcflex_prepare_control(C);
rows=cell(0,1);
baseline=[];

for k=1:numel(scenarios)
 name=scenarios{k};
 D=rcflex_control_case(name,4,8,1,9100+k,C);
 F=features(D,C);dt=1/C.outer_hz;
 tTick=(0:size(F,1)-1)'*dt+D.time(1);
 ix=tTick>1.5;   % skip the amplitude ramp every scenario opens with

 for axis=1:2
  ag=2*axis-1;                       % agonist channel of this axis
  env=F(:,ag);burst=F(:,C.semg.nch+ag);
  intentMag=abs(interp1(D.time,D.intent(:,axis),tTick,'linear',0));
  tremorMag=abs(interp1(D.time,D.tremor(:,axis),tTick,'linear',0));
  tremorAcc=interp1(D.time,gradient(gradient(D.tremor(:,axis),D.time),D.time),tTick,'linear',0);

  r=struct();
  r.scenario=string(name);r.axis=axis;
  r.env_intent_r=safecorr(env(ix),intentMag(ix));
  r.env_tremor_r=safecorr(env(ix),tremorMag(ix));
  r.burst_tremor_r=safecorr(burst(ix),tremorAcc(ix));
  r.burst_power=mean(burst(ix).^2);
  r.lead_ms=1000*lead_of(burst(ix),tremorAcc(ix),dt,D.frequency);
  r.burst_freq_Hz=dominant_freq(burst(ix),C.outer_hz);
  r.true_freq_Hz=D.frequency;
  rows{end+1,1}=r; %#ok<AGROW>
 end

 if strcmp(name,'intent');baseline=mean(cellfun(@(x)x.burst_power,rows(end-1:end)));end
end

T=struct2table([rows{:}]);
% SNR against the tremor-free scenario. Computed after the loop because the
% tremor-free case is what defines the floor, and it may not be evaluated first.
if isempty(baseline)
 Dint=rcflex_control_case('intent',4,8,1,9199,C);Fint=features(Dint,C);
 ixi=(0:size(Fint,1)-1)'/C.outer_hz>1.5;
 baseline=mean(mean(Fint(ixi,C.semg.nch+(1:2)).^2));
end
T.snr_db=10*log10(T.burst_power/max(baseline,eps));

if writeOutputs
 out=project_path('results','semg_characterization.csv');
 writetable(T,out);
 fprintf('Wrote %s\n',out);
 figure('Name','sEMG characterization','Color','w','Position',[80 80 1100 720]);
 D=rcflex_control_case('combined',4,8,1,9102,C);F=features(D,C);
 tTick=(0:size(F,1)-1)'/C.outer_hz;
 subplot(3,1,1);
 plot(D.time,rad2deg(D.intent(:,1)),'LineWidth',1.2);hold on;
 plot(D.time,rad2deg(D.tremor(:,1)),'LineWidth',.7);
 ylabel('deg');legend('intent truth','tremor truth','Location','best');
 title('combined scenario: mechanical ground truth (roll)');grid on;
 subplot(3,1,2);
 plot(tTick,F(:,1),tTick,F(:,2),'LineWidth',1);
 ylabel('V');legend('roll agonist envelope','roll antagonist envelope','Location','best');
 title('sEMG slow envelope -- voluntary effort proxy');grid on;
 subplot(3,1,3);
 plot(tTick,F(:,C.semg.nch+1),'LineWidth',1);
 xlabel('time (s)');ylabel('V');
 title('sEMG burst-band feature -- rhythmic tremor drive');grid on;
 fig=project_path('figures','semg_characterization.png');
 exportgraphics(gcf,fig,'Resolution',150);
 fprintf('Wrote %s\n',fig);
end

fprintf('\nsEMG characterization (4 s scenarios, 8 Hz tremor where present):\n');
disp(T);
end
function F=features(D,C)
% Shared pipeline, so synthetic, recorded and captured streams are processed by
% identical code and their characterisations are comparable.
F=rcflex_semg_features(D.semg,C,D.time(1));
end
function r=safecorr(a,b)
% Zero rather than NaN when one side is constant, which the tremor-free
% scenario produces by construction.
if std(a)<eps||std(b)<eps;r=0;return;end
r=corr(a(:),b(:));
end
function L=lead_of(a,m,dt,fHz)
% Positive => a leads m. Window capped at half a tremor period so a lead and a
% lag of opposite sign cannot be confused (see tests/test_rcflex_semg.m).
if std(a)<eps||std(m)<eps;L=NaN;return;end
maxLag=max(1,floor(.5/max(fHz,1)/dt));
[c,lags]=xcorr(detrend(a),detrend(m),maxLag,'normalized');
[~,j]=max(c);L=-lags(j)*dt;
end
function f=dominant_freq(x,fs)
if std(x)<eps;f=NaN;return;end
x=detrend(x(:)).*hann_local(numel(x));
n=2^nextpow2(numel(x));P=abs(fft(x,n)).^2;P=P(1:n/2+1);
fAx=(0:n/2)'*fs/n;P(fAx<1)=0;
[~,j]=max(P);f=fAx(j);
end
function w=hann_local(n)
% Local so the script does not depend on Signal Processing Toolbox.
w=.5*(1-cos(2*pi*(0:n-1)'/(n-1)));
end
