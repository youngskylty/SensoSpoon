function T=run_semg_fusion_experiment(writeOutputs)
%RUN_SEMG_FUSION_EXPERIMENT E1: does an sEMG omega observable repair the EKF?
% Tests the hypothesis CONTROL_IMPROVEMENT_PLAN.md Sec 1a arrived at and could
% not act on: that the joint intent/tremor fit is weakly identifiable while
% intent and tremor ramp together, and that the remedy is information about
% omega from outside that fit.
%
% Three arms, selected by the single switch C.semg_fusion:
%   'off'     IMU-only. The control group.
%   'semg'    burst-band frequency observable -> scalar update on state 13.
%   'oracle'  TRUE tremor frequency -> the same input. Not realisable; it
%             bounds the best achievable gain. If the oracle does not help
%             either, omega was never the binding constraint and the fusion
%             premise is wrong rather than the observable being poor.
%
% Cases are the two the baseline actually fails on: case 9 'combined'
% (51.5 % suppression -- intent and tremor ramping together, the exact Sec 1a
% failure) and case 14 'step' (-116.6 %).
%
% Scored against simulation ground truth, which exists here and does not on any
% real recording. NOTE the standing caveat: the synthetic sEMG is generated
% FROM the same tremor truth that drives the plant, so the burst band carries
% the true omega by construction. This experiment can therefore only establish
% a conditional -- IF real sEMG delivers an observable of this quality, THEN
% the estimator improves by this much. E2 is what pins down how good it must be.
if nargin<1;writeOutputs=true;end

cases={'combined',8,1,'case 9  (intent+tremor ramp together)';
       'step',    8,1,'case 14 (5 deg intent step)'};
arms={'off','semg','oracle'};
rows={};

for ci=1:size(cases,1)
 nm=cases{ci,1};fHz=cases{ci,2};amp=cases{ci,3};label=cases{ci,4};
 fprintf('\n=== %s : %s ===\n',nm,label);
 for ai=1:numel(arms)
  C=rcflex_control_parameters('ekf_predictive');
  C.semg_enabled=true;              % needed to generate the stream at all
  C.semg_fusion=arms{ai};           % THE switch
  C.semg_fusion_oracle_Hz=fHz;      % read only by the 'oracle' arm
  C=rcflex_prepare_control(C);
  D=rcflex_control_case(nm,4,fHz,amp,1701,C);
  S=run_rcflex_control_fast(D,C);
  M=rcflex_control_metrics(S);
  m=metrics_of(S,fHz);
  rows(end+1,:)={nm,arms{ai},m.omega_rms_Hz,m.lock_s,m.intent_rms_deg,...
   M.suppression_percent,M.attitude_rms_deg,m.conf_mean}; %#ok<AGROW>
  fprintf(['  %-7s omega RMS %6.3f Hz | lock %6s | intent RMS %.4f deg | ' ...
           'suppression %8.2f %% | conf %.2f\n'],...
   arms{ai},m.omega_rms_Hz,lockstr(m.lock_s),m.intent_rms_deg,M.suppression_percent,m.conf_mean);
 end
end

T=cell2table(rows,'VariableNames',{'case','arm','omega_rms_Hz','lock_s',...
 'intent_rms_deg','suppression_percent','attitude_rms_deg','semg_conf'});
fprintf('\n');disp(T);

if writeOutputs
 out=project_path('results','semg_fusion_e1.csv');
 writetable(T,out);fprintf('Wrote %s\n',out);
end
end
% -------------------------------------------------------------------------
function m=metrics_of(S,fTrue)
%METRICS_OF Score one run against the scenario's ground truth.
t=S.time;L=S.control;
% Diagnostic layout: q(1:2) ref(3:4) e(5:6) iref(7:8) limit(9:10) power(11:14)
% sat(15) cond(16) ekfDiag(17:24) semgFeat(25:32).
% ekfDiag = [omega_Hz nis gammaTrust gammaAuth iPred semgOmegaHz semgConf 0].
omega=L(:,17);semgConf=L(:,23);
ref=L(:,3:4);                       % [pitch; roll] -- the operative estimate
% Scored after the opening amplitude ramp, which every scenario carries and
% which is not what is under test.
w=t>1.5;
m.omega_rms_Hz=sqrt(mean((omega(w)-fTrue).^2));
m.conf_mean=mean(semgConf(w));
% Intent error uses the reference the controller ACTUALLY applied, not a raw
% filter state: that is the quantity the reference path consumes, so it is the
% one that decides whether the sensor was worth adding.
truth=[S.intent(:,2) S.intent(:,1)];   % [pitch roll] to match ref
m.intent_rms_deg=rad2deg(sqrt(mean((ref(w,:)-truth(w,:)).^2,'all')));
% Time to lock: first instant after which |omega-true| stays under 0.5 Hz.
bad=find(abs(omega-fTrue)>=0.5);
if isempty(bad);m.lock_s=t(1);
elseif bad(end)==numel(t);m.lock_s=NaN;      % never locked
else;m.lock_s=t(bad(end)+1);end
end
function s=lockstr(v)
if isnan(v);s='never';else;s=sprintf('%.2fs',v);end
end
