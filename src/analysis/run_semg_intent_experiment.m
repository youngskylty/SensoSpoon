function T=run_semg_intent_experiment(writeOutputs)
%RUN_SEMG_INTENT_EXPERIMENT E2: does sEMG repair the intent/tremor separation?
%
% The experiment the shipped sEMG+IMU system rests on. E1 (sEMG -> omega) was
% run first and returned a negative result; its finding was that omega was
% never the binding constraint. E2 tests the constraint the omega-capture
% experiment actually identified: that the intent states have the freedom to
% absorb tremor content the {omega, 2*omega} basis cannot represent, and that
% no setting of the intent process noise fixes it, because lowering it merely
% reverses the failure and lets the tremor states absorb voluntary movement.
%
% THREE ARMS, differing only in C.semg_fusion:
%   'off'                IMU-only at nominal q_intent. The control group.
%   'intent_gate_fixed'  IMU-ONLY ABLATION. Identical Q-scaling mechanism held
%                        at a constant confidence, with no sEMG stream at all.
%                        This is the classic "just lower q_intent" fix and it
%                        is the arm that makes the sEMG claim falsifiable: any
%                        gain the sEMG arm shows over THIS one is attributable
%                        to the sensor rather than to the mechanism.
%   'intent_gate'        the sEMG+IMU system. Same mechanism, driven by the
%                        measured voluntary muscle drive.
%
% Scored against simulation ground truth, which exists here and does not on any
% real recording. STANDING CAVEAT: the synthetic sEMG is generated from the
% same intent truth that drives the plant, so its envelope carries the
% voluntary drive by construction. This experiment therefore establishes an
% ALGORITHMIC result and a conditional -- IF a real recording delivers a
% voluntary-drive observable that separates the two classes as well as this one
% does (measured 6.3:1; see calibrate_rcflex_semg_intent), THEN the estimator
% improves by this much. It is NOT evidence about physiology, and real-data
% feasibility is a separate stage with its own gate (docs/SEMG_DATASET.md).
if nargin<1;writeOutputs=true;end

% 'reach' is the scenario the 20-case benchmark does not contain: a FAST
% voluntary movement on top of tremor. It is the case that can distinguish a
% gate that preserves voluntary motion from one that merely happens not to be
% tested on any.
cases={'mixed',     4,8,2,'case 6  off-harmonic tremor, no voluntary motion'
       'irregular', 4,8,2,'case 8  off-harmonic tremor, no voluntary motion'
       'sine',      4,12,1,'case 3  in-band tremor'
       'combined',  4,8,1,'case 9  intent and tremor ramp together'
       'intent',    8,8,0,'case 10 voluntary motion only, no tremor'
       'reach',     4,8,1,'reach   FAST voluntary movement + tremor'};
arms={'off','ekf_cv (IMU-only)',false
      'intent_gate_fixed','ekf_cv_qfix (IMU-only ablation)',false
      'intent_gate','ekf_cv_semg (sEMG+IMU)',true};
rows={};

for ci=1:size(cases,1)
 nm=cases{ci,1};dur=cases{ci,2};fHz=cases{ci,3};amp=cases{ci,4};label=cases{ci,5};
 fprintf('\n=== %-10s %s ===\n',nm,label);
 for ai=1:size(arms,1)
  C=rcflex_control_parameters('ekf_predictive');
  C.ekf_intent_model='cv';
  C.semg_fusion=arms{ai,1};
  C.semg_enabled=arms{ai,3};
  C=rcflex_prepare_control(C);
  C.seed=9200+ci;                       % seeds unseen by the benchmark
  D=rcflex_control_case(nm,dur,fHz,amp,C.seed,C);
  S=run_rcflex_control_fast(D,C,rcflex_parameters,.0005);
  M=rcflex_control_metrics(S);
  Q=rcflex_intent_split_metrics(S);
  rows(end+1,:)={nm,arms{ai,1},M.suppression_percent,M.attitude_rms_deg,...
   Q.tremor_leak_deg,Q.intent_track_deg,Q.intent_ref_rms_deg,...
   Q.gate_conf_roll,Q.gate_conf_pitch,M.current_peak_A,M.power_mean_W}; %#ok<AGROW>
  if isnan(M.suppression_percent);sup='      n/a';else;sup=sprintf('%8.2f%%',M.suppression_percent);end
  fprintf('  %-18s suppression %s | attitude %.4f deg | tremor leak %.4f deg | intent track %.4f deg | conf %.2f/%.2f\n',...
   arms{ai,1},sup,M.attitude_rms_deg,Q.tremor_leak_deg,Q.intent_track_deg,Q.gate_conf_roll,Q.gate_conf_pitch);
 end
end

T=cell2table(rows,'VariableNames',{'scenario','arm','suppression_percent',...
 'attitude_rms_deg','tremor_leak_deg','intent_track_deg','intent_ref_rms_deg',...
 'gate_conf_roll','gate_conf_pitch','current_peak_A','power_mean_W'});
fprintf('\n');disp(T);

if writeOutputs
 out=project_path('results','semg_intent_e2.csv');
 writetable(T,out);fprintf('Wrote %s\n',out);
 plot_semg_intent_experiment(T);
end
end
% -------------------------------------------------------------------------
function plot_semg_intent_experiment(T)
%PLOT_SEMG_INTENT_EXPERIMENT The single figure the E2 claim is read from.
% Left: tremor absorbed into the intent estimate (lower is better). Right:
% voluntary motion the estimator failed to follow (lower is better). Plotting
% them side by side is the point -- the IMU-only ablation wins the left panel
% and loses the right one, which is exactly the trade sEMG is there to break.
arms={'off','intent_gate_fixed','intent_gate'};
labels={'ekf\_cv (IMU-only)','ekf\_cv\_qfix (IMU-only ablation)','ekf\_cv\_semg (sEMG+IMU)'};
scen=unique(T.scenario,'stable');
L=zeros(numel(scen),3);K=L;
for i=1:numel(scen)
 for j=1:3
  ix=strcmp(T.scenario,scen{i})&strcmp(T.arm,arms{j});
  L(i,j)=T.tremor_leak_deg(ix);K(i,j)=T.intent_track_deg(ix);
 end
end
f=figure('Color','w','Visible','off','Position',[100 100 1500 560]);
if isprop(f,'Theme');f.Theme='light';end
tiledlayout(1,2);
nexttile;bar(L);set(gca,'XTickLabel',scen);ylabel('Tremor absorbed into intent (deg RMS)');
title({'Tremor leaking into the voluntary estimate','lower is better'});
legend(labels,'Location','northwest');grid on;
nexttile;bar(K);set(gca,'XTickLabel',scen);ylabel('Voluntary motion not followed (deg RMS)');
title({'Voluntary motion the estimator suppressed','lower is better'});
legend(labels,'Location','northwest');grid on;
exportgraphics(f,project_path('figures','semg_intent_separation.png'),'Resolution',160);
close(f);
fprintf('Wrote %s\n',project_path('figures','semg_intent_separation.png'));
end
