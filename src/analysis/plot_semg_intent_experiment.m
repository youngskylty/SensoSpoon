function plot_semg_intent_experiment(T)
%PLOT_SEMG_INTENT_EXPERIMENT The single figure the E2 claim is read from.
% Left: tremor absorbed into the intent estimate (lower is better). Right:
% voluntary motion the estimator failed to follow (lower is better). Plotting
% them side by side is the point -- the IMU-only ablation wins the left panel
% and loses the right one, which is exactly the trade sEMG is there to break.
%
% T is optional. With no argument the figure is rebuilt from the saved
% output/results/semg_intent_e2.csv, so START_PROJECT('figures') can regenerate
% it without re-running the hour of simulation that produced it. Split out of
% run_semg_intent_experiment.m for exactly that reason; there is one plotting
% implementation and both callers use it.
if nargin<1||isempty(T)
 f=project_path('results','semg_intent_e2.csv');
 assert(isfile(f),'SensoSpoon:SemgIntent',...
  'No saved E2 result at %s. Run START_PROJECT(''fusion'') first.',f);
 T=readtable(f,'TextType','char');
end
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
