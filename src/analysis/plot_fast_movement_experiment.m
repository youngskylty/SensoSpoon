function plot_fast_movement_experiment(T,CL)
%PLOT_FAST_MOVEMENT_EXPERIMENT The figure the E3 decision is read from.
% Left: the trade the whole experiment is about -- detection latency against
% class separation, with both declared criteria drawn as lines. A candidate has
% to sit in the corner where both are satisfied; the point of plotting it this
% way is that it shows immediately whether that corner is EMPTY. Right: what
% each admissible arm costs in the closed loop on the fast reach, against the
% IMU-only estimator that the gate has to beat.
%
% Both arguments are optional; with none, the figure is rebuilt from the saved
% CSVs so START_PROJECT('figures') does not have to re-run the experiment.
if nargin<1||isempty(T)
 f=project_path('results','fast_movement_detectors.csv');
 assert(isfile(f),'SensoSpoon:FastMove',...
  'No saved result at %s. Run START_PROJECT(''fastmove'') first.',f);
 T=readtable(f,'TextType','string');
end
if nargin<2||isempty(CL)
 f=project_path('results','fast_movement_closed_loop.csv');
 if isfile(f);CL=readtable(f,'TextType','string');else;CL=table();end
end
LAT=.350;SEP_CLOSED=.10;

f=figure('Color','w','Visible','off','Position',[100 100 1480 560]);
if isprop(f,'Theme');f.Theme='light';end
tiledlayout(1,2);

nexttile;hold on;
kinds=["amplitude","rate","hybrid"];
mk={'o','s','^'};
for k=1:numel(kinds)
 ix=T.detector==kinds(k);
 if ~any(ix);continue;end
 scatter(1000*T.latency_mean_s(ix),T.tremor_conf_max(ix),90,mk{k},'filled','MarkerFaceAlpha',.75);
end
for k=1:height(T)
 if T.detector(k)~="amplitude"
  text(1000*T.latency_mean_s(k),T.tremor_conf_max(k),sprintf('  %g Hz',T.rate_fc_Hz(k)),'FontSize',8);
 end
end
xline(1000*LAT,'k--','LineWidth',1.5,'Label','350 ms target','LabelVerticalAlignment','bottom');
yline(SEP_CLOSED,'r--','LineWidth',1.5,'Label','separation limit (conf \leq 0.10 on tremor)',...
 'LabelHorizontalAlignment','left');
grid on;xlabel('detection latency on a fast reach (ms)');
ylabel('gate confidence on a TREMOR-ONLY record (lower is better)');
legend(cellstr(kinds),'Location','northeast');
title({'The trade E3 exists to test','admissible region: left of the dashed line AND below the red line'});
hold off;

nexttile;
if isempty(CL)
 axis off;text(.5,.5,'no closed-loop result saved','HorizontalAlignment','center');
else
 r=CL(CL.scenario=="reach",:);
 g=groupsummary(r,'arm',{'mean','max'},'attitude_rms_deg');
 [~,ord]=sort(g.mean_attitude_rms_deg);
 g=g(ord,:);
 b=bar(1:height(g),g.mean_attitude_rms_deg);
 b.FaceAlpha=.8;
 hold on;
 errorbar(1:height(g),g.mean_attitude_rms_deg,...
  zeros(height(g),1),g.max_attitude_rms_deg-g.mean_attitude_rms_deg,'k.','LineWidth',1.1);
 set(gca,'XTick',1:height(g),'XTickLabel',cellstr(g.arm),'XTickLabelRotation',20);
 grid on;ylabel('attitude error on ''reach'' (deg RMS)');
 title({'Closed-loop cost of a fast voluntary movement','bar = mean over seeds, whisker = worst seed'});
 hold off;
end

exportgraphics(f,project_path('figures','fast_movement_detectors.png'),'Resolution',160);
close(f);
fprintf('Wrote %s\n',project_path('figures','fast_movement_detectors.png'));
end
