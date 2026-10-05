function plot_report_benchmark()
%PLOT_REPORT_BENCHMARK IEEE report figure 1: tremor suppression, feasible cases.
%
% Rebuilt from output/results/control_benchmark.csv, so the figure can never be
% regenerated from a different run than the table beside it.
%
% Three deliberate differences from the diagnostic figure of the same data
% (plot_rcflex_benchmark, which is kept for debugging):
%
%  1. The three hardware-INFEASIBLE cases (5, 14, 20) are not plotted. The
%     project's own feasibility bound says their suppression numbers measure
%     the actuator rather than the algorithm, and their -200 % values set the
%     axis scale for everything else. Case 10 carries no tremor at all, so
%     suppression is undefined there and it is excluded too.
%  2. Cases are re-ordered into the four comparisons the benchmark actually
%     makes -- frequency, waveform, voluntary motion, plant robustness -- rather
%     than left in the order they were written. Five of the sixteen share
%     "sine, 8 Hz, 1 deg" and differ only in payload, bus voltage or actuator
%     wear, so a label built from the tremor alone would be ambiguous.
%  3. No prose inside the axes. The caption carries the caveats.
S=rcflex_figure_style('double');
T=readtable(project_path('results','control_benchmark.csv'),'TextType','string');

arms=["simple_pid","ekf_cv","ekf_cv_semg"];
label={'Baseline PID','EKF, IMU only','EKF + sEMG'};

% Benchmark case ids, grouped. Ids are kept explicit so a row of this figure can
% be traced straight back to a row of control_benchmark.csv.
g(1).name='Tremor frequency';         g(1).id=[1 16 17 2 18 19 3];
g(1).tick={'4 Hz','5 Hz','7 Hz','8 Hz','9 Hz','11 Hz','12 Hz'};
g(2).name='Waveform / amplitude';     g(2).id=[4 6 7 8];
g(2).tick={'6 Hz, 3\circ','mixed','chirp','irregular'};
g(3).name='Voluntary';                g(3).id=9;
g(3).tick={'+ tremor'};
g(4).name='Plant robustness';         g(4).id=[11 12 13 15];
g(4).tick={'0 g','15 g','5.8 V','worn'};

id=[g.id];tick=[g.tick];
% Fail loudly if the benchmark ever changes shape underneath this figure, rather
% than quietly plotting a stale subset.
ref=T(T.arm==arms(1),:);
expect=sort(ref.case_id(ref.hardware_feasible==1&ref.amplitude_deg>0))';
assert(isequal(sort(id),expect),'SensoSpoon:ReportFigure', ...
 'Feasible scored cases in the CSV are [%s] but this figure groups [%s]. Update the grouping.', ...
 num2str(expect),num2str(sort(id)));

n=numel(id);na=numel(arms);
supp=nan(n,na);col=zeros(na,3);
for a=1:na
 col(a,:)=S.arm_rgb(S.arm_key==arms(a),:);
 for k=1:n
  q=T(T.arm==arms(a)&T.case_id==id(k),:);
  assert(height(q)==1,'SensoSpoon:ReportFigure','Expected one row for %s case %d.',arms(a),id(k));
  supp(k,a)=q.suppression_percent;
 end
end

f=figure('Color','w','Visible','off','Units','inches','Position',[1 1 S.width_in 2.45]);
if isprop(f,'Theme');f.Theme='light';end
ax=axes(f,'Units','normalized','Position',[.068 .345 .868 .545]);hold(ax,'on');
b=bar(ax,1:n,supp,'grouped','EdgeColor','none','BarWidth',.86);
for a=1:na;b(a).FaceColor=col(a,:);end
h90=yline(ax,90,'--','Color',S.ref,'LineWidth',.7);
rcflex_axis_style(ax,S);
set(ax,'XLim',[.4 n+.6],'YLim',[50 100],'YTick',50:10:100, ...
  'XTick',1:n,'XTickLabel',tick,'XTickLabelRotation',45);
ylabel(ax,'Tremor suppression (%)','FontSize',S.font_label,'Color',S.ink);

% Group separators, and group names on a second row beneath the tick labels.
e=cumsum(arrayfun(@(k)numel(g(k).id),1:numel(g)));s0=[1 e(1:end-1)+1];
for k=1:numel(g)-1
 xline(ax,e(k)+.5,'-','Color',S.grid,'LineWidth',.6,'Alpha',1);
end
for k=1:numel(g)
 text(ax,(s0(k)+e(k))/2,-.255,g(k).name,'Units','data','HorizontalAlignment','center', ...
   'VerticalAlignment','top','FontSize',S.font_tick,'FontAngle','italic', ...
   'Color',S.ink,'Clipping','off');
end
% The group row is positioned in normalized y, which 'data' units cannot express
% on a truncated ordinate, so it is placed after the axes limits are final.
for t=findobj(ax,'Type','text')'
 if strcmp(t.FontAngle,'italic');t.Position(2)=ax.YLim(1)-.315*diff(ax.YLim);end
end

lg=legend(ax,[b h90],[label {'90 % target'}],'Orientation','horizontal','Box','off', ...
  'FontSize',S.font_legend,'TextColor',S.ink);
lg.Units='normalized';lg.Position=[.068 .915 .80 .065];

out=fullfile(S.outdir,'fig_benchmark_suppression.png');
exportgraphics(f,out,'Resolution',S.res);close(f);
fprintf('Wrote %s\n',out);
end
