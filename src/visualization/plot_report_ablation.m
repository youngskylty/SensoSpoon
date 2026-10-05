function plot_report_ablation()
%PLOT_REPORT_ABLATION IEEE report figure 2: the sEMG ablation.
%
% Rebuilt from output/results/semg_intent_e2.csv. Two panels side by side is the
% whole argument: the IMU-only ablation applies the identical dial-turning
% mechanism with no muscle signal, wins panel (a) and loses panel (b), so the
% gain is attributable to the SENSOR rather than to the mechanism.
%
% Differences from the diagnostic version (plot_semg_intent_experiment):
%  1. Log ordinates. Linear axes put a 3.07 deg outlier beside 0.013 deg values
%     and make five of the six scenarios unreadable.
%  2. Scenarios re-ordered into tremor-only and voluntary-movement-present
%     blocks, which is the split the result turns on.
%  3. Panel letters instead of sentence titles; one legend instead of two; the
%     "lower is better" asides removed to the caption.
S=rcflex_figure_style('double');
f=project_path('results','semg_intent_e2.csv');
assert(isfile(f),'SensoSpoon:SemgIntent', ...
 'No saved E2 result at %s. Run START_PROJECT(''fusion'') first.',f);
T=readtable(f,'TextType','char');

arms={'off','intent_gate_fixed','intent_gate'};
label={'EKF, IMU only','Ablation: gate held fixed, no sEMG','EKF + sEMG'};
key=["ekf_cv","ekf_cv_qfix","ekf_cv_semg"];
scen={'sine','mixed','irregular','combined','intent','reach'};
tick={'sine','mixed','irregular','with tremor','alone','fast reach'};
nblock=3;   % first three scenarios are tremor-only
block={'Tremor only','Voluntary movement'};

n=numel(scen);na=numel(arms);
L=nan(n,na);K=L;col=zeros(na,3);
for a=1:na
 col(a,:)=S.arm_rgb(S.arm_key==key(a),:);
 for i=1:n
  ix=strcmp(T.scenario,scen{i})&strcmp(T.arm,arms{a});
  assert(nnz(ix)==1,'SensoSpoon:SemgIntent','Expected one row for %s / %s.',scen{i},arms{a});
  L(i,a)=T.tremor_leak_deg(ix);K(i,a)=T.intent_track_deg(ix);
 end
end

fig=figure('Color','w','Visible','off','Units','inches','Position',[1 1 S.width_in 2.80]);
if isprop(fig,'Theme');fig.Theme='light';end
yl={'Tremor absorbed into intent (\circ RMS)','Voluntary motion not followed (\circ RMS)'};
pos=[.072 .320 .400 .520;.585 .320 .400 .520];
panel={'(a)','(b)'};
for p=1:2
 if p==1;Y=L;else;Y=K;end
 ax=axes(fig,'Units','normalized','Position',pos(p,:));hold(ax,'on');
 b=bar(ax,1:n,Y,'grouped','EdgeColor','none','BarWidth',.86);
 for a=1:na;b(a).FaceColor=col(a,:);end
 rcflex_axis_style(ax,S);
 lo=10^floor(log10(min(Y(:))));hi=10^ceil(log10(max(Y(:))));
 set(ax,'YScale','log','XLim',[.4 n+.6],'YLim',[lo hi], ...
   'XTick',1:n,'XTickLabel',tick,'XTickLabelRotation',45);
 ylabel(ax,yl{p},'FontSize',S.font_label,'Color',S.ink);
 xline(ax,nblock+.5,'-','Color',S.grid,'LineWidth',.6,'Alpha',1);
 % Block names go on a second row under the tick labels. Normalized units are
 % used because the ordinate is logarithmic, where a data-unit offset is not a
 % constant distance on the page.
 for k=1:2
  if k==1;xm=(1+nblock)/2;else;xm=(nblock+1+n)/2;end
  text(ax,(xm-ax.XLim(1))/diff(ax.XLim),-.375,block{k},'Units','normalized', ...
    'HorizontalAlignment','center','VerticalAlignment','top', ...
    'FontSize',S.font_tick,'FontAngle','italic','Color',S.ink,'Clipping','off');
 end
 text(ax,-.205,1.075,panel{p},'Units','normalized','FontSize',S.font_panel, ...
   'FontWeight','bold','Color',S.ink);
 if p==1
  lg=legend(ax,b,label,'Orientation','horizontal','Box','off', ...
    'FontSize',S.font_legend,'TextColor',S.ink);
  lg.Units='normalized';lg.Position=[.072 .915 .913 .07];
 end
end

out=fullfile(S.outdir,'fig_semg_ablation.png');
exportgraphics(fig,out,'Resolution',S.res);close(fig);
fprintf('Wrote %s\n',out);
end
