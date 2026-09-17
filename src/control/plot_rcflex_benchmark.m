function plot_rcflex_benchmark(mode)
% Benchmark comparison figures, read from control_benchmark.csv. Only arms
% actually present in the CSV are drawn, so this is usable mid-sweep.
%
% mode is THE SINGLE SWITCH selecting which arms appear:
%   'final' (default) the headline three-way comparison the report leads with:
%                     simple_pid (baseline) vs ekf_cv (IMU-only EKF) vs
%                     ekf_cv_semg (sEMG+IMU).
%   'fusion'          adds ekf_cv_qfix, the IMU-only ABLATION of the sEMG
%                     mechanism. This is the mode to read when the question is
%                     "what did the sEMG stream actually buy?", because the
%                     ablation applies the identical Q-scaling with no sensor.
%   'exp1'            simple_pid vs ekf_cv vs ekf_cv_osc2 (second tremor
%                     oscillator experiment).
%   'debug'           every arm, including ekf_cv_legacy. That arm is retained
%                     for historical attribution only -- it is what isolates the
%                     intent-propagation fix from the algorithm change -- and is
%                     not part of the controller comparison.
% benchmark_rcflex_control still RUNS every arm, so all numbers stay
% reproducible; this switch only governs what is plotted.
if nargin<1||isempty(mode);mode='final';end
T=readtable(project_path('results','control_benchmark.csv'),'TextType','string');
switch mode
 case 'final';want=["simple_pid","ekf_cv","ekf_cv_semg"];
 case 'fusion';want=["simple_pid","ekf_cv","ekf_cv_qfix","ekf_cv_semg"];
 case 'exp1';want=["simple_pid","ekf_cv","ekf_cv_osc2"];
 case 'debug';want=["simple_pid","ekf_cv_legacy","ekf_cv","ekf_cv_osc2","ekf_cv_qfix","ekf_cv_semg"];
 otherwise;error('SensoSpoon:PlotBenchmark','Unknown mode: %s (use ''final'', ''fusion'', ''exp1'' or ''debug'')',mode);
end
arms=want(ismember(want,unique(T.arm)));arms=arms(:);na=numel(arms);
assert(na>0,'No requested arm found in control_benchmark.csv');
% Categorical slots 1-3, validated against a LIGHT surface, so every figure below
% pins Theme='light' rather than inheriting the desktop's dark theme. Colour is
% keyed to the ARM, not to its position in the list, so dropping the legacy arm
% never repaints the two that remain.
slot=[42 120 214;235 104 52;27 175 122;237 161 0;140 140 138;148 60 190]/255;
key=["simple_pid","ekf_cv_legacy","ekf_cv","ekf_cv_osc2","ekf_cv_qfix","ekf_cv_semg"];
col=zeros(na,3);for a=1:na;col(a,:)=slot(key==arms(a),:);end
ink=[.04 .04 .04];ink2=[.32 .32 .31];bad=[.75 .1 .1];
label=strrep(cellstr(arms),'_','\_');
ncase=20;cases=(1:ncase)';
scen=strings(ncase,1);base=nan(ncase,1);fq=nan(ncase,1);
for k=1:ncase
 r=T(T.case_id==k,:);
 if ~isempty(r);scen(k)=r.scenario(1);base(k)=r.baseline_rms_deg(1);fq(k)=r.frequency_Hz(1);end
end
supp=nan(ncase,na);rms=nan(ncase,na);ok=true(ncase,na);
for a=1:na
 for k=1:ncase
  q=T(T.arm==arms(a)&T.case_id==k,:);
  if ~isempty(q);supp(k,a)=q.suppression_percent;rms(k,a)=q.attitude_rms_deg;ok(k,a)=q.within_limits==1;end
 end
end
tick=arrayfun(@(k)sprintf('%d %s %gHz',k,scen(k),fq(k)),cases,'UniformOutput',false);
outdir=project_path('figures');if ~isfolder(outdir);mkdir(outdir);end

% ---- Figure 1: suppression, broken y axis --------------------------------
% The working cases sit in 85-100 % while the two failures reach -197 %. On one
% linear axis the failures flatten everything else, so the axis is split.
f1=figure('Name','Suppression by case','Color','w','Position',[60 60 1460 760]);f1.Theme='light';
axT=axes(f1,'Position',[.07 .585 .90 .275]);
axB=axes(f1,'Position',[.07 .215 .90 .315]);
for p=1:2
 if p==1;ax=axT;else;ax=axB;end
 hold(ax,'on');
 b=bar(ax,cases,supp,'grouped','EdgeColor','none','BarWidth',.86);
 for a=1:na;b(a).FaceColor=col(a,:);end
 yline(ax,90,'--','Color',ink2,'LineWidth',1.4);
 yline(ax,0,'-','Color',ink2,'LineWidth',1);
 set(ax,'XLim',[.4 ncase+.6],'FontSize',9,'XColor',ink2,'YColor',ink2,'Color','w', ...
   'Box','off','TickDir','out','YGrid','on','XGrid','off','GridColor',[.85 .85 .84]);
 if p==1
  set(ax,'YLim',[84 101],'YTick',85:5:100,'XTickLabel',[]);
  text(ax,ncase+.5,90,' 90% target','Color',ink2,'FontSize',9,'VerticalAlignment','bottom','HorizontalAlignment','right');
  title(ax,'Tremor suppression across the 20-case benchmark — identical cases and sensor seeds','FontSize',13,'Color',ink,'FontWeight','bold');
  lg=legend(ax,b,label,'FontSize',10,'Box','off','TextColor',ink,'Orientation','horizontal');
  lg.Units='normalized';lg.Position=[.07 .895 .30 .035];
 else
  set(ax,'YLim',[-215 70],'YTick',-200:50:50,'XTick',cases,'XTickLabel',tick,'XTickLabelRotation',55);
 end
 % Direct-label the bars that miss the target. This is the secondary encoding the
 % palette's contrast warning requires, and the misses are the story.
 for a=1:na
  for k=1:ncase
   v=supp(k,a);if isnan(v);continue;end
   inview=v>=ax.YLim(1)&&v<=ax.YLim(2);
   if inview&&v<90
    text(ax,b(a).XEndPoints(k),v,sprintf(' %.0f',v),'HorizontalAlignment','left', ...
      'VerticalAlignment','middle','FontSize',7.5,'Color',ink2,'Rotation',90);
   end
  end
 end
end
ylabel(axT,'suppression (%)','FontSize',11,'Color',ink);
ylabel(axB,'suppression (%)','FontSize',11,'Color',ink);
% Axis-break marks on both facing edges.
for s=[0 1]
 annotation(f1,'line',[.055 .085]+s*.885,[.566 .586],'Color',ink2,'LineWidth',1.2);
 annotation(f1,'line',[.055 .085]+s*.885,[.551 .571],'Color',ink2,'LineWidth',1.2);
end
% Flag every case that left the hardware envelope, regardless of suppression.
viol=find(any(~ok,2));
for k=viol'
 text(axB,k,-215,'limits','HorizontalAlignment','center','VerticalAlignment','bottom', ...
   'FontSize',8,'FontWeight','bold','Color',bad,'Rotation',90);
end
if any(isnan(supp(10,:)))
 text(axT,10,92.5,{'case 10: no tremor,','suppression undefined','(see error figure)'}, ...
   'HorizontalAlignment','center','FontSize',8,'Color',ink2);
end
annotation(f1,'textbox',[.07 .015 .90 .075],'String', ...
 'Red "limits" = case exceeded the hardware envelope (joint travel / current / power / side load); its suppression number is not a usable result.', ...
 'EdgeColor','none','FontSize',9,'Color',ink2,'VerticalAlignment','top');
exportgraphics(f1,fullfile(outdir,'bench_suppression.png'),'Resolution',140);

% ---- Figure 2: absolute attitude error, log scale ------------------------
f2=figure('Name','Attitude error by case','Color','w','Position',[80 80 1460 660]);f2.Theme='light';
ax=axes(f2,'Position',[.07 .26 .90 .63]);hold(ax,'on');
b2=bar(ax,cases,rms,'grouped','EdgeColor','none','BarWidth',.86);
for a=1:na;b2(a).FaceColor=col(a,:);end
h=plot(ax,cases,base,'d','MarkerSize',8,'MarkerFaceColor',[.15 .15 .15],'MarkerEdgeColor','w','LineStyle','none');
set(ax,'YScale','log','XLim',[.4 ncase+.6],'XTick',cases,'XTickLabel',tick,'XTickLabelRotation',55, ...
  'FontSize',9,'XColor',ink2,'YColor',ink2,'Color','w','Box','off','TickDir','out','YGrid','on','XGrid','off','GridColor',[.85 .85 .84]);
ylabel(ax,'attitude error RMS (deg) — lower is better','FontSize',11,'Color',ink);
xlabel(ax,'benchmark case','FontSize',11,'Color',ink2);
title(ax,'Absolute attitude error — the only valid metric on case 10 (tremor-free voluntary motion)','FontSize',13,'Color',ink,'FontWeight','bold');
legend(ax,[b2 h],[label;{'no control (uncontrolled hand)'}],'Location','northwest','FontSize',10,'Box','off','TextColor',ink,'Orientation','horizontal');
% A bar above the diamond means the controller is worse than doing nothing.
for k=1:ncase
 if any(rms(k,:)>base(k))&&base(k)>0
  text(ax,k,max(rms(k,:))*1.35,'worse than no control','Rotation',90,'HorizontalAlignment','left', ...
    'VerticalAlignment','middle','FontSize',7.5,'Color',bad,'FontWeight','bold');
 end
end
exportgraphics(f2,fullfile(outdir,'bench_attitude_error.png'),'Resolution',140);

% ---- Figure 3: change vs the frozen simple_pid baseline ------------------
ib=find(arms=="simple_pid");
if ~isempty(ib)&&na>1
 other=setdiff(1:na,ib);d=supp(:,other)-supp(:,ib);
 f3=figure('Name','Change vs baseline','Color','w','Position',[100 100 1460 620]);f3.Theme='light';
 ax=axes(f3,'Position',[.07 .28 .90 .60]);hold(ax,'on');
 b3=bar(ax,cases,d,'grouped','EdgeColor','none','BarWidth',.8);
 for j=1:numel(other);b3(j).FaceColor=col(other(j),:);end
 yline(ax,0,'-','Color',ink2,'LineWidth',1.4);
 set(ax,'XLim',[.4 ncase+.6],'XTick',cases,'XTickLabel',tick,'XTickLabelRotation',55,'FontSize',9, ...
   'XColor',ink2,'YColor',ink2,'Color','w','Box','off','TickDir','out','YGrid','on','XGrid','off','GridColor',[.85 .85 .84]);
 ylabel(ax,{'change in suppression vs simple\_pid','(percentage points)'},'FontSize',11,'Color',ink);
 xlabel(ax,'benchmark case','FontSize',11,'Color',ink2);
 title(ax,'Where the EKF helps and where it hurts, against the frozen baseline','FontSize',13,'Color',ink,'FontWeight','bold');
 legend(ax,b3,label(other),'Location','northwest','FontSize',10,'Box','off','TextColor',ink,'Orientation','horizontal');
 for j=1:numel(other)
  for k=1:ncase
   v=d(k,j);if isnan(v);continue;end
   text(ax,b3(j).XEndPoints(k),v,sprintf(' %+.0f',v),'HorizontalAlignment',ternary(v>=0,'left','right'), ...
     'VerticalAlignment','middle','FontSize',7.5,'Color',ink2,'Rotation',90);
  end
 end
 % A delta between two envelope-breaching runs is not an improvement; say so on
 % the chart rather than letting case 20's +105 read as a win.
 for k=find(any(~ok,2))'
  text(ax,k,ax.YLim(1),'both arms breach limits — delta not meaningful ', ...
    'Rotation',90,'HorizontalAlignment','left','VerticalAlignment','middle', ...
    'FontSize',7.5,'FontWeight','bold','Color',bad);
 end
 exportgraphics(f3,fullfile(outdir,'bench_delta_vs_baseline.png'),'Resolution',140);
end
fprintf('arms plotted: %s\n',strjoin(cellstr(arms),', '));
fprintf('figures written to %s\n',outdir);
end
function o=ternary(c,a,b);if c;o=a;else;o=b;end;end
