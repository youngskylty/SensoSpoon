function plot_report_real_tremor()
%PLOT_REPORT_REAL_TREMOR IEEE report figure 3: recorded human tremor.
%
% Rebuilt from output/results/real_tremor_replay.csv plus the recordings
% themselves, so it always describes the sweep that is on disk.
%
% (a) is the premise and (b) is the consequence. (a) puts four seconds of a
% conditioned recording next to the RMS-matched sine the benchmark has been
% using all along, which is the clearest statement of what the synthetic tremor
% model left out. (b) is what replacing one with the other costs each of the
% three controllers, as a PAIRED comparison over the records that have both a
% recorded run and a synthetic control at their own dominant frequency.
%
% The record drawn in (a) is the one whose crest factor is the MEDIAN of the
% primary set. Choosing the most extreme record would make the same point more
% dramatically and less honestly.
S=rcflex_figure_style('single');
f=project_path('results','real_tremor_replay.csv');
assert(isfile(f),'SensoSpoon:RealTremor', ...
 'No saved replay at %s. Run START_PROJECT(''realtremor'') first.',f);
T=readtable(f,'TextType','string');

arms=["simple_pid","ekf_cv","ekf_cv_semg"];
label={'Baseline PID','EKF, IMU only','EKF + sEMG'};
arms=arms(ismember(arms,unique(T.arm)));
na=numel(arms);
assert(na>0,'No known arm in %s',f);

% Paired subset: records carrying both a recorded row and a synthetic control.
rec=unique(T.record(T.input=="recorded"));
ctl=unique(T.record(T.input=="sine_matched"));
pair=intersect(rec,ctl);
assert(~isempty(pair),'No record has both a recorded run and a synthetic control.');

% --- representative record for panel (a) ---------------------------------
prim=T(T.input=="recorded"&T.minutes_off==0,:);
[~,iu]=unique(prim.record,'stable');prim=prim(iu,:);
[~,im]=min(abs(prim.crest_factor-median(prim.crest_factor)));
show=char(prim.record(im));
tw=(0:1/2000:2)';
[shape,meta]=rcflex_tremor_record(show,tw);
sine=rms(shape(:,1))*sqrt(2)*sin(2*pi*meta.dominant_Hz*tw);

% Panel (b) is restricted to records whose tremor is inside the declared 4-20 Hz
% envelope. Outside it the baseline's fixed notch chain has nothing to reject
% with, so those records say something about the design brief rather than about
% waveform shape, and letting them set the error bars would hide the comparison
% the panel exists to make. The full set is in the table.
band=unique(T.record(T.input=="recorded"&T.in_design_band==1));
pair=intersect(pair,band);
assert(~isempty(pair),'No in-band record carries both a recorded run and a control.');

fig=figure('Color','w','Visible','off','Units','inches','Position',[1 1 S.width_in 3.95]);
if isprop(fig,'Theme');fig.Theme='light';end
grey=[.62 .62 .61];
purple=S.arm_rgb(S.arm_key=="ekf_cv_semg",:);

% --- (a) waveform ---------------------------------------------------------
ax=axes(fig,'Units','normalized','Position',[.190 .690 .785 .230]);hold(ax,'on');
plot(ax,tw,sine,'-','Color',grey,'LineWidth',.6);
plot(ax,tw,shape(:,1),'-','Color',purple,'LineWidth',.6);
rcflex_axis_style(ax,S);
set(ax,'XLim',[0 2],'XTick',0:.5:2,'YLim',[-2.2 2.2],'YTick',-2:1:2);
xlabel(ax,'Time (s)','FontSize',S.font_label,'Color',S.ink);
ylabel(ax,'Tremor (norm.)','FontSize',S.font_label,'Color',S.ink);
lg=legend(ax,{sprintf('Synthetic sine, %.1f Hz',meta.dominant_Hz), ...
  sprintf('Recorded, patient %s',meta.subject)}, ...
  'Orientation','horizontal','Box','off','FontSize',S.font_legend,'TextColor',S.ink);
lg.Units='normalized';lg.Position=[.190 .930 .785 .050];
lg.ItemTokenSize=[9 6];
text(ax,-.175,1.06,'(a)','Units','normalized','FontSize',S.font_panel, ...
  'FontWeight','bold','Color',S.ink);

% --- (b) paired suppression ----------------------------------------------
M=nan(na,2);E=nan(na,2);
for a=1:na
 for c=1:2
  if c==1;in="sine_matched";else;in="recorded";end
  v=T.suppression_percent(T.arm==arms(a)&T.input==in&ismember(T.record,pair));
  M(a,c)=mean(v);E(a,c)=std(v);
 end
end
ax=axes(fig,'Units','normalized','Position',[.190 .160 .785 .375]);hold(ax,'on');
b=bar(ax,1:na,M,'grouped','EdgeColor','none','BarWidth',.78);
b(1).FaceColor=grey;b(2).FaceColor=purple;
for c=1:2
 errorbar(ax,b(c).XEndPoints,M(:,c),E(:,c),'LineStyle','none', ...
   'Color',S.ink,'LineWidth',.5,'CapSize',2.5);
end
yline(ax,90,'--','Color',S.ref,'LineWidth',.7);
rcflex_axis_style(ax,S);
set(ax,'XLim',[.45 na+.55],'XTick',1:na,'XTickLabel',label,'XTickLabelRotation',14);
ylabel(ax,'Tremor suppression (%)','FontSize',S.font_label,'Color',S.ink);
lo=min(M(:)-E(:));
set(ax,'YLim',[max(0,10*floor(lo/10)) 100],'YTick',0:10:100);
lg=legend(ax,b,{'Synthetic sine',sprintf('Recorded tremor (n=%d)',numel(pair))}, ...
  'Orientation','horizontal','Box','off','FontSize',S.font_legend,'TextColor',S.ink);
lg.Units='normalized';lg.Position=[.190 .545 .785 .050];
lg.ItemTokenSize=[9 6];
text(ax,-.175,1.20,'(b)','Units','normalized','FontSize',S.font_panel, ...
  'FontWeight','bold','Color',S.ink);

out=fullfile(S.outdir,'fig_real_tremor.png');
exportgraphics(fig,out,'Resolution',S.res);close(fig);
fprintf('Wrote %s (paired over %d in-band records, panel (a) shows %s)\n',out,numel(pair),show);
end
