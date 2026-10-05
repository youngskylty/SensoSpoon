function plot_report_reach()
%PLOT_REPORT_REACH IEEE report figure 4: the fast-reach scenario and the gate.
%
% Section F.1 of the report states that a 0.8 s voluntary reach superimposed on
% tremor was added, that a 350 ms response target was declared in advance, and
% that the delivered gate achieves 407 ms. Those are three numbers with no
% picture, and the scenario they describe is the one case in the whole study
% where the delivered system is WORSE than the alternative. This figure shows
% it, because a limitation the reader can see is more credible than one only
% asserted.
%
% (a) is the scenario: the raw hand attitude the IMU measures, the voluntary
%     motion hidden inside it, and what each estimator believes that voluntary
%     motion to be.
% (b) is the mechanism: the gate confidence that scales the estimator's intent
%     process noise, with the movement onset, the 350 ms target and the
%     achieved response drawn on it, so the shortfall is legible rather than
%     quoted.
%
% Re-runs the two closed-loop arms (about 45 s) rather than reading a CSV,
% because a time trace is not something the benchmark saves.
S=rcflex_figure_style('double');

arms={'off',false;'intent_gate',true};
R=cell(2,1);
for a=1:2
 C=rcflex_control_parameters('ekf_predictive');
 C.ekf_intent_model='cv';C.semg_fusion=arms{a,1};C.semg_enabled=arms{a,2};
 C=rcflex_prepare_control(C);C.seed=4243;
 D=rcflex_control_case('reach',4,8,1,C.seed,C);
 R{a}=run_rcflex_control_fast(D,C,rcflex_parameters,5e-4);
end
C=rcflex_control_parameters('ekf_predictive');   % for the declared target only

t=R{2}.time;
hand=rad2deg(R{2}.base(:,1));          % measured hand roll: intent + tremor
truth=rad2deg(R{2}.intent(:,1));       % voluntary truth, never given to the controller
refOff=rad2deg(R{1}.control(:,4));     % IMU-only estimate of voluntary motion
refSemg=rad2deg(R{2}.control(:,4));    % sEMG + IMU estimate
conf=R{2}.control(:,22);               % gate confidence, roll axis

% Onset and response use the SAME definitions as run_fast_movement_experiment's
% reach_latency and tests/test_rcflex_semg_intent_gate: onset is t = 2 s, the
% commanded start of the minimum-jerk profile, and the gate is "open" at a
% confidence above 0.5. Measuring the figure by any other rule -- for instance
% by where the voluntary truth visibly leaves rest, which is about 70 ms later
% -- would produce a smaller number than the text quotes and make the figure
% appear to contradict it. The seed is one of the four the experiment declared.
onset=2.0;
iResp=find(t>onset&conf>0.5,1);
assert(~isempty(iResp),'Gate never opened on the reach; nothing to mark.');
resp=t(iResp)-onset;
target=0.350;

cOff=S.arm_rgb(S.arm_key=="ekf_cv",:);
cSemg=S.arm_rgb(S.arm_key=="ekf_cv_semg",:);
grey=[.72 .72 .70];

fig=figure('Color','w','Visible','off','Units','inches','Position',[1 1 S.width_in 3.60]);
if isprop(fig,'Theme');fig.Theme='light';end

% ---- (a) the scenario -----------------------------------------------------
ax=axes(fig,'Units','normalized','Position',[.085 .565 .895 .345]);hold(ax,'on');
plot(ax,t,hand,'-','Color',grey,'LineWidth',.5);
plot(ax,t,truth,'-','Color',[.12 .12 .12],'LineWidth',1.4);
plot(ax,t,refOff,'-','Color',cOff,'LineWidth',.9);
plot(ax,t,refSemg,'-','Color',cSemg,'LineWidth',.9);
rcflex_axis_style(ax,S);
set(ax,'XLim',[0 4],'XTick',0:.5:4,'XTickLabel',[]);
ylabel(ax,'Roll (\circ)','FontSize',S.font_label,'Color',S.ink);
lg=legend(ax,{'Measured hand (intent + tremor)','Voluntary motion, truth', ...
  'Estimate, IMU only','Estimate, sEMG + IMU'}, ...
  'Orientation','horizontal','Box','off','FontSize',S.font_legend,'TextColor',S.ink);
lg.Units='normalized';lg.Position=[.085 .930 .895 .050];
lg.ItemTokenSize=[9 6];
text(ax,-.068,1.07,'(a)','Units','normalized','FontSize',S.font_panel, ...
  'FontWeight','bold','Color',S.ink);

% ---- (b) the gate ---------------------------------------------------------
ax=axes(fig,'Units','normalized','Position',[.085 .175 .895 .300]);hold(ax,'on');
area(ax,t,conf,'FaceColor',cSemg,'FaceAlpha',.16,'EdgeColor','none');
plot(ax,t,conf,'-','Color',cSemg,'LineWidth',1.0);
% The target and the achieved response are 53 ms apart on a 4 s axis, so
% labelling both in place guarantees they overlap. They go in a legend instead.
hOn=xline(ax,onset,'-','Color',[.12 .12 .12],'LineWidth',.7);
hTgt=xline(ax,onset+target,'--','Color',S.ref,'LineWidth',.7);
hAch=xline(ax,onset+resp,':','Color',cSemg,'LineWidth',1.2);
rcflex_axis_style(ax,S);
set(ax,'XLim',[0 4],'XTick',0:.5:4,'YLim',[0 1.05],'YTick',0:.25:1);
xlabel(ax,'Time (s)','FontSize',S.font_label,'Color',S.ink);
ylabel(ax,{'Gate confidence','(roll)'},'FontSize',S.font_label,'Color',S.ink);
lg2=legend(ax,[hOn hTgt hAch],{'Movement onset', ...
  sprintf('%d ms target',round(target*1e3)), ...
  sprintf('%d ms achieved',round(resp*1e3))}, ...
  'Orientation','horizontal','Box','off','FontSize',S.font_legend, ...
  'TextColor',S.ink,'Location','southeast');
lg2.ItemTokenSize=[12 6];
text(ax,-.068,1.07,'(b)','Units','normalized','FontSize',S.font_panel, ...
  'FontWeight','bold','Color',S.ink);

out=fullfile(S.outdir,'fig_reach_gate.png');
exportgraphics(fig,out,'Resolution',S.res);close(fig);
fprintf('Wrote %s (onset %.3f s, response %d ms against a %d ms target)\n', ...
 out,onset,round(resp*1e3),round(target*1e3));
end
