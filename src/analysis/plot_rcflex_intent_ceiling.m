function plot_rcflex_intent_ceiling()
%PLOT_RCFLEX_INTENT_CEILING The dilemma in one figure: same knob, opposite ways.
%
% Reads output/results/intent_authority_ceiling.csv, so it can be regenerated
% without re-running the 30 closed-loop simulations behind it. Split out of
% run_intent_authority_experiment.m for exactly that reason -- a plotting bug
% should never cost a half-hour sweep.
T=readtable(project_path('results','intent_authority_ceiling.csv'),'TextType','string');
scales=unique(T.q_intent_scale);            % ascending, which the axis requires
x=log2(scales);
lab=arrayfun(@(s)sprintf('1/%g',1/s),scales,'uni',0);
ink=[.04 .04 .04];ink2=[.32 .32 .31];
f=figure('Color','w','Visible','off','Position',[100 100 1500 560]);
if isprop(f,'Theme');f.Theme='light';end
tiledlayout(1,2);

nexttile;hold on;
for nm=["mixed","irregular"]
 ix=T.scenario==nm;
 [xx,o]=sort(log2(T.q_intent_scale(ix)));yy=T.suppression_percent(ix);yy=yy(o);
 plot(xx,yy,'-o','LineWidth',1.6,'DisplayName',nm);
end
ix=T.scenario=="combined";
[xx,o]=sort(log2(T.q_intent_scale(ix)));yy=T.suppression_percent(ix);yy=yy(o);
plot(xx,yy,'-s','LineWidth',1.6,'Color',[.85 .2 .2],'DisplayName','combined (intent+tremor)');
set(gca,'XTick',x,'XTickLabel',lab,'XColor',ink2,'YColor',ink2,'Box','off','TickDir','out');
xlabel('intent process noise scale');ylabel('Tremor suppression (%)');
title({'Lowering the intent process noise','helps tremor-only cases, destroys mixed ones'},'Color',ink,'FontWeight','bold');
legend('Location','southwest','Box','off');grid on;

nexttile;
ix=T.scenario=="intent";
[xx,o]=sort(log2(T.q_intent_scale(ix)));yy=T.attitude_rms_deg(ix);yy=yy(o);
plot(xx,yy,'-o','LineWidth',1.6,'Color',[.85 .2 .2]);
set(gca,'XTick',x,'XTickLabel',lab,'XColor',ink2,'YColor',ink2,'Box','off','TickDir','out');
xlabel('intent process noise scale');ylabel('Attitude error (deg RMS)');
title({'Voluntary-motion-only case','the same knob suppresses the movement'},'Color',ink,'FontWeight','bold');grid on;

out=project_path('figures','intent_authority_ceiling.png');
exportgraphics(f,out,'Resolution',160);close(f);
fprintf('Wrote %s\n',out);
end
