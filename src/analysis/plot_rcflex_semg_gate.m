function plot_rcflex_semg_gate(scenario,fHz,amp,dur)
%PLOT_RCFLEX_SEMG_GATE The mechanism figure: what sEMG actually does, in time.
%
% Runs one scenario twice -- IMU-only and sEMG+IMU -- and plots the chain the
% architecture claims: the muscle signal says whether the movement is
% voluntary, that opens or closes the estimator's intent authority, and the
% resulting reference either follows the voluntary movement or refuses to
% follow the tremor.
%
% Default scenario is 'reach': a fast voluntary movement superimposed on
% tremor, which is the case that can tell a gate that preserves voluntary
% motion apart from one that merely happens never to be tested on any.
if nargin<1||isempty(scenario);scenario='reach';end
if nargin<2||isempty(fHz);fHz=8;end
if nargin<3||isempty(amp);amp=1;end
if nargin<4||isempty(dur);dur=4;end

arms={'off',false;'intent_gate',true};
S=cell(2,1);
for a=1:2
 C=rcflex_control_parameters('ekf_predictive');
 C.ekf_intent_model='cv';C.semg_fusion=arms{a,1};C.semg_enabled=arms{a,2};
 C=rcflex_prepare_control(C);C.seed=9207;
 D=rcflex_control_case(scenario,dur,fHz,amp,C.seed,C);
 S{a}=run_rcflex_control_fast(D,C,rcflex_parameters,.0005);
 S{a}.D=D;
end
t=S{2}.time;
% ref is logged [pitch roll]; intent truth columns are [roll pitch].
refOff=rad2deg(S{1}.control(:,4));refSemg=rad2deg(S{2}.control(:,4));
truth=rad2deg(S{2}.intent(:,1));
handRaw=rad2deg(S{2}.base(:,1));
conf=S{2}.control(:,22);

ink=[.04 .04 .04];ink2=[.32 .32 .31];
cOff=[27 175 122]/255;cSemg=[148 60 190]/255;cTruth=[.15 .15 .15];
f=figure('Color','w','Visible','off','Position',[80 80 1280 900]);
if isprop(f,'Theme');f.Theme='light';end
tl=tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;hold on;
plot(t,handRaw,'-','Color',[.72 .72 .70],'LineWidth',.8);
plot(t,truth,'-','Color',cTruth,'LineWidth',2.2);
plot(t,refOff,'-','Color',cOff,'LineWidth',1.3);
plot(t,refSemg,'-','Color',cSemg,'LineWidth',1.3);
ylabel('roll (deg)');grid on;set(gca,'XColor',ink2,'YColor',ink2,'Box','off','TickDir','out');
legend({'measured hand attitude (intent + tremor)','TRUE voluntary intent',...
 'estimated intent, IMU-only','estimated intent, sEMG+IMU'},'Location','northwest','Box','off');
title('What the estimator calls voluntary motion','Color',ink,'FontWeight','bold');

nexttile;hold on;
plot(t,refOff-truth,'-','Color',cOff,'LineWidth',1.3);
plot(t,refSemg-truth,'-','Color',cSemg,'LineWidth',1.3);
yline(0,'-','Color',ink2);
ylabel({'error in the voluntary','estimate (deg)'});grid on;
set(gca,'XColor',ink2,'YColor',ink2,'Box','off','TickDir','out');
legend({'IMU-only','sEMG+IMU'},'Location','northwest','Box','off');
title('Oscillation here is tremor being absorbed into intent and passed to the spoon','Color',ink,'FontWeight','bold');

nexttile;hold on;
area(t,conf,'FaceColor',cSemg,'FaceAlpha',.22,'EdgeColor',cSemg,'LineWidth',1.4);
yline(1,'--','Color',ink2);
ylim([0 1.08]);ylabel({'sEMG voluntary-drive','confidence (roll)'});xlabel('time (s)');
grid on;set(gca,'XColor',ink2,'YColor',ink2,'Box','off','TickDir','out');
title('The gate: intent process noise is scaled by this, per axis, per sample','Color',ink,'FontWeight','bold');
text(.05,.92,sprintf('warm-up: gate held open to 1.0 s so it cannot assert\nan absence of drive before its filters have settled'),...
 'Units','normalized','FontSize',8.5,'Color',ink2,'VerticalAlignment','top');

title(tl,sprintf('sEMG intent-authority gate — scenario ''%s'', %g Hz tremor, %g deg',scenario,fHz,amp),...
 'FontWeight','bold','FontSize',13);
out=project_path('figures',sprintf('semg_gate_%s.png',scenario));
exportgraphics(f,out,'Resolution',150);close(f);
fprintf('Wrote %s\n',out);

m1=rcflex_intent_split_metrics(S{1});m2=rcflex_intent_split_metrics(S{2});
M1=rcflex_control_metrics(S{1});M2=rcflex_control_metrics(S{2});
fprintf('%-12s %14s %16s %18s %16s\n','arm','suppression','attitude RMS','tremor leak (deg)','intent track (deg)');
fprintf('%-12s %13.2f%% %15.4f %18.4f %16.4f\n','IMU-only',M1.suppression_percent,M1.attitude_rms_deg,m1.tremor_leak_deg,m1.intent_track_deg);
fprintf('%-12s %13.2f%% %15.4f %18.4f %16.4f\n','sEMG+IMU',M2.suppression_percent,M2.attitude_rms_deg,m2.tremor_leak_deg,m2.intent_track_deg);
end
