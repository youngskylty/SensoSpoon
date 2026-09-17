function T=run_omega_capture_experiment()
% Experiment 2: does a dominant tremor component capturing the shared omega
% systematically push OTHER frequencies into the intent states?
%
% Open-loop estimator only -- no controller, no plant. The filter is the shipped
% corrected arm (ekf_predictive + ekf_intent_model='cv', ekf_osc2 off) with every
% parameter left at its default; this script reads the EKF and changes nothing.
%
% The measurement model is exact (rcflex_intent_ekf_init.m:62):
%     y = th_intent + a1 + a2
% so at the secondary frequency the input can be split, with no modelling
% assumption, into the part the INTENT states claim and the part the TREMOR
% oscillator claims, as complex phasors.
%
% Conditions, at each secondary frequency:
%   'secondary_only'  0.2 deg secondary tone alone           (control)
%   'dominant_plus'   the same tone + a fixed 1.3 deg 8 Hz dominant
% 1.3 deg / 0.2 deg and the 8 Hz carrier are the case-6 pitch proportions
% (rcflex_control_case.m:20 with amplitude=2).
%
% Writes output/results/omega_capture_experiment.csv and a figure. Deterministic:
% no random input, so re-running reproduces the table exactly.
C=rcflex_control_parameters('ekf_predictive');
C.ekf_intent_model='cv';C.ekf_osc2=false;
C=rcflex_prepare_control(C);E=C.ekf;idx=E.idx;n=E.n;
dt=1/C.outer_hz;

domF=8;domAmp=1.3;domPhase=.7;secAmp=.2;secPhase=0;
f2=[2 3 4 5 6 7 8 9];
settle=12;win=12;                  % 12 s settle, then EXACTLY 12 s scored.
                                   % Every probe frequency is an integer, so an
                                   % integer-second window makes the phasor
                                   % projections exactly orthogonal.
t=(0:dt:settle+win)';
k0=find(t>=settle,1);ixw=k0:k0+round(win/dt)-1;tt=t(ixw);nn=numel(ixw);

conds={'secondary_only',false;'dominant_plus',true};
rows=struct([]);
for c=1:size(conds,1)
 for j=1:numel(f2)
  drive=deg2rad(secAmp)*sin(2*pi*f2(j)*t+secPhase);
  if conds{c,2};drive=drive+deg2rad(domAmp)*sin(2*pi*domF*t+domPhase);end
  s=[];N=numel(t);
  th=zeros(N,1);os=zeros(N,1);rs=zeros(N,1);nb=zeros(N,1);w=zeros(N,1);
  for k=1:N
   z=[drive(k);drive(k)];
   [s,~,info]=rcflex_intent_ekf_step(s,z,C);
   x=s(1:n);
   th(k)=x(idx.thy);os(k)=x(idx.a1y)+x(idx.a2y);
   r=z-E.H*x;rs(k)=r(2);            % posterior reconstruction error, pitch axis
   nb(k)=info(2);w(k)=info(1);
  end
  e=exp(-2i*pi*f2(j)*tt);P=@(v)2*sum(v(ixw).*e)/nn;   % complex phasor at f2
  pin=P(drive);pth=P(th);pos=P(os);
  ref=deg2rad(secAmp);                                % nominal secondary input
  row=struct('condition',string(conds{c,1}),'secondary_Hz',f2(j), ...
   'input_nominal_deg',secAmp, ...
   'drive_measured_at_f2_deg',rad2deg(abs(pin)), ...
   'intent_amp_deg',rad2deg(abs(pth)), ...
   'osc_amp_deg',rad2deg(abs(pos)), ...
   'intent_absorption_pct',100*abs(pth)/ref, ...
   'osc_share_pct',100*abs(pos)/ref, ...
   'intent_phase_deg',rad2deg(angle(pth)), ...
   'osc_phase_deg',rad2deg(angle(pos)), ...
   'phase_separation_deg',rad2deg(angle(pos/pth)), ...
   'recon_error_at_f2_pct',100*abs(pth+pos-pin)/ref, ...
   'resid_rms_deg',rad2deg(rms(rs(ixw))), ...
   'nis_bar',mean(nb(ixw)), ...
   'omega_hat_Hz',mean(w(ixw)));
  rows=[rows;row]; %#ok<AGROW>
  fprintf('%-15s f2=%2d Hz: intent %6.1f%%  osc %6.1f%%  omega %5.2f Hz  NIS %6.2f\n', ...
    conds{c,1},f2(j),row.intent_absorption_pct,row.osc_share_pct,row.omega_hat_Hz,row.nis_bar);
 end
end
T=struct2table(rows);
writetable(T,project_path('results','omega_capture_experiment.csv'));
fprintf('\nwrote %s\n',project_path('results','omega_capture_experiment.csv'));
plot_result(T,f2,domF,domAmp,secAmp);
end

function plot_result(T,f2,domF,domAmp,secAmp)
col=[42 120 214;235 104 52]/255;ink=[.04 .04 .04];ink2=[.32 .32 .31];bad=[.75 .1 .1];
a=T.intent_absorption_pct(T.condition=="secondary_only");
b=T.intent_absorption_pct(T.condition=="dominant_plus");
fg=figure('Name','Omega capture experiment','Color','w','Position',[70 70 1180 680]);fg.Theme='light';
ax=axes(fg,'Position',[.09 .14 .88 .72]);hold(ax,'on');
h(1)=plot(ax,f2,a,'-o','Color',col(1,:),'LineWidth',2.2,'MarkerSize',8,'MarkerFaceColor',col(1,:),'MarkerEdgeColor','w');
h(2)=plot(ax,f2,b,'-o','Color',col(2,:),'LineWidth',2.2,'MarkerSize',8,'MarkerFaceColor',col(2,:),'MarkerEdgeColor','w');
for j=1:numel(f2)
 text(ax,f2(j),a(j)-6,sprintf('%.0f',a(j)),'HorizontalAlignment','center','FontSize',9,'Color',col(1,:));
 text(ax,f2(j),b(j)+6,sprintf('%.0f',b(j)),'HorizontalAlignment','center','FontSize',9,'Color',col(2,:));
end
xline(ax,domF,'--','Color',bad,'LineWidth',1.4);
text(ax,domF-.08,118,'dominant tone sits here — secondary is not separable ', ...
 'Rotation',90,'HorizontalAlignment','right','VerticalAlignment','top','FontSize',9,'Color',bad,'FontWeight','bold');
set(ax,'XLim',[1.6 9.4],'YLim',[-8 148],'XTick',f2,'FontSize',10,'XColor',ink2,'YColor',ink2, ...
 'Color','w','Box','off','TickDir','out','YGrid','on','GridColor',[.85 .85 .84]);
xlabel(ax,'secondary tone frequency (Hz)','FontSize',11,'Color',ink2);
ylabel(ax,'share of the secondary component claimed by the INTENT states (%)','FontSize',11,'Color',ink);
title(ax,sprintf('Does a dominant tone capturing omega push other frequencies into intent?  (%.1f deg @ %g Hz + %.1f deg secondary)',domAmp,domF,secAmp), ...
 'FontSize',12.5,'Color',ink,'FontWeight','bold');
legend(ax,h,{'secondary tone ALONE (control)','dominant 8 Hz PRESENT'},'Location','northwest','Box','off','FontSize',10.5,'TextColor',ink);
outdir=project_path('figures');if ~isfolder(outdir);mkdir(outdir);end
exportgraphics(fg,fullfile(outdir,'omega_capture_experiment.png'),'Resolution',140);
fprintf('figure written to %s\n',fullfile(outdir,'omega_capture_experiment.png'));
end
