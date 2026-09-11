function f=plot_rcflex_control(S,filename)
if nargin<2;filename='control_response.png';end
[m,E]=rcflex_control_metrics(S);t=S.time;
f=figure('Color','w','Position',[80 80 1200 850],'Name','SensoSpoon closed-loop validation');
if isprop(f,'Theme');f.Theme='light';end
tiledlayout(3,2,'TileSpacing','compact');
nexttile;plot(t,rad2deg(S.base(:,2)-S.intent(:,2)),'Color',[.65 .65 .65]);hold on;plot(t,rad2deg(E(:,2)),'b');ylabel('Pitch error (deg)');legend('Locked spoon baseline','Controlled','Location','best');grid on;
nexttile;plot(t,rad2deg(S.base(:,1)-S.intent(:,1)),'Color',[.65 .65 .65]);hold on;plot(t,rad2deg(E(:,1)),'b');ylabel('Roll error (deg)');grid on;
nexttile;plot(t,S.Y(:,5:6));hold on;yline(S.H.current_limit_A,'r--');yline(-S.H.current_limit_A,'r--');ylabel('Coil current (A)');legend('A','B');grid on;
nexttile;plot(t,S.Y(:,15:16)*1000);hold on;yline(S.H.vca_stroke_mm,'r--');yline(-S.H.vca_stroke_mm,'r--');ylabel('Slider displacement (mm)');grid on;
nexttile;plot(t,E(:,4:6)*1000);ylabel('Bowl position error (mm)');xlabel('Time (s)');legend('x','y','z');grid on;
nexttile;plot(t,S.Y(:,21:22));hold on;yline(S.H.coil_continuous_power_W,'r--');ylabel('Instantaneous copper loss (W)');xlabel('Time (s)');grid on;
sgtitle(sprintf('%s | attitude RMS %.3f deg | suppression %.1f%% | position RMS %.2f mm',strrep(S.solver,'_',' '),m.attitude_rms_deg,m.suppression_percent,m.position_rms_mm));
exportgraphics(f,project_path('figures',filename),'Resolution',150);
end
