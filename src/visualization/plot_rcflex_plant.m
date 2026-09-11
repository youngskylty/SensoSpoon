function plot_rcflex_plant(S)
if nargin<1;d=load(project_path('results','analytic_voltage.mat'),'S');S=d.S;end
f=figure('Color','w','Position',[100 100 1200 850]);if isprop(f,'Theme');f.Theme='light';end
tiledlayout(3,2,'TileSpacing','compact');t=S.time;y=S.Y;
data={S.voltage_request,y(:,5:6),rad2deg(y(:,1:2)),y(:,15:16)*1000,y(:,13:14),y(:,21:22)};
titles={'Applied voltage request','Solved coil current','Solved spoon joint angle','Solved VCA displacement','Electromagnetic force','Copper loss'};
units={'V','A','deg','mm','N','W'};
for k=1:6;nexttile;plot(t,data{k},'LineWidth',1);grid on;xlabel('Time (s)');ylabel(units{k});title(titles{k});end
sgtitle('Physical plant: open-loop common and differential voltage tests');
exportgraphics(f,project_path('figures','rcflex_voltage_response.png'),'Resolution',150);
end
