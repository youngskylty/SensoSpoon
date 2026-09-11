function preview_spoon_dimensions()
H=rcflex_parameters();s=unique([linspace(0,62,600) H.bowl_front_station_mm 37.2]);w=rcflex_spoon_outline(s,H);
f=figure('Color','w','Position',[100 100 1450 850],'Visible','off');if isprop(f,'Theme');f.Theme='light';end
tiledlayout(2,2,'TileSpacing','loose','Padding','loose');
nexttile([1 2]);fill([s fliplr(s)],[w -fliplr(w)],[.37 .39 .42]);hold on;
plot([62 75 75 62],[5 5 -5 -5],'k-');axis equal;axis([-5 83 -31 32]);grid on;
dim([0 27],[62 27],'62 mm');dim([70 -20],[70 20],'40 mm');
plot([37.2 37.2],[-23 23],'--','Color',[.2 .4 .7]);text(37.2,-26,'37.2 mm from tip (60%)','HorizontalAlignment','center');
dim([H.bowl_front_station_mm -14],[H.bowl_front_station_mm 14],'28 mm');dim([78 -5],[78 5],'10 mm');
title('TOP / drawing dimensions in mm');xlabel('Distance from spoon tip (mm)');ylabel('Width (mm)');
r=zeros(size(s));left=s<=37.2;r(left)=(37.2-s(left))/37.2;r(~left)=(s(~left)-37.2)/(62-37.2);
g=1-r.^2;g(~left)=1-3*r(~left).^2+2*r(~left).^3;outer=-1.2-10.8*g;inner=-10.2*g;
nexttile;fill([s fliplr(s)],[outer fliplr(inner)],[.37 .39 .42]);hold on;dim([-3 -12],[-3 0],'12 mm');axis equal;axis([-8 66 -15 4]);grid on;title('SIDE / longitudinal centre section');xlabel('Distance from tip (mm)');
nexttile;y=linspace(-20,20,400);r=abs(y)/20;outer=-1.2-10.8*(1-r.^2);inner=-10.2*(1-r.^2);
fill([y fliplr(y)],[outer fliplr(inner)],[.37 .39 .42]);hold on;
dim([0 -12],[0 -10.2],'1.8 mm');dim([22 -1.2],[22 0],'1.2 mm');axis equal;axis([-24 31 -15 4]);grid on;title('SECTION / maximum-width station');xlabel('Lateral position (mm)');
sgtitle('62 x 40 x 12 mm revised spoon | front-width station inferred from elliptical nose','FontSize',15);
exportgraphics(f,project_path('figures','spoon_dimension_verification.png'),'Resolution',160);close(f);
end
function dim(a,b,label)
plot([a(1) b(1)],[a(2) b(2)],'-','Color',[.05 .3 .7],'LineWidth',1);
plot([a(1) b(1)],[a(2) b(2)],'+','Color',[.05 .3 .7]);
c=(a+b)/2;if a(1)==b(1);c=c+[1 0];else;c=c+[0 1.3];end
text(c(1),c(2),label,'Color',[.05 .3 .7],'FontSize',10,'BackgroundColor','w');
end
