function preview_spoon_closeup()
d=load(project_path('assets','assembly.mat'));f=figure('Color','w','Position',[80 80 1100 700],'Visible','off');
if isprop(f,'Theme');f.Theme='light';end
ax=axes(f);hold(ax,'on');
for k=1:numel(d.A)
 a=d.A(k);if ~(strcmp(a.category,'Spoon')||startsWith(a.name,'Spherical_')||strcmp(a.name,'Quick_release_collar'));continue;end
 patch(ax,'Vertices',a.V,'Faces',a.F,'FaceColor',a.color,'EdgeColor','none','FaceLighting','gouraud','SpecularStrength',.45);
end
axis(ax,'equal');axis(ax,'off');xlim(ax,[-110 25]);ylim(ax,[-35 35]);zlim(ax,[-28 28]);
view(ax,[-35 25]);camtarget(ax,[-45 0 -4]);campos(ax,[-145 -260 220]);camva(ax,21);
camlight(ax,'headlight');camlight(ax,'left');
title(ax,'Deeper bowl / slender curved neck','FontSize',16);
exportgraphics(f,project_path('figures','spoon_curved_neck_closeup.png'),'Resolution',160);close(f);
end

