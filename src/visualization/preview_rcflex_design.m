function f=preview_rcflex_design(viewMode,saveImage)
%PREVIEW_RCFLEX_DESIGN Dimensionally scaled preview of the actual STL assets.
% viewMode: assembled, cutaway, exploded, orthographic. Units are millimetres.
if nargin<1;viewMode='cutaway';end
if nargin<2;saveImage=true;end
if ~exist(project_path('assets','assembly.mat'),'file');generate_rcflex_assets();end
S=load(project_path('assets','assembly.mat'));A=S.A;H=S.H;
f=figure('Color','w','Position',[100 100 1400 650],'Name',['RC-Flex / ' viewMode]);
if isprop(f,'Theme');f.Theme='light';end
if strcmp(viewMode,'orthographic')
    tiledlayout(2,2,'TileSpacing','compact','Padding','loose');
    views=[0 90;0 0;90 0;-35 25]; names={sprintf('TOP / %g mm overall',H.total_length_mm),'SIDE / direct pushrods','FRONT / spherical carrier','ASSEMBLY'};
    for j=1:4;ax=nexttile;drawparts(ax,A,'assembled');view(ax,views(j,:));title(ax,names{j});end
else
    ax=axes(f,'Position',[.035 .09 .93 .74]);drawparts(ax,A,viewMode);view(ax,[-35 25]);
    xlim(ax,[-110 175]);ylim(ax,[-60 60]);zlim(ax,[-65 55]);
    camtarget(ax,[65 0 0]);campos(ax,[-260 -400 280]);camup(ax,[0 0 1]);camva(ax,20);
    annotation(f,'textbox',[.04 .89 .92 .08],'String',['RC-Flex / ' upper(viewMode)], ...
        'EdgeColor','none','FontWeight','bold','FontSize',19);
    annotation(f,'textbox',[.04 .83 .92 .06],'String',sprintf('%g mm / staggered voice coils / narrow grip',H.total_length_mm), ...
        'EdgeColor','none','FontSize',11);
    annotation(f,'textbox',[.04 .015 .92 .045],'String','Detachable bowl   /   internal-leaf flexure pivots   /   grounded voice coils + pushrods   /   2S LiPo and electronics', ...
        'EdgeColor','none','FontSize',10);
end
if saveImage
    if ~exist(project_path('figures'),'dir');mkdir(project_path('figures'));end
    exportgraphics(f,fullfile(project_path('figures'),['rcflex_' viewMode '.png']),'Resolution',170);
end
end

function drawparts(ax,A,mode)
hold(ax,'on');
for k=1:numel(A)
    P=A(k);alpha=1;V=P.V;
    if strcmp(mode,'cutaway')
        if any(strcmp(P.name,{'Shell_upper','Spherical_fixed_bezel','Spherical_moving_carrier','Gimbal_guard','Control_panel','Power_button'})) || startsWith(P.name,'Status_LED');continue;end
        if strcmp(P.name,'Bellows_seal');alpha=.13;end
    elseif strcmp(mode,'exploded')
        if strcmp(P.category,'Spoon');V=V+[-55 0 0];
        elseif strcmp(P.category,'FlexureFrame');V=V+[-20 0 0];
        elseif startsWith(P.name,'Pushrod') || startsWith(P.name,'Rod_end');V=V+[-10 0 0];
        elseif strcmp(P.name,'Shell_upper') || strcmp(P.name,'Control_panel') || strcmp(P.name,'Power_button') || startsWith(P.name,'Status_LED');V=V+[0 0 50];
        elseif strcmp(P.name,'Shell_lower') || startsWith(P.name,'Grip');V=V+[0 0 -35];
        elseif strcmp(P.category,'Electronics');V=V+[0 0 28];
        elseif strcmp(P.category,'Battery');V=V+[0 0 -12];end
        if strcmp(P.name,'Bellows_seal');alpha=.12;end
    end
    patch(ax,'Faces',P.F,'Vertices',V,'FaceColor',P.color,'EdgeColor','none', ...
        'FaceAlpha',alpha,'FaceLighting','gouraud','AmbientStrength',.45,'DiffuseStrength',.65, ...
        'SpecularStrength',.55,'SpecularExponent',35);
end
axis(ax,'equal');axis(ax,'off');set(ax,'Projection','orthographic');
camlight(ax,'headlight');camlight(ax,'left');
end
function label(ax,p,q,s)
plot3(ax,[p(1) q(1)],[p(2) q(2)],[p(3) q(3)],'Color',[.35 .4 .46]);
text(ax,q(1),q(2),q(3),s,'FontSize',9,'Color',[.1 .16 .22],'BackgroundColor','w','Margin',2);
end
