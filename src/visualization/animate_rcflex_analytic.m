function f=animate_rcflex_analytic(S,videoFile,viewMode)
% Geometry follows SOLVED q/current. Both rods retain exact length at every frame.
if nargin<1;d=load(project_path('results','analytic_voltage.mat'),'S');S=d.S;end
if nargin<2;videoFile='';end
if nargin<3;viewMode='cutaway';end
d=load(project_path('assets','assembly.mat'),'A');A=d.A;H=S.H;L0=rcflex_linkage([0;0],H);
f=figure('Color','w','Position',[80 80 1300 720],'Name','RC-Flex physical voltage test');
if isprop(f,'Theme');f.Theme='light';end
ax=axes(f,'Position',[.03 .15 .94 .74]);hold(ax,'on');hh=gobjects(numel(A),1);
for k=1:numel(A)
    alpha=1;
    if strcmp(viewMode,'cutaway')
        if any(strcmp(A(k).name,{'Shell_upper','Spherical_fixed_bezel','Spherical_moving_carrier','Gimbal_guard','Control_panel','Power_button'}))||startsWith(A(k).name,'Status_LED');alpha=0;end
        if endsWith(A(k).name,'_housing')&&startsWith(A(k).name,'VCA');alpha=.22;end
    end
    hh(k)=patch(ax,'Vertices',A(k).V-[H.grip_x_mm 0 0],'Faces',A(k).F,'FaceColor',A(k).color,'EdgeColor','none','FaceAlpha',alpha,'FaceLighting','gouraud');
end
axis(ax,'equal');axis(ax,'off');view(ax,[-35 22]);xlim(ax,[-180 115]);ylim(ax,[-60 60]);zlim(ax,[-65 55]);
camtarget(ax,[-50 0 -5]);campos(ax,[80 -400 180]);camup(ax,[0 0 1]);camva(ax,24);
camlight(ax,'headlight');camlight(ax,'left');set(ax,'Projection','orthographic');
status=annotation(f,'textbox',[.05 .9 .90 .09],'EdgeColor','none','FontSize',13,'FontWeight','bold','Color',[.1 .15 .2]);
footer='Open-loop voltage test: motion comes from the RL circuit and mechanical equations. No stabilisation algorithm.';
if strcmp(S.mode,'closed_loop');footer=['Closed loop: dual IMU + current PI. Solved electromechanical motion. Solver: ' strrep(S.solver,'_',' ')];end
annotation(f,'textbox',[.05 .02 .9 .08],'String',footer, ...
    'EdgeColor','none','FontSize',11,'Color',[.1 .15 .2]);
if ~isempty(videoFile);writer=VideoWriter(videoFile,'MPEG-4');writer.FrameRate=30;open(writer);cleanup=onCleanup(@()close(writer));end
for n=unique(round(linspace(1,numel(S.time),ceil(S.time(end)*30)+1)))
    if ~isgraphics(f);break;end
    q=S.Y(n,1:2)';L=rcflex_linkage(q,H);Rp=ry(q(1));Rs=Rp*rx(q(2));Hb=rx(S.base(n,1))*ry(S.base(n,2));
    for k=1:numel(A)
        V=A(k).V;body=A(k).body;
        if strcmp(body,'PitchCarrier');V=V*Rp';
        elseif strcmp(body,'SpoonCarrier');V=V*Rs';
        elseif startsWith(body,'Plunger_');j=1+endsWith(body,'B');V=V+[1000*L.s(j) 0 0];
        elseif startsWith(body,'InputRod_');j=1+endsWith(body,'B');V=rod(V,L0.input(:,j),L0.slider(:,j),L.input(:,j),L.slider(:,j));end
        V=(V-[H.grip_x_mm 0 0])*Hb';
        if size(S.base,2)>=15;V=V+1000*S.base(n,7:9);end
        set(hh(k),'Vertices',V);
    end
    set(status,'String',sprintf('Moticont GVCM-016-019-01M x2 | t = %.2f s\nV = [%+.2f, %+.2f] V    I = [%+.3f, %+.3f] A    pitch / roll = [%+.2f, %+.2f] deg', ...
        S.time(n),S.Y(n,11:12),S.Y(n,5:6),rad2deg(q)));
    drawnow;
    if ~isempty(videoFile);writeVideo(writer,getframe(f));end
end
if isgraphics(f);name='rcflex_physical_test.png';if strcmp(S.mode,'closed_loop');name='rcflex_control_animation.png';end;exportgraphics(f,project_path('figures',name),'Resolution',150);end
end
function V=rod(V,a,b,c,d)
u=(b-a)/norm(b-a);v=(d-c)/norm(d-c);w=cross(u,v);co=dot(u,v);
K=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];R=eye(3)+K+K*K/max(1+co,eps);
V=(V-1000*(a+b)'/2)*R'+1000*(c+d)'/2;
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
