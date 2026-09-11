function f=animate_rcflex_comparison(S,videoFile,liquid)
% Same base input and camera; rigidly locked baseline versus solved control.
% The locked baseline is NOT a simulation of a freely moving unpowered gimbal.
if nargin<1;d=load(project_path('results','control_native.mat'),'S');S=d.S;end
if nargin<2;videoFile=project_path('figures','control_comparison.mp4');end
hasLiquid=nargin>=3;
assert(strcmp(S.solver,'Simulink_ode23t'),'Use the native continuous reference');
d=load(project_path('assets','assembly.mat'),'A');A=d.A;H=S.H;
L0=rcflex_linkage([0;0],H);[m,E]=rcflex_control_metrics(S);
inputLabel='Identical recorded handle input';
if isfield(S,'scenario');inputLabel=sprintf('Same %g Hz, %g deg amplitude per axis',S.scenario.frequency_Hz,S.scenario.amplitude_deg);end
errorLimit=max(1,1.15*max(abs(rad2deg([S.base(:,1:2)-S.intent(:,1:2);E(:,1:2)])),[],'all'));
f=figure('Color','w','Position',[50 50 1500 850],'Name','SensoSpoon: identical-input comparison');
if isprop(f,'Theme');f.Theme='light';end
colors=[.80 .22 .16;.02 .46 .52];ax=gobjects(2,1);parts=gobjects(numel(A),2);
for j=1:2
 ax(j)=axes(f,'Position',[.025+(j-1)*.5 .32 .45 .49]);hold(ax(j),'on');
 for k=1:numel(A)
  parts(k,j)=patch(ax(j),'Vertices',A(k).V-[H.grip_x_mm 0 0],'Faces',A(k).F,'FaceColor',A(k).color,'EdgeColor','none','FaceLighting','gouraud');
 end
 axis(ax(j),'equal');axis(ax(j),'off');xlim(ax(j),[-180 115]);ylim(ax(j),[-60 60]);zlim(ax(j),[-55 55]);
 camtarget(ax(j),[-33 0 -5]);campos(ax(j),[67 -400 170]);camup(ax(j),[0 0 1]);camva(ax(j),25);
 set(ax(j),'Projection','orthographic');camlight(ax(j),'headlight');camlight(ax(j),'left');
end
annotation(f,'textbox',[.02 .91 .96 .075],'String',sprintf('SensoSpoon | %s | %g g payload',inputLabel,1000*H.food_mass_kg),'EdgeColor','none','FontSize',19,'FontWeight','bold','HorizontalAlignment','center');
if hasLiquid
 for j=1:2
  camtarget(ax(j),[-133 0 -10]);campos(ax(j),[-110 -150 110]);camva(ax(j),23);
  xlim(ax(j),[-177 -85]);ylim(ax(j),[-42 42]);zlim(ax(j),[-50 25]);
 end
 titles=findall(f,'Type','textboxshape');delete(titles);
 annotation(f,'textbox',[.02 .91 .96 .075],'String',sprintf('SensoSpoon | %.1f mL soup | %s',liquid.initial_mL,inputLabel),'EdgeColor','none','FontSize',19,'FontWeight','bold','HorizontalAlignment','center');
end
labels={'WITHOUT COMPENSATION','WITH COMPENSATION'};
subs={'Rigid connection: gimbal locked to handle','Dual IMU + PID: native Simulink dynamics'};
for j=1:2
 annotation(f,'textbox',[.02+(j-1)*.5 .86 .46 .04],'String',labels{j},'EdgeColor','none','FontSize',17,'FontWeight','bold','Color',colors(j,:),'HorizontalAlignment','center');
 annotation(f,'textbox',[.02+(j-1)*.5 .82 .46 .035],'String',subs{j},'EdgeColor','none','FontSize',11,'HorizontalAlignment','center');
end
status=annotation(f,'textbox',[.04 .30 .92 .055],'EdgeColor','none','FontSize',13,'HorizontalAlignment','center');
if hasLiquid
 water=gobjects(2,1);droplets=gobjects(2,1);amount=gobjects(2,1);
 pv=liquid.P;pv.grid_mm=.25;VG=rcflex_liquid_geometry(H,pv);
 highlight=.7*exp(-((VG.x+.008)/.019).^2-((VG.y-.006)/.003).^2);
 rgb=zeros([size(VG.x) 3]);baseColor=[.86 .57 .16];
 for channel=1:3;rgb(:,:,channel)=baseColor(channel)+(1-baseColor(channel))*highlight;end
 for j=1:2
  water(j)=surf(ax(j),nan(size(VG.X)),nan(size(VG.X)),nan(size(VG.X)),rgb,'FaceColor','interp','EdgeColor','none','FaceAlpha',.72,'FaceLighting','gouraud','SpecularStrength',.9,'SpecularExponent',70);
  droplets(j)=scatter3(ax(j),nan,nan,nan,1,[.90 .43 .04],'filled');
  amount(j)=annotation(f,'textbox',[.025+(j-1)*.5 .76 .45 .04],'EdgeColor','none','FontSize',12,'HorizontalAlignment','center');
 end
end
curves=gobjects(2,1);dots=gobjects(2,2);
for j=1:2
 a=axes(f,'Position',[.07+(j-1)*.5 .09 .40 .17]);hold(a,'on');
 component=3-j; % pitch=2, roll=1
 plot(a,S.time,rad2deg(S.base(:,component)-S.intent(:,component)),'Color',colors(1,:),'LineWidth',1.3);
 plot(a,S.time,rad2deg(E(:,component)),'Color',colors(2,:),'LineWidth',1.3);
 curves(j)=xline(a,0,'k:');
 for v=1:2;dots(j,v)=plot(a,0,0,'o','MarkerFaceColor',colors(v,:),'MarkerEdgeColor','w','MarkerSize',6);end
 grid(a,'on');xlim(a,[0 S.time(end)]);ylim(a,[-errorLimit errorLimit]);xlabel(a,'Simulation time (s)');
 if j==1;ylabel(a,'Pitch error (deg)');else;ylabel(a,'Roll error (deg)');end
 legend(a,{'Without','With'},'Location','southwest','Orientation','horizontal','FontSize',9);
end
note=sprintf('Attitude RMS: %.3f -> %.3f deg (%.1f%% reduction). Same motion scale; bowl position is not perfectly fixed.',m.baseline_rms_deg,m.attitude_rms_deg,m.suppression_percent);
if hasLiquid;note='Reduced slosh + rim overflow model (one-way, not CFD). Same liquid/forcing on both sides. Spill is computed, not scripted.';end
annotation(f,'textbox',[.03 .01 .94 .04],'String',note,'EdgeColor','none','FontSize',12,'HorizontalAlignment','center');
w=VideoWriter(videoFile,'MPEG-4');w.FrameRate=30;w.Quality=95;open(w);cleanup=onCleanup(@()close(w));
for pass=1:2
 speed=1;if pass==2;speed=.25;end
 times=linspace(0,S.time(end),round(S.time(end)*30/speed)+1);
 for t=times
  b=interp1(S.time,S.base,t);qSolved=interp1(S.time,S.Y(:,1:2),t)';Hb=rx(b(1))*ry(b(2));
  for j=1:2
   q=zeros(2,1);if j==2;q=qSolved;end
   L=rcflex_linkage(q,H);Rp=ry(q(1));Rs=Rp*rx(q(2));
   for k=1:numel(A)
    V=A(k).V;body=A(k).body;
    if strcmp(body,'PitchCarrier');V=V*Rp';
    elseif strcmp(body,'SpoonCarrier');V=V*Rs';
    elseif startsWith(body,'Plunger_');v=1+endsWith(body,'B');V=V+[1000*L.s(v) 0 0];
    elseif startsWith(body,'InputRod_');v=1+endsWith(body,'B');V=rod(V,L0.input(:,v),L0.slider(:,v),L.input(:,v),L.slider(:,v));end
    V=(V-[H.grip_x_mm 0 0])*Hb'+1000*b(7:9);set(parts(k,j),'Vertices',V);
   end
   if hasLiquid
    frame=min(numel(liquid.time),max(1,round(t*liquid.P.output_hz)+1));G=VG;s=liquid.sides(j).slope(frame,:);depth=max(0,liquid.sides(j).level(frame)+s(1)*G.x+s(2)*G.y-G.b);
    z=G.b+depth+G.center(3);z(depth<liquid.P.dry_depth_m|~G.mask)=NaN;
    V=[G.X(:) G.Y(:) z(:)]*1000*Rs';V=(V-[H.grip_x_mm 0 0])*Hb'+1000*b(7:9);
    set(water(j),'XData',reshape(V(:,1),size(z)),'YData',reshape(V(:,2),size(z)),'ZData',reshape(V(:,3),size(z)));
    amount(j).String=sprintf('Remaining %.3f mL | Spilled %.3f mL',liquid.sides(j).remaining_mL(frame),liquid.sides(j).spilled_mL(frame));
    e=liquid.sides(j).events;age=t-e(:,1);use=age>=0&age<.35;e=e(use,:);age=age(use);
    pos=e(:,2:4)+age.*e(:,5:7)+.5*age.^2.*[0 0 -9.80665];
    radius=(3*e(:,8)/(4*pi)).^(1/3)*1000;
    set(droplets(j),'XData',pos(:,1)*1000,'YData',pos(:,2)*1000,'ZData',pos(:,3)*1000,'SizeData',max(.1,(2*radius*4).^2));
   end
  end
  for j=1:2
   component=3-j;curves(j).Value=t;
   set(dots(j,1),'XData',t,'YData',rad2deg(b(component)-interp1(S.time,S.intent(:,component),t)));
   set(dots(j,2),'XData',t,'YData',rad2deg(interp1(S.time,E(:,component),t)));
  end
  mode='REAL TIME';if pass==2;mode='4x SLOW MOTION (amplitude unchanged)';end
  status.String=sprintf('%s  |  simulation t = %.3f s',mode,t);
  drawnow;writeVideo(w,getframe(f));
 end
end
name='control_comparison.png';if hasLiquid;[~,name]=fileparts(videoFile);name=[name '.png'];end
exportgraphics(f,project_path('figures',name),'Resolution',140);
end
function V=rod(V,a,b,c,d)
u=(b-a)/norm(b-a);v=(d-c)/norm(d-c);w=cross(u,v);co=dot(u,v);
K=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];R=eye(3)+K+K*K/max(1+co,eps);
V=(V-1000*(a+b)'/2)*R'+1000*(c+d)'/2;
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
