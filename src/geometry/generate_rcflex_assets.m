function A=generate_rcflex_assets(H)
%GENERATE_RCFLEX_ASSETS Parametric, closed triangle meshes at drawing scale.
% All parts use the same nominal global mm coordinates. STL files use metres.
% Curved bowl, hollow shell, crossed leaves, yokes, grounded VCA housings,
% pushrods, battery, PCB, boot, balance mass, fasteners and quick release.
if nargin<1;H=rcflex_parameters();end
out=project_path('assets');
if ~exist(out,'dir'); mkdir(out); end
A=struct('name',{},'body',{},'category',{},'color',{},'mass',{},'V',{},'F',{},'file',{});
silver=[.58 .60 .63]; dark=[.10 .11 .13]; copper=[.61 .32 .16];
% Hollow ergonomic shell, longitudinal axis x. Upper/lower independently visible.
add('Shell_lower','Handle','Housing',[.16 .17 .19],.020,shellmesh(H,pi,2*pi));
add('Shell_upper','Handle','Housing',[.76 .78 .80],.020,shellmesh(H,0,pi));
M=sphereshell(H.bezel_outer_radius_mm,H.bezel_inner_radius_mm,H.bezel_x_limits_mm);
[~,~,vol]=rcflex_mesh_mass_properties(M.V/1000,M.F,1);
add('Spherical_fixed_bezel','Handle','Housing',[.76 .78 .80],vol*1200,M);
M=sphereshell(H.ball_outer_radius_mm,H.ball_inner_radius_mm,H.ball_x_limits_mm);
add('Spherical_moving_carrier','SpoonCarrier','FlexureFrame',[.40 .43 .47],.001,M);
for a=[0 2*pi/3 4*pi/3]
 M=boxmesh([0 0 0],[2 9 1.5]);M.V=M.V*rotx_local(a)'+[-10 9.5*cos(a) 9.5*sin(a)];
 add(sprintf('Ball_carrier_rib_%d',round(a*100)),'SpoonCarrier','FlexureFrame',silver,.0001,M);
end
add('Handle_front_cap','Handle','Housing',[.26 .28 .31],.003,frontcapmesh(H));
% The dark lower shell forms the continuous grip; no protruding grip blocks.
add('Control_panel','Handle','Misc',[.065 .07 .08],.001,panelmesh());
add('Power_button','Handle','Misc',silver,.0005,cylmesh([105 0 14.5],3.5,.7,'z'));
for j=1:4
    add(sprintf('Status_LED_%d',j),'Handle','Misc',[.02 .65 .95],.0001,boxmesh([111+5*j 0 14.9],[1.2 1.5 .2]));
end
% Battery: 2S LiPo pouch per PDF. Rendering's cylindrical blue cell is not
% dimensioned and does not establish a 2S battery format.
add('LiPo_2S_pouch','Handle','Battery',[.18 .32 .56],H.battery_mass_kg,boxmesh([133 0 4],H.battery_mm));
add('PCB','Handle','Electronics',[.08 .31 .17],.004,boxmesh([133 0 12],[45 16 1.2]));
add('STM32G431CB_MCU','Handle','Electronics',dark,.0002,boxmesh([124 0 13.4],[7 7 1.4]));
add('BMI270_IMU','Handle','Electronics',[.15 .15 .17],.0001,boxmesh([117 4 13],[3 2.5 .8]));
add('DRV8876_A','Handle','Electronics',dark,.0002,boxmesh([138 -3.8 13.2],[5 6.4 1.2]));
add('DRV8876_B','Handle','Electronics',dark,.0002,boxmesh([147 3.8 13.2],[5 6.4 1.2]));
add('USB_C','Handle','Misc',silver,.001,boxmesh([163 0 6],[3 8.5 3]));
% Two stators are fixed to Handle. Plungers and rods have separate bodies.
for j=1:2
    c=H.vca_centers_mm(j,:); tag=char('A'+j-1);
    body=['Plunger_' tag];
    add(['VCA_' tag '_winding'],body,'VCA',copper,.009,cylmesh(c+[-4 0 0],6,12,'x'));
    add(['VCA_' tag '_housing'],'Handle','VCA',dark,.014,ringmesh(c+[-2.4 0 0],7.95,6.5,19.1));
    add(['VCA_' tag '_rear'],'Handle','VCA',dark,.003,cylmesh(c+[9.55 0 0],7.95,4.8,'x'));
    add(['VCA_' tag '_mount'],'Handle','Misc',silver,.001,boxmesh(c+[13 0 -5],[2 12 4]));
    add(['VCA_' tag '_plunger'],body,'VCA',silver,.002,cylmesh([c(1)-14.5 c(2) c(3)],1.6,22,'x'));
    if j==2;add('B_shaft_extension',body,'Misc',silver,pi*.0015^2*.032*7800,cylmesh([46 c(2) c(3)],1.5,32,'x'));end
    L0=rcflex_linkage([0;0],H);pa=L0.attach(:,j)'*1000;pb=L0.slider(:,j)'*1000;
    M=cylmesh([0 0 0],1,norm(pb-pa),'x');
    u=(pb-pa)/norm(pb-pa);v=[0 -u(3) u(2)];K=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
    R=eye(3)+K+K*K/(1+u(1));M.V=M.V*R'+(pa+pb)/2;
    rodMass=pi*(H.pushrod_diameter_mm/2000)^2*H.link.input_length(j)*7800;
    add(['Pushrod_' tag],['InputRod_' tag],'Misc',silver,rodMass,M);
    add(['Rod_end_' tag],'SpoonCarrier','Misc',dark,.0002,ellipsoidmesh(pa,[1.5 1.5 1.5]));
    add(['Slider_joint_' tag],['Plunger_' tag],'Misc',silver,.0002,ellipsoidmesh(pb,[1.5 1.5 1.5]));
    add(['Gimbal_socket_' tag],'SpoonCarrier','FlexureFrame',silver,.0004,boxmesh([6 H.link.attachment(2,j)*1000 -8],[2.5 4 6]));
    % Ring grooves visually identify the copper winding without fake joints.
    for n=1:9
        add(sprintf('Coil_%s_turn_%02d',tag,n),body,'Misc',[.35 .20 .10],0,...
            ringmesh(c+[-10+1.2*n 0 0],6.03,5.9,.15));
    end
end
% Open local supports. No complete outer gimbal rings.
% Handle side cheeks -> B10 fixed sleeves -> pitch bridge -> D10 fixed sleeve.
for side=[-1 1]
    M=ringmesh([0 0 0],3,3.96748/2,2.7);
    M.V=M.V(:,[2 1 3])+[0 side*12.45 0];M.F=M.F(:,[1 3 2]);
    add(sprintf('Pitch_fixed_seat_%d',side+2),'Handle','FlexureFrame',silver,.0002,M);
    add(sprintf('Side_cheek_%d',side+2),'Handle','FlexureFrame',silver,.0002,boxmesh([9 side*12.45 0],[18 2.5 3]));
    add(sprintf('Cheek_handle_root_%d',side+2),'Handle','FlexureFrame',silver,.0002,boxmesh([18 side*14 0],[3 5 3]));
end
add('Pitch_bridge','PitchCarrier','FlexureFrame',silver,.001,boxmesh([0 0 0],[2.5 19 2.5]));
% Moving D10 sleeve -> short collet -> transverse drive bridge / spoon hub.
add('Roll_moving_collet','SpoonCarrier','FlexureFrame',silver,.0002,ringmesh([-6 0 0],4.3,3.175,2.2));
add('Roll_drive_bridge','SpoonCarrier','FlexureFrame',silver,.001,boxmesh([-8.3 0 0],[2.5 19 2.5]));
add('Gimbal_rear_saddle','SpoonCarrier','FlexureFrame',silver,.001,boxmesh([-1 0 -6],[16 21 1.5]));
add('Gimbal_saddle_root','SpoonCarrier','FlexureFrame',silver,.0004,boxmesh([-8 0 -3],[2 19 6]));
add('Roll_pivot_collet','PitchCarrier','FlexureFrame',silver,.0002,ringmesh([1 0 0],4.5,3.175,4));
% B-10: OD .1562 in, length .250 in. Sleeves split across pitch interface.
% Mass is a solid-cylinder upper estimate (steel), NOT vendor mass data.
for s=[-1 1]
    mass=pi*(3.96748/2)^2*6.35*7.8e-6/2;
    add(sprintf('Cflex_B10_fixed_%d',s+2),'Handle','FlexureFrame',dark,mass,cylmesh([0 s*(10.8+1.6375) 0],3.96748/2,3.075,'y'));
    add(sprintf('Cflex_B10_moving_%d',s+2),'PitchCarrier','FlexureFrame',silver,mass,cylmesh([0 s*(10.8-1.6375) 0],3.96748/2,3.075,'y'));
end
% D-10: OD .250 in, length .400 in; roll axis remains the longitudinal X axis.
mass=pi*3.175^2*10.16*7.8e-6/2;
add('Cflex_D10_fixed','PitchCarrier','FlexureFrame',dark,mass,cylmesh([.59 0 0],3.175,4.98,'x'));
add('Cflex_D10_moving','SpoonCarrier','FlexureFrame',silver,mass,cylmesh([-4.59 0 0],3.175,4.98,'x'));
add('Floating_yoke_hub','SpoonCarrier','FlexureFrame',silver,.002,ringmesh([-10 0 0],5,3.6,5));
add('Quick_release_collar','SpoonCarrier','Misc',[.48 .51 .54],.0015,ringmesh([-20 0 0],5.5,3.5,6));
add('Bayonet_pin','SpoonCarrier','Misc',dark,.0003,cylmesh([-20 0 0],.8,13,'y'));
N=neckmesh(H);W=bowlmesh(H);
[~,~,vn]=rcflex_mesh_mass_properties(N.V/1000,N.F,1);
[~,~,vw]=rcflex_mesh_mass_properties(W.V/1000,W.F,1);
add('Spoon_neck','SpoonCarrier','Spoon',[.66 .68 .70],vn*7900,N);
add('Spoon_bowl','SpoonCarrier','Spoon',[.73 .75 .77],vw*7900,W);
add('Spoon_IMU_PCB','SpoonCarrier','Electronics',[.08 .31 .17],.0005,boxmesh([-10 0 5],[6 5 .8]));
add('Spoon_BMI270','SpoonCarrier','Electronics',dark,.0001,boxmesh([-10 0 6],[3 2.5 .8]));
% Bellows: one boot; translucent only in cutaway preview, not a rigid bearing.
add('Bellows_seal','SpoonCarrier','Misc',[.14 .15 .17],.0015,ringmesh([-17 0 0],6,5,3));
add('Static_balance_mass','SpoonCarrier','Misc',[.53 .47 .31],.0015,cylmesh([10 0 -7],3.4,6,'y'));
% Purchased VCA/battery masses are never rescaled to an arbitrary total.
% Custom-part masses remain explicit nominal allocations pending CAD/material QA.
for k=1:numel(A)
    % Weld periodic seams/poles and remove zero-area indexing triangles.
    [V,~,map]=unique(round(A(k).V*1e9)/1e9,'rows');
    F=reshape(map(A(k).F),size(A(k).F));
    F=F(F(:,1)~=F(:,2)&F(:,2)~=F(:,3)&F(:,1)~=F(:,3),:);
    A(k).V=V;A(k).F=F;
    % Normal orientation is standardised for downstream CAD/STL viewers.
    V=A(k).V;F=A(k).F;
    volume=sum(dot(V(F(:,1),:),cross(V(F(:,2),:),V(F(:,3),:),2),2))/6;
    if volume<0;A(k).F=F(:,[1 3 2]);end
    A(k).file=fullfile(out,[A(k).name '.stl']);
    write_stl(A(k).file,A(k).V/1000,A(k).F);
end
save(fullfile(out,'assembly.mat'),'A','H');
% Animation uses solved analytic states with exact rod closure.
names={A.name}'; bodies={A.body}'; categories={A.category}'; mass_g=[A.mass]'*1000;
writetable(table(names,bodies,categories,mass_g),fullfile(out,'parts_manifest.csv'));
fprintf('Generated %d detailed parts; mass %.1f g; overall nominal length %.0f mm.\n',numel(A),sum([A.mass])*1000,H.total_length_mm);

    function add(name,body,category,color,mass,M)
        if strcmp(name,'Control_panel')||strcmp(name,'Power_button')||startsWith(name,'Status_LED')
            M.V(:,3)=M.V(:,3)+(H.handle_mm(3)/2)*interp1(H.shell_x_mm,H.shell_z_scale,M.V(:,1),'pchip')+interp1(H.shell_x_mm,H.shell_center_z_mm,M.V(:,1),'pchip')-14;
        end
        if strcmp(category,'FlexureFrame') && ~startsWith(name,'Cflex_')
            v=M.V;ff=M.F;mass=abs(sum(dot(v(ff(:,1),:),cross(v(ff(:,2),:),v(ff(:,3),:),2),2))/6)*2.70e-6;
        end
        A(end+1)=struct('name',name,'body',body,'category',category,'color',color, ...
            'mass',mass,'V',M.V,'F',M.F,'file','');
    end
end

function M=panelmesh()
% Rounded planform with closely sampled length to follow the curved lid.
xs=linspace(99.05,140.95,100);V=[];F=[];
for x=xs
 w=5.5;if x<104.5;w=sqrt(5.5^2-(x-104.5)^2);elseif x>135.5;w=sqrt(5.5^2-(x-135.5)^2);end
 V=[V;repmat([x 0 14.25],4,1)+[0 -w -.3;0 w -.3;0 w .3;0 -w .3]];
end
for i=0:numel(xs)-2
 for j=1:4;a=i*4+j;b=i*4+mod(j,4)+1;F=[F;a b b+4;a b+4 a+4];end
end
n=size(V,1);F=[F;1 3 2;1 4 3;n-3 n-2 n-1;n-3 n-1 n];
M=struct('V',V,'F',F);
end
function M=boxmesh(c,d)
V=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;-1 -1 1;1 -1 1;1 1 1;-1 1 1].*d/2+c;
F=[1 3 2;1 4 3;5 6 7;5 7 8;1 2 6;1 6 5;2 3 7;2 7 6;3 4 8;3 8 7;4 1 5;4 5 8];
M=struct('V',V,'F',F);
end

function M=cylmesh(c,r,len,axis)
n=40;t=(0:n-1)'*2*pi/n; V=[-len/2*ones(n,1) r*cos(t) r*sin(t);len/2*ones(n,1) r*cos(t) r*sin(t);-len/2 0 0;len/2 0 0];
F=[];
for i=1:n;j=mod(i,n)+1; F=[F;i j n+j;i n+j n+i;2*n+1 j i;2*n+2 n+i n+j];end %#ok<AGROW>
if axis=='y'; V=V(:,[2 1 3]); F=F(:,[1 3 2]); elseif axis=='z'; V=V(:,[2 3 1]);end
M=struct('V',V+c,'F',F);
end

function M=ringmesh(c,ro,ri,len)
n=48;t=(0:n-1)'*2*pi/n;V=[];
for x=[-len/2 len/2];for r=[ro ri];V=[V;x*ones(n,1) r*cos(t) r*sin(t)];end;end %#ok<AGROW>
F=[];
for i=1:n;j=mod(i,n)+1;
    quads=[i j 2*n+j 2*n+i;n+i 3*n+i 3*n+j n+j;i n+i n+j j;2*n+i 2*n+j 3*n+j 3*n+i];
    F=[F;quads(:,[1 2 3]);quads(:,[1 3 4])]; %#ok<AGROW>
end
M=struct('V',V+c,'F',F);
end

function p=roundedrect(w,h,r)
p=[];centers=[w/2-r h/2-r;-w/2+r h/2-r;-w/2+r -h/2+r;w/2-r -h/2+r];
for k=1:4;t=(k-1)*pi/2+(0:7)'*pi/16;p=[p;centers(k,:)+r*[cos(t) sin(t)]];end %#ok<AGROW>
end

function M=shellmesh(H,t0,t1)
% Rounded, hollow superellipse shell, capped rear and front rim.
xs=unique([H.shell_x_mm linspace(H.shell_x_mm(1),H.shell_x_mm(end),100)]);
sy=interp1(H.shell_x_mm,H.shell_y_scale,xs,'pchip');
sz=interp1(H.shell_x_mm,H.shell_z_scale,xs,'pchip');
if t0==0;t0=-.35;t1=pi+.35;else;t0=pi+.35;t1=2*pi-.35;end
tt=linspace(t0,t1,49);nv=numel(xs)*numel(tt);V=zeros(2*nv,3);
for layer=1:2
    for i=1:numel(xs)
        yy=(H.handle_mm(2)/2-(layer-1)*H.shell_wall_mm)*sy(i);
        zz=(H.handle_mm(3)/2-(layer-1)*H.shell_wall_mm)*sz(i);
        id=(layer-1)*nv+(i-1)*numel(tt)+(1:numel(tt));
        exponent=interp1([0 55 168],[1 .85 .85],xs(i),'linear','extrap');
        V(id,:)=[xs(i)*ones(numel(tt),1) yy*sign(cos(tt')).*abs(cos(tt')).^exponent zz*sign(sin(tt')).*abs(sin(tt')).^exponent+interp1(H.shell_x_mm,H.shell_center_z_mm,xs(i),'pchip')];
    end
end
F=gridfaces(numel(xs),numel(tt)); F=[F;F(:,[1 3 2])+nv];
edges=boundarygrid(numel(xs),numel(tt));
for k=1:size(edges,1);a=edges(k,1);b=edges(k,2);F=[F;a a+nv b+nv;a b+nv b];end %#ok<AGROW>
M=struct('V',V,'F',F);
end

function M=sphereshell(ro,ri,limits)
xs=linspace(limits(1),limits(2),32);t=(0:79)'*2*pi/80;n=numel(t);nx=numel(xs);V=[];
for radius=[ro ri]
 for x=xs
  r=sqrt(radius^2-x^2);V=[V;x*ones(n,1),r*cos(t),r*sin(t)]; %#ok<AGROW>
 end
end
F=[];nv=n*nx;
for j=1:nx-1;for k=1:n;l=mod(k,n)+1;a=(j-1)*n+k;b=(j-1)*n+l;
 F=[F;a b b+n;a b+n a+n;a+nv b+n+nv b+nv;a+nv a+n+nv b+n+nv];end;end %#ok<AGROW>
for k=1:n;l=mod(k,n)+1;a=(nx-1)*n+k;b=(nx-1)*n+l;
 F=[F;k k+nv l+nv;k l+nv l;a b b+nv;a b+nv a+nv];end %#ok<AGROW>
M=struct('V',V,'F',F);
end

function M=frontcapmesh(H)
% Closed end plate with two genuine rod openings; no decorative solid disk.
t=(0:95)'*2*pi/96;
y=17.5*sign(cos(t)).*abs(cos(t)).^.7;
z=14*sign(sin(t)).*abs(sin(t)).^.7;
p=polyshape(y,z);
for j=1:2
 w=H.front_rod_windows_mm(j,:);yy=w(1)+[-1 1 1 -1]*w(3)/2;zz=w(2)+[-1 -1 1 1]*w(4)/2;
 p=subtract(p,polyshape(yy,zz));
end
T=triangulation(p);n=size(T.Points,1);tri=T.ConnectivityList;
V=[(H.front_cap_x_mm-.7)*ones(n,1) T.Points;(H.front_cap_x_mm+.7)*ones(n,1) T.Points];
F=[tri(:,[1 3 2]);tri+n];e=freeBoundary(T);
for k=1:size(e,1);a=e(k,1);b=e(k,2);F=[F;a b b+n;a b+n a+n];end %#ok<AGROW>
M=struct('V',V,'F',F);
end

function M=bowlmesh(H)
s=unique([linspace(0,H.bowl_mm(1),100) H.bowl_front_station_mm H.bowl_widest_station_mm]);
w=rcflex_spoon_outline(s,H);L=H.bowl_mm(1);
rear=linspace(w(end),-w(end),18)';
outline=[s' w'; L*ones(16,1) rear(2:end-1);flip(s(2:end))' -flip(w(2:end))'];
center=[H.bowl_widest_station_mm 0];n=size(outline,1);nr=28;r=linspace(.00001,1,nr);V=[];
for side=1:2
 for j=1:nr
  xy=center+r(j)*(outline-center);
  span=H.bowl_widest_station_mm*ones(n,1);span(xy(:,1)>center(1))=H.bowl_mm(1)-center(1);
  localWidth=rcflex_spoon_outline(xy(:,1),H);
  t=abs((xy(:,1)-center(1))./span);g=1-t.^2;
  back=xy(:,1)>center(1);g(back)=1-3*t(back).^2+2*t(back).^3;
  f=max(0,g.*(1-(xy(:,2)./max(localWidth,eps)).^2));
  if j==nr;f=zeros(n,1);end
  if side==1;z=-H.bowl_wall_mm-(H.bowl_mm(3)-H.bowl_wall_mm)*f;
  else;z=-(H.bowl_mm(3)-H.bowl_bottom_thickness_mm)*f;end
  V=[V;xy(:,1)+H.bowl_tip_x_mm xy(:,2) z+H.bowl_rim_z_mm]; %#ok<AGROW>
 end
end
F=[];nv=n*nr;
for j=1:nr-1;for k=1:n;l=mod(k,n)+1;a=(j-1)*n+k;b=(j-1)*n+l;c=j*n+l;d=j*n+k;
 F=[F;a b c;a c d;a+nv c+nv b+nv;a+nv d+nv c+nv];end;end %#ok<AGROW>
for k=1:n;l=mod(k,n)+1;a=(nr-1)*n+k;b=(nr-1)*n+l;F=[F;a b a+nv;b b+nv a+nv;k k+nv l;l k+nv l+nv];end %#ok<AGROW>
M=struct('V',V,'F',F);
end

function M=neckmesh(H)
start=H.bowl_tip_x_mm+H.bowl_mm(1);xs=linspace(start,-10,48);V=[];
for x=xs
 w=pchip([start start+10 -20 -10],[H.bowl_connection_width_mm/2 3.5 3 3.2],x);
 u=(x-start)/(-10-start);z=H.bowl_rim_z_mm-.6-H.bowl_rim_z_mm*(3*u^2-2*u^3);
 section=roundedrect(2*w,1.2,.3);nt=size(section,1);
 V=[V;x*ones(nt,1) section(:,1) z+section(:,2)]; %#ok<AGROW>
end
F=[];for j=1:numel(xs)-1;for k=1:nt;l=mod(k,nt)+1;a=nt*(j-1)+k;b=nt*(j-1)+l;F=[F;a b b+nt;a b+nt a+nt];end;end %#ok<AGROW>
nv=size(V,1);V=[V;mean(V(1:nt,:),1);mean(V(end-nt+1:end,:),1)];
for k=1:nt;l=mod(k,nt)+1;F=[F;nv+1 l k;nv+2 nv-nt+k nv-nt+l];end %#ok<AGROW>
M=struct('V',V,'F',F);
end
function M=ellipsoidmesh(c,r)
[x,y,z]=sphere(18);[F,V]=surf2patch(x,y,z,'triangles');
M=struct('V',V.*r+c,'F',F);
end

function F=gridfaces(n,m)
F=[];for i=1:n-1;for j=1:m-1;a=(i-1)*m+j;F=[F;a a+m a+m+1;a a+m+1 a+1];end;end %#ok<AGROW>
end
function E=boundarygrid(n,m)
v=[1:m,m+m:m:n*m,n*m-1:-1:(n-1)*m+1,(n-2)*m+1:-m:1];
E=[v(1:end-1)' v(2:end)'];
end
function R=rotx_local(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function write_stl(file,V,F)
fid=fopen(file,'wb');cleanup=onCleanup(@() fclose(fid));
fwrite(fid,zeros(1,80,'uint8'),'uint8');fwrite(fid,size(F,1),'uint32');
for k=1:size(F,1)
    p=V(F(k,:),:);n=cross(p(2,:)-p(1,:),p(3,:)-p(1,:));n=n/max(norm(n),eps);
    fwrite(fid,[n p(1,:) p(2,:) p(3,:)],'single');fwrite(fid,0,'uint16');
end
end

