function report=test_rcflex_shell_clearance(limit)
% Sample moving surface vertices against the proposed inner shell profile.
% Not a triangle/triangle collision certificate, nor a tolerance stack-up.
d=load(project_path('assets','assembly.mat'));H=d.H;worst=0;name='';guardWorst=0;guardName='';
if nargin<1;limit=17;end
for p=linspace(-limit,limit,13)
 for r=linspace(-limit,limit,13)
  P=ry(deg2rad(p));R=P*rx(deg2rad(r));
  for k=1:numel(d.A)
   a=d.A(k);if strcmp(a.body,'SpoonCarrier');V=a.V*R';elseif strcmp(a.body,'PitchCarrier');V=a.V*P';else;continue;end
   if isfield(H,'guard_x_mm')
    W=V(V(:,1)>=H.guard_x_mm(1)&V(:,1)<=H.guard_x_mm(end),:);
    if ~isempty(W)
     gy=(H.handle_mm(2)/2-1)*interp1(H.guard_x_mm,H.guard_y_scale,W(:,1));
     gz=(H.handle_mm(3)/2-1)*interp1(H.guard_x_mm,H.guard_z_scale,W(:,1));
     theta=atan2(W(:,3)./gz,W(:,2)./gy);theta(theta<H.guard_angle_rad(1))=theta(theta<H.guard_angle_rad(1))+2*pi;
     covered=theta<=H.guard_angle_rad(2);
     rr=(abs(W(:,2))./gy).^(2/.7)+(abs(W(:,3))./gz).^(2/.7);
     if any(covered)&&max(rr(covered))>guardWorst;guardWorst=max(rr(covered));guardName=a.name;end
    end
   end
   V=V(V(:,1)>=H.shell_x_mm(1)&V(:,1)<=35,:);if isempty(V);continue;end
   yy=(H.handle_mm(2)/2-H.shell_wall_mm)*interp1(H.shell_x_mm,H.shell_y_scale,V(:,1),'pchip');
   zz=(H.handle_mm(3)/2-H.shell_wall_mm)*interp1(H.shell_x_mm,H.shell_z_scale,V(:,1),'pchip');
   exponent=interp1([0 55 168],[1 .85 .85],V(:,1),'linear','extrap');
   V(:,3)=V(:,3)-interp1(H.shell_x_mm,H.shell_center_z_mm,V(:,1),'pchip');
   ratio=(abs(V(:,2))./yy).^(2./exponent)+(abs(V(:,3))./zz).^(2./exponent);
   if max(ratio)>worst;worst=max(ratio);name=a.name;end
  end
 end
end
report=struct('max_inner_profile_utilisation',worst,'limiting_part',name);
report.guard_utilisation=guardWorst;report.guard_limiting_part=guardName;
disp(report);assert(worst<1,'Sampled moving surface intersects proposed shell.');
assert(guardWorst<1,'Sampled moving surface intersects partial guard.');
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
