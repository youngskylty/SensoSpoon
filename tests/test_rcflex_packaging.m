function report=test_rcflex_packaging()
% Nominal fixed-component surface-vertex containment in the shell cavity.
% Does not include intentional through-port USB or the front attachment plate.
d=load(project_path('assets','assembly.mat'));H=d.H;worst=0;part='';
for k=1:numel(d.A)
 a=d.A(k);
 if ~strcmp(a.body,'Handle')||~(startsWith(a.name,'VCA_')||strcmp(a.category,'Battery')||strcmp(a.category,'Electronics'));continue;end
 V=a.V;
 yy=(H.handle_mm(2)/2-H.shell_wall_mm)*interp1(H.shell_x_mm,H.shell_y_scale,V(:,1),'pchip');
 zz=(H.handle_mm(3)/2-H.shell_wall_mm)*interp1(H.shell_x_mm,H.shell_z_scale,V(:,1),'pchip');
 e=interp1([0 55 168],[1 .85 .85],V(:,1),'linear','extrap');
 V(:,3)=V(:,3)-interp1(H.shell_x_mm,H.shell_center_z_mm,V(:,1),'pchip');
   ratio=(abs(V(:,2))./yy).^(2./e)+(abs(V(:,3))./zz).^(2./e);
 assert(all(isfinite(ratio)),'Component lies beyond shell profile.');
 if max(ratio)>worst;worst=max(ratio);part=a.name;end
end
report=struct('maximum_cavity_utilisation',worst,'limiting_part',part);
disp(report);assert(worst<1,'Fixed component intersects shell cavity.');
end
