function report=test_rcflex_spherical_clearance()
% Sampled surface clearance, not a continuous collision/tolerance certificate.
d=load(project_path('assets','assembly.mat'));H=d.H;gap=inf;part='';
for p=deg2rad(linspace(-17,17,25))
 for r=deg2rad(linspace(-17,17,25))
  P=[cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)];
  R=P*[1 0 0;0 cos(r) -sin(r);0 sin(r) cos(r)];
  for k=1:numel(d.A)
   a=d.A(k);
   if strcmp(a.body,'SpoonCarrier');V=a.V*R';
   elseif strcmp(a.body,'PitchCarrier');V=a.V*P';else;continue;end
   V=V(V(:,1)>=H.bezel_x_limits_mm(1)&V(:,1)<=H.bezel_x_limits_mm(2),:);
   if isempty(V);continue;end
   g=H.bezel_inner_radius_mm-max(vecnorm(V,2,2));
   if g<gap;gap=g;part=a.name;end
  end
 end
end
report=struct('minimum_sampled_bezel_gap_mm',gap,'limiting_part',part,...
 'concentric_ball_radial_gap_mm',H.bezel_inner_radius_mm-H.ball_outer_radius_mm);
disp(report);assert(gap>.2,'Moving geometry intersects spherical bezel.');
end
