function report=test_rcflex_front_cap()
% Rod centreline and finite radius clearance through both faces of end plate.
H=rcflex_parameters();margin=inf;frontGap=inf;s=load(project_path('assets','assembly.mat'),'A');
for p=linspace(-17,17,25)
 for r=linspace(-17,17,25)
  L=rcflex_linkage(deg2rad([p;r]),H);
  P=[cosd(p) 0 sind(p);0 1 0;-sind(p) 0 cosd(p)];R=P*[1 0 0;0 cosd(r) -sind(r);0 sind(r) cosd(r)];
  for k=1:numel(s.A)
   a=s.A(k);if strcmp(a.body,'SpoonCarrier');v=a.V*R';elseif strcmp(a.body,'PitchCarrier');v=a.V*P';else;continue;end
   frontGap=min(frontGap,H.front_cap_x_mm-.7-max(v(:,1)));
  end
  for j=1:2
   a=L.attach(:,j)*1000;b=L.slider(:,j)*1000;d=b-a;w=H.front_rod_windows_mm(j,:);
   for x=H.front_cap_x_mm+[-.7 .7]
    f=(x-a(1))/d(1);assert(f>0&&f<1,'Rod does not cross front plate.');c=a+f*d;
    % Oblique cylinder section has conservative radius r/cos(angle).
    radius=H.pushrod_diameter_mm/2*norm(d)/abs(d(1));
    gap=min(w(3)/2-abs(c(2)-w(1)),w(4)/2-abs(c(3)-w(2)))-radius;
    margin=min(margin,gap);
   end
  end
 end
end
report=struct('minimum_sampled_rod_window_margin_mm',margin,'gimbal_to_front_cap_gap_mm',frontGap);disp(report);
assert(margin>.2,'Insufficient rod-window clearance.');
assert(frontGap>.2,'Moving gimbal body crosses the front cap.');
end
