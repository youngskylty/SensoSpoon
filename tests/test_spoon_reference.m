function test_spoon_reference()
H=rcflex_parameters();d=load(project_path('assets','assembly.mat'));p=d.A(strcmp({d.A.name},'Spoon_bowl'));
extent=max(p.V)-min(p.V);assert(max(abs(extent-H.bowl_mm))<1e-6);
assert(abs(2*rcflex_spoon_outline(H.bowl_front_station_mm,H)-28)<1e-10);
assert(abs(2*rcflex_spoon_outline(62,H)-H.bowl_connection_width_mm)<1e-10);
s=unique([linspace(0,62,10001) 37.2]);w=rcflex_spoon_outline(s,H);[~,idx]=max(w);assert(abs(s(idx)-37.2)<1e-8);
% Actual mesh contains bottom pair and rim pair with the specified separation.
z=p.V(:,3)-H.bowl_rim_z_mm;assert(abs(min(z)+H.bowl_mm(3))<1e-6 && any(abs(z+H.bowl_mm(3)-H.bowl_bottom_thickness_mm)<1e-6));
assert(any(abs(z+1.2)<1e-8)&&any(abs(z)<1e-8));
fprintf('Reference spoon dimensions passed; front-width station inferred at %.3f mm.\n',H.bowl_front_station_mm);
end
