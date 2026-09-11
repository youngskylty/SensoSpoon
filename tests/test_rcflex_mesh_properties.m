function test_rcflex_mesh_properties()
d=[.04 .02 .01];center=[.1 -.05 .03];m=.12;
V=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;-1 -1 1;1 -1 1;1 1 1;-1 1 1].*d/2+center;
F=[1 3 2;1 4 3;5 6 7;5 7 8;1 2 6;1 6 5;2 3 7;2 7 6;3 4 8;3 8 7;4 1 5;4 5 8];
[c,I,v]=rcflex_mesh_mass_properties(V,F,m);
expected=m*diag([d(2)^2+d(3)^2 d(1)^2+d(3)^2 d(1)^2+d(2)^2])/12;
assert(norm(c-center)<1e-12 && norm(I-expected)<1e-12 && abs(v-prod(d))<1e-14);
s=load(project_path('assets','assembly.mat'),'A');
for k=1:numel(s.A)
    p=s.A(k);f=p.F;e=sort([f(:,[1 2]);f(:,[2 3]);f(:,[3 1])],2);
    [~,~,g]=unique(e,'rows');assert(all(accumarray(g,1)==2),'Non-closed mesh: %s',p.name);
end
fprintf('Mesh mass moments match a translated analytical cuboid; all assembly meshes are closed.\n');
end
