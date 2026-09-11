function [com,I,volume]=rcflex_mesh_mass_properties(V,F,mass)
% Uniform closed polyhedron, exact tetrahedral volume moments. V in metres.
% The supplied mass may be measured; geometry determines its distribution.
a=V(F(:,1),:);b=V(F(:,2),:);c=V(F(:,3),:);
v=dot(a,cross(b,c,2),2)/6;volume=sum(v);
assert(abs(volume)>1e-15,'Degenerate closed mesh.');
com=sum(v.*(a+b+c),1)/(4*volume);Q=zeros(3);
s=a+b+c;
for j=1:3
 for k=1:3
  Q(j,k)=sum(v.*(a(:,j).*a(:,k)+b(:,j).*b(:,k)+c(:,j).*c(:,k)+s(:,j).*s(:,k)))/20;
 end
end
C=Q/volume-com'*com;I=mass*(trace(C)*eye(3)-C);I=(I+I')/2;
assert(min(eig(I))>=-1e-12,'Invalid solid inertia.');volume=abs(volume);
end
