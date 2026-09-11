function B=rcflex_body_properties(H)
% Geometry-linked masses and closed-mesh volume moments for moving custom parts.
S=load(project_path('assets','assembly.mat'),'A');A=S.A;
for j=1:2
    if j==1;name='PitchCarrier';else;name='SpoonCarrier';end
    parts=A(strcmp({A.body},name));m=[parts.mass]';c=zeros(numel(parts),3);I=zeros(3);local=zeros(3,3,numel(parts)+1);
    for k=1:numel(parts);[c(k,:),local(:,:,k)]=rcflex_mesh_mass_properties(parts(k).V/1000,parts(k).F,m(k));end
    if j==2
        m=[m;H.food_mass_kg];c=[c;H.bowl_measurement_x_mm/1000 0 (H.bowl_rim_z_mm-H.bowl_mm(3)/2)/1000];
        d=[.025 .02 .005];local(:,:,numel(m))=H.food_mass_kg*diag([d(2)^2+d(3)^2 d(1)^2+d(3)^2 d(1)^2+d(2)^2])/12;
    end
    com=sum(m.*c,1)/sum(m);
    for k=1:numel(m)
        v=c(k,:)-com;
        I=I+local(:,:,k)+m(k)*(dot(v,v)*eye(3)-v'*v);
    end
    B(j).mass=sum(m);B(j).com=com';B(j).inertia=I; %#ok<AGROW>
end
for j=1:2
    B(j+2).mass=H.coil_mass_kg;
    B(j+2).com=[(H.vca_centers_mm(j,1)-6.5)/1000;H.vca_centers_mm(j,2)/1000;H.vca_centers_mm(j,3)/1000];
    B(j+2).inertia=diag(H.coil_mass_kg*[.006^2/2,(3*.006^2+.012^2)/12,(3*.006^2+.012^2)/12]);
end
% Rear actuator's rigid coaxial shaft extension; mass and parallel-axis terms.
m=pi*.0015^2*.032*7800;c=[.046;H.vca_centers_mm(2,2:3)'/1000];
a=B(4).mass;b=B(4).com;cc=(a*b+m*c)/(a+m);
I=m*diag([.0015^2/2,(3*.0015^2+.032^2)/12,(3*.0015^2+.032^2)/12]);
u=b-cc;v=c-cc;
B(4).inertia=B(4).inertia+I+a*((u'*u)*eye(3)-u*u')+m*((v'*v)*eye(3)-v*v');
B(4).mass=a+m;B(4).com=cc;
% Direct rods have real translational and transverse rotational inertia.
for j=1:2
    part=A(strcmp({A.name},['Pushrod_' char('A'+j-1)]));
    m=part.mass;r=H.pushrod_diameter_mm/2000;len=H.link.input_length(j);
    B(j+4).mass=m;B(j+4).com=zeros(3,1);
    B(j+4).inertia=diag(m*[r*r/2,(3*r*r+len*len)/12,(3*r*r+len*len)/12]);
end
end
