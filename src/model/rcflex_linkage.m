function L=rcflex_linkage(q,H)
% Exact direct-rod constraints and Jacobian; q=[pitch;roll].
p=q(1);r=q(2);P=[cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)];T=[1 0 0;0 cos(r) -sin(r);0 sin(r) cos(r)];
dP=[-sin(p) 0 cos(p);0 0 0;-cos(p) 0 -sin(p)];dT=[0 0 0;0 -sin(r) -cos(r);0 cos(r) -sin(r)];
L.attach=P*T*H.link.attachment;
L.Aq=cat(3,dP*T*H.link.attachment,P*dT*H.link.attachment);
gy=H.link.pivot(2,:)-L.attach(2,:);gz=H.link.slider_z-L.attach(3,:);
root=sqrt(H.link.input_length.^2-gy.^2-gz.^2);
if isreal(q)&&any(real(root)<=0|imag(root)~=0);error('RCFlex:DirectReach','Direct rod cannot reach.');end
L.s=(L.attach(1,:)+root-H.link.slider_x).';L.J=zeros(2);
for k=1:2
    L.J(:,k)=(L.Aq(1,:,k)+(gy.*L.Aq(2,:,k)+gz.*L.Aq(3,:,k))./root).';
end
L.slider=[H.link.slider_x+L.s.';H.link.pivot(2,:);H.link.slider_z*[1 1]];
L.input=L.attach;L.output=L.attach;L.beta=zeros(2,1);L.B=zeros(2);
L.Iq=L.Aq;L.Oq=L.Aq;
end
