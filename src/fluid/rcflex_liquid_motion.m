function [o,R,v,omega,alpha,acc]=rcflex_liquid_motion(S,k,controlled)
% Bowl-frame origin is the gimbal centre; compute motion from solved q/qd/qdd.
b=S.base(k,:)';q=zeros(2,1);qd=q;qdd=q;
if controlled;q=S.Y(k,1:2)';qd=S.Y(k,3:4)';qdd=S.Y(k,19:20)';end
z=[b(1:2);q];zd=[b(3:4);qd];zdd=[b(5:6);qdd];dt=1e-5;
[o,R,W]=pose(z,S.H);[op,~,Wp]=pose(z+dt*zd+.5*dt^2*zdd,S.H);[om,~,Wm]=pose(z-dt*zd+.5*dt^2*zdd,S.H);
v=(op-om)/(2*dt)+b(10:12);acc=(op-2*o+om)/dt^2+b(13:15);o=o+b(7:9);
omega=R'*(W*zd);alpha=R'*((Wp-Wm)/(2*dt)*zd+W*zdd);
end
function [o,R,W]=pose(z,H)
X=rx(z(1));Hb=X*ry(z(2));P=ry(z(3));R=Hb*P*rx(z(4));
o=Hb*[-H.grip_x_mm/1000;0;0];
W=[[1;0;0] X*[0;1;0] Hb*[0;1;0] Hb*P*[1;0;0]];
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
