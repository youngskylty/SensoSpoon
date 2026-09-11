function [M,bias,E]=rcflex_mass_bias(q,v,hand,hd,hdd,H,B,translation)
if nargin<8;translation=zeros(9,1);end % world position, velocity, acceleration
% Reduced rigid-body Newton-Euler dynamics, including moving-base excitation.
% z=[hand roll, hand pitch, correction pitch, correction roll].
% Both moving coil/shaft masses and both rods have inertia. Rod length remains
% rigid, and spherical joints are ideal. Guide friction is parameterized separately.
z=[hand(:);q(:)];zd=[hd(:);v(:)];M=zeros(2);bias=zeros(2,1);E.kinetic=0;E.gravity=0;
for n=1:numel(B)
    [R,x,Jv,Jw]=kin(z,B(n).com,n,H);
    [~,~,jv,jw]=kin(z+1i*1e-20*zd,B(n).com,n,H);
    Jvd=imag(jv)/1e-20;Jwd=imag(jw)/1e-20;
    W=R*B(n).inertia*R.';omega=Jw*zd;vel=Jv*zd+translation(4:6);
    av=Jv(:,1:2)*hdd+Jvd*zd+translation(7:9);aw=Jw(:,1:2)*hdd+Jwd*zd;
    M=M+B(n).mass*(Jv(:,3:4)'*Jv(:,3:4))+Jw(:,3:4)'*W*Jw(:,3:4);
    bias=bias+Jv(:,3:4)'*(B(n).mass*(av+[0;0;9.80665]))+Jw(:,3:4)'*(W*aw+cross(omega,W*omega));
    E.kinetic=E.kinetic+.5*B(n).mass*(vel'*vel)+.5*omega'*W*omega;
    E.gravity=E.gravity+B(n).mass*9.80665*(x(3)+translation(3));
end
end
function [R,x,Jv,Jw]=kin(z,c,n,H)
a=z(1);b=z(2);p=z(3);r=z(4);
if n>4
    L=rcflex_linkage([p;r],H);j=n-4;X=rx(a);Y=ry(b);Hb=X*Y;
    d=L.slider(:,j)-L.attach(:,j);u=d/H.link.input_length(j);
    w=[0;-u(3);u(2)];K=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
    R=Hb*(eye(3)+K+K*K/(1+u(1)));
    center=(L.slider(:,j)+L.attach(:,j))/2-[H.grip_x_mm/1000;0;0];x=Hb*center;
    dx=[0 0 0;0 -sin(a) -cos(a);0 cos(a) -sin(a)];
    dy=[-sin(b) 0 cos(b);0 0 0;-cos(b) 0 -sin(b)];
    Jv=[dx*Y*center X*dy*center zeros(3,2)];
    Jw=[[1;0;0] X*[0;1;0] zeros(3,2)];
    for k=1:2
        sliderq=[L.J(j,k);0;0];aq=L.Aq(:,j,k);
        Jv(:,k+2)=Hb*(sliderq+aq)/2;
        Jw(:,k+2)=Hb*cross(u,(sliderq-aq)/H.link.input_length(j));
    end
    return
end
if n>2
    L=rcflex_linkage([p;r],H);j=n-2;X=rx(a);Y=ry(b);R=X*Y;
    c=c+[L.s(j);0;0];u=c-[H.grip_x_mm/1000;0;0];x=R*u;
    dx=[0 0 0;0 -sin(a) -cos(a);0 cos(a) -sin(a)];
    dy=[-sin(b) 0 cos(b);0 0 0;-cos(b) 0 -sin(b)];
    Jv=[dx*Y*u X*dy*u R*[L.J(j,1);0;0] R*[L.J(j,2);0;0]];
    Jw=[[1;0;0] X*[0;1;0] zeros(3,2)];return
end
X=rx(a);Y=ry(b);P=ry(p);T=rx(r);if n==1;T=eye(3);end
Hb=X*Y;C=P*T;R=Hb*C;u=[-H.grip_x_mm/1000;0;0]+C*c;x=Hb*u;
dx=[0 0 0;0 -sin(a) -cos(a);0 cos(a) -sin(a)];
dy=[-sin(b) 0 cos(b);0 0 0;-cos(b) 0 -sin(b)];
dp=[-sin(p) 0 cos(p);0 0 0;-cos(p) 0 -sin(p)];
dr=[0 0 0;0 -sin(r) -cos(r);0 cos(r) -sin(r)];
Jv=[dx*Y*u X*dy*u Hb*dp*T*c Hb*P*dr*c];
Jw=[[1;0;0] X*[0;1;0] Hb*[0;1;0] Hb*P*[1;0;0]];
if n==1;Jv(:,4)=0;Jw(:,4)=0;end
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
