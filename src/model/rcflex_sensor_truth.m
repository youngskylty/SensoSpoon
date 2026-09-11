function y=rcflex_sensor_truth(plant,base,H)
% Two body-mounted IMUs. Outputs [handle gyro;handle accel;spoon gyro;spoon accel].
z=[base(1:2);plant(1:2)];zd=[base(3:4);plant(3:4)];zdd=[base(5:6);plant(19:20)];
dt=1e-4;y=zeros(12,1);
for j=1:2
    [pos,R,W]=pose(z,j,H);pp=pose(z+dt*zd+.5*dt^2*zdd,j,H);pm=pose(z-dt*zd+.5*dt^2*zdd,j,H);
    accel=(pp-2*pos+pm)/dt^2;
    if numel(base)>=15;accel=accel+base(13:15);end
    y((j-1)*6+(1:3))=R'*(W*zd);
    y((j-1)*6+(4:6))=R'*(accel+[0;0;9.80665]);
end
end
function [p,R,W]=pose(z,j,H)
X=rx(z(1));Hb=X*ry(z(2));P=ry(z(3));T=rx(z(4));
if j==1
    R=Hb;p=Hb*([.117;.004;.013]-[H.grip_x_mm/1000;0;0]);
    W=[[1;0;0] X*[0;1;0] zeros(3,2)];
else
    R=Hb*P*T;p=Hb*[-H.grip_x_mm/1000;0;0]+R*[-.010;0;.006];
    W=[[1;0;0] X*[0;1;0] Hb*[0;1;0] Hb*P*[1;0;0]];
end
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
