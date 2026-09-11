function out=rcflex_world_metrics(q,base,H)
% Independent world-frame attitude and bowl-centre position metrics.
n=size(q,1);pos=zeros(n,3);tilt=zeros(n,1);attitude=zeros(n,1);
for k=1:n
    Hb=rx(base(k,1))*ry(base(k,2));R=Hb*ry(q(k,1))*rx(q(k,2));
    pos(k,:)=(Hb*[-H.grip_x_mm/1000;0;0]+R*[H.bowl_measurement_x_mm/1000;0;(H.bowl_rim_z_mm-H.bowl_mm(3)/2)/1000]).';
    if size(base,2)>=15;pos(k,:)=pos(k,:)+base(k,7:9);end
    tilt(k)=acos(max(-1,min(1,R(3,3))));
    attitude(k)=acos(max(-1,min(1,(trace(R)-1)/2)));
end
out.position_m=pos;out.tilt_rad=tilt;out.attitude_rad=attitude;
out.position_rms_mm=1000*sqrt(mean(sum((pos-mean(pos,1)).^2,2)));
out.tilt_rms_deg=rad2deg(sqrt(mean(tilt.^2)));
out.attitude_rms_deg=rad2deg(sqrt(mean(attitude.^2)));
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
