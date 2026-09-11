function [m,E]=rcflex_control_metrics(S)
% Ground truth is used ONLY here for scoring, never for feedback.
n=numel(S.time);E=zeros(n,6);baseline=zeros(n,6);H=S.H;
v=[H.bowl_measurement_x_mm;0;H.bowl_rim_z_mm-H.bowl_mm(3)/2]/1000;g=[-H.grip_x_mm;0;0]/1000;
for k=1:n
 Rh=rx(S.base(k,1))*ry(S.base(k,2));Rd=rx(S.intent(k,1))*ry(S.intent(k,2));Rs=Rh*ry(S.Y(k,1))*rx(S.Y(k,2));
 E(k,1:3)=rotation_vector(Rd'*Rs)';baseline(k,1:3)=rotation_vector(Rd'*Rh)';
 desired=Rd*(g+v);E(k,4:6)=(S.base(k,7:9)'+Rh*g+Rs*v-desired)';
 baseline(k,4:6)=(S.base(k,7:9)'+Rh*(g+v)-desired)';
end
ix=S.time>=1.5;assert(any(ix),'Need at least 1.5 seconds for steady scoring');
rmsv=@(a)sqrt(mean(sum(a.^2,2)));
m.attitude_rms_deg=rad2deg(rmsv(E(ix,1:3)));m.baseline_rms_deg=rad2deg(rmsv(baseline(ix,1:3)));
m.suppression_percent=NaN;if m.baseline_rms_deg>1e-7;m.suppression_percent=100*(1-m.attitude_rms_deg/m.baseline_rms_deg);end
m.roll_rms_deg=rad2deg(sqrt(mean(E(ix,1).^2)));m.pitch_rms_deg=rad2deg(sqrt(mean(E(ix,2).^2)));
m.attitude_peak_deg=rad2deg(max(vecnorm(E(:,1:3),2,2)));m.startup_rms_deg=rad2deg(rmsv(E(S.time<1.5,1:3)));
m.position_rms_mm=1000*rmsv(E(ix,4:6));m.baseline_position_rms_mm=1000*rmsv(baseline(ix,4:6));
m.current_peak_A=max(abs(S.Y(:,5:6)),[],'all');m.current_rms_A=max(sqrt(mean(S.Y(ix,5:6).^2,1)));
m.voltage_peak_V=max(abs(S.Y(:,11:12)),[],'all');m.stroke_peak_mm=1000*max(abs(S.Y(:,15:16)),[],'all');m.joint_peak_deg=rad2deg(max(abs(S.Y(:,1:2)),[],'all'));
m.power_mean_W=max(mean(S.Y(ix,21:22),1));m.temperature_rise_K=max(S.Y(:,7:8),[],'all');m.allocation_saturation_percent=100*mean(S.control(ix,15)>0);
m.side_load_peak_N=0;
for k=1:n
 L=rcflex_linkage(S.Y(k,1:2)',H);d=L.slider-L.attach;ratio=vecnorm(d(2:3,:))./abs(d(1,:));
 m.side_load_peak_N=max(m.side_load_peak_N,max(ratio.*abs(S.Y(k,13:14))));
end
m.current_utilization_percent=100*m.current_peak_A/H.current_limit_A;
m.stroke_utilization_percent=100*m.stroke_peak_mm/H.vca_stroke_mm;
end
function v=rotation_vector(R)
a=acos(max(-1,min(1,(trace(R)-1)/2)));v=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/2;
if a>1e-7;v=v*a/sin(a);end
end
function R=rx(a);R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];end
function R=ry(a);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
