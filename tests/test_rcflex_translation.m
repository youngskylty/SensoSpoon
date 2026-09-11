function test_rcflex_translation()
H=rcflex_parameters();B=rcflex_body_properties(H);assert(numel(B)==6);
x=zeros(8,1);u=zeros(17,1);
d0=rcflex_physics(x,u,H,B,'voltage');u(17)=2;
d1=rcflex_physics(x,u,H,B,'voltage');
assert(norm(d1(3:4)-d0(3:4))>1,'Base translation must excite the mechanism.');
base=zeros(15,1);plant=zeros(26,1);y0=rcflex_sensor_truth(plant,base,H);
base(13:15)=[1;2;3];y1=rcflex_sensor_truth(plant,base,H);
assert(max(abs(y1([4:6 10:12])-y0([4:6 10:12])-[1;2;3;1;2;3]))<1e-8);
t=(0:.001:1)';base=rcflex_tremor_input(t,6,3,1);q=-base(:,[2 1]);
m=rcflex_world_metrics(q,base,H);assert(m.attitude_rms_deg<1e-5 && m.position_rms_mm>.1);
fprintf('Translation dynamics, both IMUs, and attitude-versus-position checks passed.\n');
end
