function T=test_rcflex_hardware()
% Core component acceptance tests, independent of a stabilisation algorithm.
H=rcflex_parameters();B=rcflex_body_properties(H);H.battery_R_ohm=0;
opt=odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',1e-5);t=(0:1e-5:.003)';
R=H.coil_R_ohm+H.driver_R_ohm;
[~,i]=ode15s(@(~,i)locked(i,[1;0],H,B),t,[0;0],opt);
exact=(1-exp(-t*R/H.coil_L_H))/R;
err=max(abs(i(:,1)-exact));assert(err<1e-7,'Locked-rotor RL step error.');
[~,neg]=ode15s(@(~,i)locked(i,[-1;0],H,B),t,[0;0],opt);
polarity=max(abs(i(:,1)+neg(:,1)));assert(polarity<1e-7);
x=zeros(8,1);x(5)=.3;h=H;h.current_limit_A=.3;
d=rcflex_physics(x,[zeros(6,1);100;0],h,B,'voltage');assert(abs(d(5))<1e-8,'Current chopping fails.');
[~,y]=rcflex_physics(zeros(8,1),[zeros(6,1);100;-100],H,B,'voltage');assert(max(abs(y(11:12)))<=H.bus_voltage_V);
assert(d(7)>0,'Copper loss does not heat coil.');
% Moving load is physical: increasing payload changes free dynamics.
tt=(0:.001:.15)';op=odeset('RelTol',1e-7,'AbsTol',1e-9,'MaxStep',.001);
u=[zeros(6,1);1;1];[~,light]=ode15s(@(~,x)rcflex_physics(x,u,H,B,'voltage'),tt,zeros(8,1),op);
h=H;h.food_mass_kg=.015;bh=rcflex_body_properties(h);
[~,heavy]=ode15s(@(~,x)rcflex_physics(x,u,h,bh,'voltage'),tt,zeros(8,1),op);
loadEffect=max(abs(light(:,1:2)-heavy(:,1:2)),[],'all');assert(loadEffect>1e-4);
sensor=rcflex_sensor_truth(zeros(26,1),zeros(6,1),H);
assert(max(abs(sensor([6 12])-9.80665))<1e-6,'Accelerometer gravity/sign error.');
T=table(err,polarity,loadEffect,'VariableNames',{'LockedRotorStepError_A','ReversedPolarityError_A','PayloadResponseDifference_rad'});
writetable(T,project_path('results','analytic_hardware_validation.csv'));disp(T);
end
function di=locked(i,v,H,B)
x=zeros(8,1);x(5:6)=i;d=rcflex_physics(x,[zeros(6,1);v],H,B,'voltage');di=d(5:6);
end
