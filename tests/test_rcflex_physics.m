function report=test_rcflex_physics()
% Independent physical checks; no suppression target or controller tuning.
H=rcflex_parameters();B=rcflex_body_properties(H);closure=0;jerror=0;energyError=0;maxCond=0;maxSide=0;
for p=linspace(-15,15,7)
 for r=linspace(-15,15,7)
    q=deg2rad([p;r]);L=rcflex_linkage(q,H);
    closure=max([closure abs(vecnorm(L.input-L.slider)-H.link.input_length) abs(vecnorm(L.output-L.attach)-H.link.output_length)]);
    assert(all(abs(L.s)<H.vca_stroke_mm/1000),'Stroke cannot cover the proposed joint range.');
    assert(cond(L.J)<20,'Transmission near singularity.');
    maxCond=max(maxCond,cond(L.J));
    if isfield(H.link,'mode')&&strcmp(H.link.mode,'direct')
        d=L.slider-L.attach;
        side=H.current_limit_A*H.vca_force_constant_N_A*sqrt(sum(d(2:3,:).^2,1))./abs(d(1,:));
        maxSide=max(maxSide,max(side));
        assert(max(side)<H.shaft_side_load_N,'Direct rod exceeds shaft side-load rating.');
    end
    for k=1:2
        e=zeros(2,1);e(k)=1e-6;lp=rcflex_linkage(q+e,H);lm=rcflex_linkage(q-e,H);
        jerror=max(jerror,max(abs((lp.s-lm.s)/2e-6-L.J(:,k))));
        lc=rcflex_linkage(q+1i*1e-20*e/1e-6,H);
        assert(max(abs(imag(lc.J)/1e-20-(lp.J-lm.J)/2e-6),[],'all')<1e-7,'Mass-bias Jacobian derivative failed.');
    end
    [M,~,~]=rcflex_mass_bias(q,[0;0],[0;0],[0;0],[0;0],H,B);assert(min(eig(M))>0);
 end
end
assert(closure<1e-12 && jerror<1e-8,'Linkage closure/Jacobian failed.');
% Mechanical-electrical energy balance at a state away from end stops.
x=[.03;-.02;.4;-.2;.2;-.15;0;0];u=[zeros(6,1);1;-.5];
[dx,Y]=rcflex_physics(x,u,H,B,'voltage');epsT=1e-7;
[~,yp]=rcflex_physics(x+epsT*dx,u,H,B,'voltage');[~,ym]=rcflex_physics(x-epsT*dx,u,H,B,'voltage');
dEnergy=(sum(yp(23:24))+.5*H.coil_L_H*sum(yp(5:6).^2)-sum(ym(23:24))-.5*H.coil_L_H*sum(ym(5:6).^2))/(2*epsT);
sd=Y(25:26);loss=(H.coil_R_ohm+H.driver_R_ohm)*sum(x(5:6).^2)+H.flexure_damping_Nms_rad*sum(x(3:4).^2)+sum((H.guide_friction_N*tanh(sd/.001)+H.guide_viscous_Ns_m*sd).*sd);
expected=dot(Y(11:12),x(5:6))-loss;energyError=abs(dEnergy-expected);
assert(energyError<1e-5,'Energy balance error %.6g W.',energyError);
% Turning off force constant must remove direct current-to-acceleration effect.
h0=H;h0.vca_force_constant_N_A=0;x0=zeros(8,1);x1=x0;x1(5:6)=.4;
d0=rcflex_physics(x0,zeros(8,1),h0,B,'voltage');d1=rcflex_physics(x1,zeros(8,1),h0,B,'voltage');
assert(max(abs(d0(3:4)-d1(3:4)))<1e-10);
d2=rcflex_physics(x1,zeros(8,1),H,B,'voltage');assert(norm(d2(3:4)-d0(3:4))>1);
% Actual dynamic passive decay and independent solver tolerance convergence.
opts=odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',.001);
t=(0:.001:.2)';fun=@(~,x)rcflex_physics(x,zeros(8,1),H,B,'off');
[~,a]=ode15s(fun,t,x,opts);[~,b]=ode15s(fun,t,x,odeset(opts,'RelTol',1e-8,'AbsTol',1e-10,'MaxStep',.0005));
convergence=max(abs(a(:,1:2)-b(:,1:2)),[],'all');assert(convergence<1e-4);
en=zeros(numel(t),1);
for k=1:numel(t);[~,y]=rcflex_physics(b(k,:)',zeros(8,1),H,B,'off');en(k)=sum(y(23:24))+.5*H.coil_L_H*sum(y(5:6).^2);end
assert(en(end)<en(1),'Passive mechanism gains energy.');
report=struct('rod_closure_error_m',closure,'jacobian_error',jerror,'energy_balance_error_W',energyError,'solver_angle_difference_rad',convergence,'max_transmission_condition',maxCond,'max_direct_side_load_N',maxSide);
disp(report);fid=fopen(project_path('results','analytic_validation.txt'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
end
