function F=simulate_rcflex_liquid(S,P,saveResult)
% One-way two-mode slosh approximation driven by recorded spoon motion.
% NOT CFD: planar free surface, linear modes, empirical rim weir outflow.
% Existing plant payload is retained. Fluid reaction and mass-loss feedback
% are not fed back into the mechanical plant; use only as a screening model.
if nargin<2;P=rcflex_liquid_parameters;end
if nargin<3;saveResult=true;end
G=rcflex_liquid_geometry(S.H,P);initial=S.H.food_mass_kg/P.density_kg_m3;
assert(initial>0&&initial<G.capacity_m3,'Liquid must initially fit in bowl');
t=(0:P.step_s:S.time(end))';tout=unique([0:(1/P.output_hz):S.time(end) S.time(end)])';
F=struct('P',P,'G',G,'time',tout,'initial_mL',initial*1e6,'source_solver',S.solver,'sides',[]);
for side=1:2
 forcing=zeros(numel(S.time),3);poseO=zeros(numel(S.time),3);poseR=zeros(3,3,numel(S.time));vel=zeros(numel(S.time),3);omega=zeros(numel(S.time),3);angular=zeros(numel(S.time),2);
 for k=1:numel(S.time)
  [o,R,v,w,alpha,a]=rcflex_liquid_motion(S,k,side==2);
  ac=R'*a+cross(alpha,G.center)+cross(w,cross(w,G.center));
  effective=R'*[0;0;-9.80665]-ac;gn=-effective(3);
  if gn<=.5;error('SensoSpoon:FluidSeparation','Effective gravity <= 0.5 m/s^2; free-flight fluid model required.');end
  forcing(k,:)=[effective(1)/gn effective(2)/gn gn];
  poseO(k,:)=o';poseR(:,:,k)=R;vel(k,:)=v';omega(k,:)=w';
  angular(k,:)=[alpha(2) -alpha(1)]; %#ok<AGROW>
 end
 force=interp1(S.time,forcing,t);ang=interp1(S.time,angular,t);
 slope=zeros(2,1);rate=slope;volume=initial;out=1;level=rcflex_liquid_level(volume,slope,G);
 wet=G.b<level&G.mask;meanDepth=volume/(sum(wet,'all')*G.area);
 lengths=[max(G.X(wet))-min(G.X(wet));max(G.Y(wet))-min(G.Y(wet))];waveNumber=pi./lengths;
 Hsurf=zeros([size(G.b),numel(tout)],'single');levels=zeros(numel(tout),1);slopes=zeros(numel(tout),2);remaining=zeros(numel(tout),1);crossing=zeros(numel(tout),1);
 events=zeros(0,8);bucket=zeros(size(G.rim_ds));spill=0;maxHead=0;
 for k=1:numel(t)
  if t(k)+P.step_s/2>=tout(out)
   [level,h]=rcflex_liquid_level(volume,slope,G);Hsurf(:,:,out)=single(h);levels(out)=level;slopes(out,:)=slope';remaining(out)=volume*1e6;crossing(out)=maxHead;
   % Parcels represent the actual integrated flux over each rim segment.
   idx=find(bucket>0);src=min(numel(S.time),max(1,round(t(k)/(S.time(2)-S.time(1)))+1));R=poseR(:,:,src);o=poseO(src,:)';
   for j=idx'
    r=[G.rim_xy(j,:)';G.center(3)];normal=G.rim_local(j,:)';normal=normal/max(norm(normal),eps);
    speed=sqrt(2*force(k,3)*max(maxHead,0));relative=[normal*speed;0];
    p=o+R*r;vv=vel(src,:)'+R*(cross(omega(src,:)',r)+relative);
    events(end+1,:)=[t(k),p',vv',bucket(j)]; %#ok<AGROW>
   end
   bucket(:)=0;maxHead=0;out=out+1;
   if out>numel(tout);break;end
  end
  % Surface slope includes inertial lag against body angular acceleration.
  wn=sqrt(force(k,3)*waveNumber.*tanh(waveNumber*meanDepth));
  accel=wn.^2.*(force(k,1:2)'-slope)-2*P.damping_ratio*wn.*rate+ang(k,:)';
  rate=rate+P.step_s*accel;slope=slope+P.step_s*rate;
  if norm(slope)>.5;error('SensoSpoon:FluidNonlinear','Surface slope exceeds reduced-model range');end
  [level,h]=rcflex_liquid_level(volume,slope,G);
  head=max(0,level+G.rim_local*slope);maxHead=max(maxHead,max(head));
  dv=(2/3)*P.weir_coefficient*sqrt(2*force(k,3))*head.^1.5.*G.rim_ds*P.step_s;
  if sum(dv)>volume;dv=dv*(volume/sum(dv));end
  volume=volume-sum(dv);spill=spill+sum(dv);bucket=bucket+dv;
 end
 assert(abs(volume+spill-initial)<1e-12,'Liquid mass balance failed');
 F.sides(side).h=Hsurf;F.sides(side).level=levels;F.sides(side).slope=slopes;F.sides(side).remaining_mL=remaining;
 F.sides(side).spilled_mL=initial*1e6-remaining;F.sides(side).head_m=crossing;F.sides(side).events=events;
 F.sides(side).frequency_Hz=sqrt(9.80665*waveNumber.*tanh(waveNumber*meanDepth))/(2*pi);
 F.sides(side).mass_balance_error_mL=abs(volume+spill-initial)*1e6;
 fprintf('Liquid side %d: %.3f mL initial, %.6f mL spilled; modes %.2f/%.2f Hz\n',side,initial*1e6,spill*1e6,F.sides(side).frequency_Hz);
end
if saveResult;save(project_path('results','liquid_comparison.mat'),'F','S');end
end
