function test_rcflex_liquid()
H=rcflex_parameters;P=rcflex_liquid_parameters;G=rcflex_liquid_geometry(H,P);
[level,h]=rcflex_liquid_level(5e-6,[.03;-.02],G);
assert(abs(sum(h,'all')*G.area-5e-6)<1e-12,'Volume reconstruction');
assert(level<0&&all(h(:)>=0));
S.H=H;S.time=(0:.001:.5)';S.Y=zeros(numel(S.time),26);S.base=zeros(numel(S.time),15);S.solver='test_fixture';
F=simulate_rcflex_liquid(S,P,false);
assert(max(abs(F.sides(1).slope),[],'all')<1e-12,'Still water should not slosh');
assert(F.sides(1).spilled_mL(end)<1e-9,'Still water should not spill');
assert(isequal(F.sides(1).h,F.sides(2).h),'Identical motion must produce identical liquid');
% Independent static tilted fixture: near-full liquid must spill naturally.
S.H.food_mass_kg=.008;S.time=(0:.001:2)';S.Y=zeros(numel(S.time),26);S.base=zeros(numel(S.time),15);S.base(:,2)=deg2rad(8);
F=simulate_rcflex_liquid(S,P,false);
for j=1:2
 assert(F.sides(j).spilled_mL(end)>.01,'Tilted near-full fixture should spill');
 assert(all(diff(F.sides(j).remaining_mL)<=1e-9),'Liquid volume cannot increase');
 assert(F.sides(j).mass_balance_error_mL<1e-6,'Mass ledger');
 assert(abs(sum(F.sides(j).events(:,8))*1e6-F.sides(j).spilled_mL(end))<1e-6,'Droplets must match computed outflow');
end
fprintf('Liquid checks passed: volume, rest state, identical forcing, tilt overflow, and parcel mass balance.\n');
end
