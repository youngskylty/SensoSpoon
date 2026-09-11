function G=rcflex_liquid_geometry(H,P)
% Exact analytic inner bowl profile used by bowlmesh, sampled for quadrature.
d=P.grid_mm;s=(d/2:d:H.bowl_mm(1)-d/2)';y=-H.bowl_mm(2)/2+d/2:d:H.bowl_mm(2)/2-d/2;
[S,Y]=ndgrid(s,y);width=rcflex_spoon_outline(S,H);inside=abs(Y)<width;
c=H.bowl_widest_station_mm;span=c*ones(size(S));span(S>c)=H.bowl_mm(1)-c;
t=abs((S-c)./span);g=1-t.^2;back=S>c;g(back)=1-3*t(back).^2+2*t(back).^3;
b=-(H.bowl_mm(3)-H.bowl_bottom_thickness_mm)*max(0,g.*(1-(Y./max(width,eps)).^2));b(~inside)=0;
G.X=(S+H.bowl_tip_x_mm)/1000;G.Y=Y/1000;G.b=b/1000;
G.mask=inside;G.area=(d/1000)^2;G.center=[H.bowl_measurement_x_mm;0;H.bowl_rim_z_mm]/1000;
G.x=G.X-G.center(1);G.y=G.Y;G.capacity_m3=sum(-G.b(inside))*G.area;
% Actual outline, closed at the neck. Segment-centre overflow quadrature.
u=linspace(0,H.bowl_mm(1),180)';w=rcflex_spoon_outline(u,H);
v=[u w;flipud(u) -flipud(w)];v(:,1)=v(:,1)+H.bowl_tip_x_mm;
a=v/1000;bb=circshift(a,-1,1);G.rim_xy=(a+bb)/2;G.rim_ds=vecnorm(bb-a,2,2);
G.rim_local=G.rim_xy-G.center(1:2)';
end
