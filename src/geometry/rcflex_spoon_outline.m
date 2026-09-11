function w=rcflex_spoon_outline(s,H)
% Half-width of the supplied dimensioned plan view, mm from front tip.
% Elliptical nose and a tangent-continuous shoulder into the parameterized neck.
c=H.bowl_widest_station_mm;w=zeros(size(s));front=s<=c;
w(front)=20*sqrt(max(0,1-((s(front)-c)/c).^2));
middle=s>c&s<=56;w(middle)=20*sqrt(1-((s(middle)-c)/26).^2);
rear=s>56;u=(s(rear)-56)/6;v=20*sqrt(1-((56-c)/26)^2);
slope=-20*(56-c)/(26^2*sqrt(1-((56-c)/26)^2));
w(rear)=(2*u.^3-3*u.^2+1)*v+(u.^3-2*u.^2+u)*6*slope+(-2*u.^3+3*u.^2)*(H.bowl_connection_width_mm/2);
end
