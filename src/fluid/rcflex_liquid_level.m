function [level,h]=rcflex_liquid_level(volume,slope,G)
% Solve the free-surface height intercept from retained volume (no mass creation).
shape=slope(1)*G.x+slope(2)*G.y;b=G.b;
lo=min(b(G.mask)-shape(G.mask))-.01;hi=max(b(G.mask)-shape(G.mask))+.03;
for j=1:35
 level=(lo+hi)/2;h=max(0,level+shape-b);v=sum(h(G.mask))*G.area;
 if v>volume;hi=level;else;lo=level;end
end
level=(lo+hi)/2;h=max(0,level+shape-b);h(~G.mask)=0;
end
