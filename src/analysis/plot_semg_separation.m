function plot_semg_separation(T,R)
%PLOT_SEMG_SEPARATION Real forearm sEMG: voluntary drive against quiescent rest.
% One panel per gate path, every labelled segment plotted, log scale, with the
% synthetic separation printed beside the measured one.
%
% The title of every panel states that the measured ratio is an UPPER BOUND,
% and it says so because the two classes here are voluntary and REST, not
% voluntary and TREMOR. That caveat belongs on the figure rather than only in
% the caption: a figure gets pasted into a report on its own.
%
% Both arguments are optional; with none the figure is rebuilt from the saved
% CSVs, so START_PROJECT('figures') can regenerate it without re-analysing.
if nargin<1||isempty(T)
 f=project_path('results','semg_separation_real.csv');
 assert(isfile(f),'SensoSpoon:SemgSeparation',...
  'No saved result at %s. Run START_PROJECT(''realdata'') first.',f);
 T=readtable(f,'TextType','string');
end
if nargin<2||isempty(R)
 f=project_path('results','semg_separation_real_summary.csv');
 assert(isfile(f),'SensoSpoon:SemgSeparation','Missing %s.',f);
 R=table2struct(readtable(f,'TextType','string'));
end
paths=unique(T.gate_path,'stable');
% Split the voluntary class by DIRECTION. Pooling them hides the whole finding:
% the observable separates flexion from rest well and extension from rest badly,
% and a single "voluntary" column would average that away.
groups=[17 12 11];names={'rest','extension','flexion'};

f=figure('Color','w','Visible','off','Position',[100 100 480*numel(paths) 580]);
if isprop(f,'Theme');f.Theme='light';end
tiledlayout(1,numel(paths));
for pi=1:numel(paths)
 nexttile;hold on;
 g=T(T.gate_path==paths(pi),:);
 for c=1:numel(groups)
  y=g.drive_rms(g.gesture==groups(c));
  if isempty(y);continue;end
  scatter(c+.14*(rand(numel(y),1)-.5),y,34,'filled','MarkerFaceAlpha',.65);
 end
 set(gca,'YScale','log','XTick',1:numel(groups),'XTickLabel',names,'XLim',[.5 numel(groups)+.5]);
 grid on;ylabel('gate drive (RMS, calibrated units)');
 e=R(strcmp(string({R.gate_path}),paths(pi)));
 sep=min([e.separation]);syn=[e.synthetic_separation];
 if isnan(syn(1));synTxt='synthetic: not derived';else;synTxt=sprintf('synthetic %.1f:1',syn(1));end
 title({sprintf('%s path',paths(pi)),...
  sprintf('worst-case real %.1f:1 -- and it is EXTENSION that sets it',sep),...
  [synTxt '   (classes are rest, not tremor: an upper bound)']});
 hold off;
end
exportgraphics(f,project_path('figures','semg_separation_real.png'),'Resolution',160);
close(f);
fprintf('Wrote %s\n',project_path('figures','semg_separation_real.png'));
end
