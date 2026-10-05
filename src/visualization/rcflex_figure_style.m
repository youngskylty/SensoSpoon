function S=rcflex_figure_style(width)
%RCFLEX_FIGURE_STYLE The single place every IEEE report figure's style is set.
%
% This is THE SWITCH for publication styling. Change a font size, a column
% width, an export resolution or an arm colour here and every report figure
% follows; no plotting function carries its own geometry. The arm palette is
% keyed to the ARM NAME rather than to a position in a list, so dropping an arm
% from a comparison never repaints the arms that remain.
%
%   width : 'single' (3.5 in, one IEEE column) or 'double' (7.16 in, page width)
%
% Report figures are written to output/figures/report/ and are held to IEEE
% conventions: no sentence-style titles, no explanatory prose inside the axes,
% no "lower is better" asides. All of that belongs in the LaTeX caption, which
% is read together with the figure. The diagnostic figures in output/figures/
% are unaffected and keep their annotations, because they are read alone.
if nargin<1||isempty(width);width='double';end
switch lower(char(width))
 case 'single';S.width_in=3.5;
 case 'double';S.width_in=7.16;
 otherwise;error('SensoSpoon:FigureStyle','Unknown width: %s (use ''single'' or ''double'')',width);
end
S.width=width;
% Point sizes chosen so that text is >=7 pt once the figure is placed at its
% true column width, which is the floor most IEEE templates enforce.
S.font_tick=7;S.font_label=8;S.font_legend=7.5;S.font_panel=9;
S.res=600;                        % dpi; IEEE asks 300 minimum for line art
S.ink=[.10 .10 .10];              % axis text
S.grid=[.80 .80 .79];
S.ref=[.35 .35 .34];              % reference/target lines
% Arm palette, validated on a light surface. Figures pin Theme='light' so the
% export never inherits a dark desktop theme.
S.arm_key=["simple_pid","ekf_cv_legacy","ekf_cv","ekf_cv_osc2","ekf_cv_qfix","ekf_cv_semg"];
S.arm_rgb=[ 42 120 214; 235 104  52;  27 175 122; 237 161   0; 217  95   2; 148  60 190]/255;
S.outdir=project_path('figures','report');
if ~isfolder(S.outdir);mkdir(S.outdir);end
end
