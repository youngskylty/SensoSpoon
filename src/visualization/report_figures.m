function report_figures()
%REPORT_FIGURES Regenerate every IEEE report figure from the saved CSVs.
%
% The report figure set is deliberately small. Each one earns a column-inch by
% carrying a comparison that no table states as well:
%
%   fig_benchmark_suppression.png  the headline three-way controller comparison
%   fig_semg_ablation.png          the ablation that attributes the gain to the
%                                  sensor rather than to the mechanism
%   fig_real_tremor.png            what a recorded human tremor looks like next
%                                  to the synthetic one, and what replacing one
%                                  with the other costs each controller
%   fig_reach_gate.png             the fast-reach scenario and the gate's
%                                  response, i.e. the delivered system's one
%                                  clear regression, shown rather than asserted
%
% Everything else the project measured -- the joint-travel repair, the rejected
% rate detector, the real-data flexion/extension split -- is carried as a table
% or as body text, and its diagnostic figure stays in output/figures/ where it
% can still be inspected. Style lives in rcflex_figure_style; this function only
% decides which figures the report gets.
plot_report_benchmark();
plot_report_ablation();
plot_report_real_tremor();
plot_report_reach();
fprintf('Report figures are in %s\n',project_path('figures','report'));
end
