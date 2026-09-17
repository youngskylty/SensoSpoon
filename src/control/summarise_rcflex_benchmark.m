function S=summarise_rcflex_benchmark(writeOutputs)
%SUMMARISE_RCFLEX_BENCHMARK The headline table, with the exclusions made explicit.
%
% Reads output/results/control_benchmark.csv and reduces it to one row per arm.
% It exists so that the headline number is produced by code with a stated rule,
% rather than by averaging a column by hand and deciding afterwards which cases
% to leave out.
%
% THREE MEANS ARE REPORTED, ALWAYS TOGETHER. Quoting only the most flattering
% one would be the exact failure this function is designed to prevent.
%
%   mean_all_scored      every case with a defined suppression figure, i.e. all
%                        20 except case 10 (no tremor, so suppression is 0/0).
%                        Includes the cases the hardware cannot serve, and is
%                        therefore dominated by them.
%
%   mean_feasible        the HEADLINE. Restricted to cases the hardware could
%                        track at all, as judged by rcflex_case_feasible -- an
%                        inverse-dynamics bound that assumes perfect knowledge
%                        and exact tracking and knows nothing about any
%                        controller. Cases 5, 14 and 20 fail it (the first two
%                        on bus voltage, the last on current, voltage and
%                        continuous copper power simultaneously), so no
%                        controller could have succeeded on them and scoring
%                        one on them measures the actuator, not the algorithm.
%
%   mean_in_envelope     the subset of mean_feasible where the run ALSO stayed
%                        inside the limits (within_limits). Reported because
%                        "the input was servable" and "this arm served it
%                        without breaching a limit" are different claims.
%
% Case 10 is reported separately as attitude RMS, since it is the only case
% that measures voluntary-motion preservation and the only one where a
% suppression percentage is undefined.
if nargin<1;writeOutputs=true;end
T=readtable(project_path('results','control_benchmark.csv'),'TextType','string');
arms=unique(T.arm,'stable');
rows={};
for a=1:numel(arms)
 q=T(T.arm==arms(a),:);
 scored=~isnan(q.suppression_percent);
 feas=scored&q.hardware_feasible==1;
 inenv=feas&q.within_limits==1;
 c10=q(q.case_id==10,:);
 rows(end+1,:)={arms(a),q.algorithm(1),q.intent_model(1),q.semg_fusion(1),...
  mean(q.suppression_percent(scored)),mean(q.suppression_percent(feas)),...
  mean(q.suppression_percent(inenv)),sum(feas),...
  min(q.suppression_percent(feas)),c10.attitude_rms_deg,...
  sum(q.meets_90_percent_target(feas)),max(q.current_peak_A),max(q.power_mean_W)}; %#ok<AGROW>
end
S=cell2table(rows,'VariableNames',{'arm','algorithm','intent_model','semg_fusion',...
 'mean_all_scored','mean_feasible','mean_in_envelope','n_feasible',...
 'worst_feasible','case10_voluntary_rms_deg','n_meets_90_of_feasible',...
 'peak_current_A','peak_mean_power_W'});
disp(S);
ex=unique(T.case_id(T.hardware_feasible==0));
fprintf('Excluded from mean_feasible as outside the hardware envelope: cases %s\n',mat2str(ex(:)'));
fprintf('Case 10 carries no tremor, so its suppression is undefined; it is scored by attitude RMS only.\n');
if writeOutputs
 out=project_path('results','control_benchmark_summary.csv');
 writetable(S,out);fprintf('Wrote %s\n',out);
end
end
