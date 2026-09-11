function S=run_tremor_platform_check()
% Dynamic smoke test with simultaneous rotation and world translation.
% No controller: zero-voltage bridge gives passive electromechanical response.
H=rcflex_parameters();t=(0:1/H.fs:2)';base=rcflex_tremor_input(t,6,1,1);
S=run_rcflex_analytic('off',2,H,zeros(numel(t),2),base);
S.metrics=rcflex_world_metrics(S.Y(:,1:2),base,H);
assert(all(isfinite(S.Y),'all')&&all(isfinite(S.sensors.Data),'all'));
save(project_path('results','tremor_passive_check.mat'),'S');
fprintf('Mixed rotation/translation dynamic simulation passed; passive result is not a stabilisation score.\n');
end
