function run_validation()
%RUN_VALIDATION Current mechanism, electrical model and sensor checks.
checks={@test_rcflex_mesh_properties,@test_spoon_reference,...
 @test_rcflex_packaging,@test_rcflex_shell_clearance,@test_rcflex_front_cap,...
 @test_rcflex_spherical_clearance,@test_rcflex_physics,...
 @test_rcflex_hardware,@test_rcflex_translation,@test_rcflex_control,@test_rcflex_liquid};
names=cellfun(@func2str,checks,'UniformOutput',false);elapsed=zeros(numel(checks),1);
for k=1:numel(checks)
 t=tic;checks{k}();elapsed(k)=toc(t);
end
passed=true(numel(checks),1);
writetable(table(names',passed,elapsed,'VariableNames',{'check','passed','seconds'}),...
 project_path('results','validation_summary.csv'));
fprintf('All %d validation groups passed.\n',numel(checks));
end
