function write_rcflex_control_report()
T=readtable(project_path('results','control_benchmark.csv'));N=readtable(project_path('results','control_numerical_verification.csv'));
d=load(project_path('results','control_native.mat'),'S');v=load(project_path('results','control_numerical_verification.mat'),'V');
assert(v.V.duration_s==d.S.time(end)&&isequaln(v.V.C,d.S.C),'Numerical verification must match the latest native run');
f=fopen(project_path('results','CONTROL_RESULTS.md'),'w');cleanup=onCleanup(@()fclose(f));
fprintf(f,'# Closed-loop validation results\n\nFixed five-notch intent filter (4/6/8/10/12 Hz, bandwidth parameter 7 Hz), bounded phase lead and 10 Hz rolloff; attitude bandwidth 35 Hz; 800 Hz outer and 10 kHz inner loop.\n\n');
fprintf(f,'Not all scenarios meet the target: movement tracking, larger amplitudes and out-of-band disturbance remain limitations. Inspect failed cases, not only nominal single-frequency results.\n\n');
fprintf(f,'Attitude scoring includes intention tracking error. Baseline: rigidly attached spoon under identical base input. Steady window: t >= 1.5 s; no scoring filter.\n\n');
% Rows are identified by ARM, not by algorithm: several arms share an
% algorithm and differ only in the intent model or the sEMG fusion switch, so
% the algorithm column alone cannot tell them apart.
if ~ismember('arm',T.Properties.VariableNames);T.arm=repmat("simple_pid",height(T),1);end
if ~ismember('hardware_feasible',T.Properties.VariableNames);T.hardware_feasible=ones(height(T),1);end
% Headline means first, with the exclusions stated. Written by the same code
% that computes them (summarise_rcflex_benchmark) so the report and the CSV
% cannot disagree.
S=summarise_rcflex_benchmark(false);
fprintf(f,'## Headline, by arm\n\n');
fprintf(f,'`mean_feasible` is the headline: it excludes cases the hardware could not serve under ANY controller, judged by an inverse-dynamics bound that knows nothing about the controller (`rcflex_case_feasible.m`). All three means are shown so the exclusion is visible rather than silent. Case 10 carries no tremor, so its suppression is undefined and it is scored by attitude RMS only.\n\n');
fprintf(f,'| Arm | sEMG fusion | Mean suppression, all scored (%%) | Mean suppression, feasible cases (%%) | Worst feasible case (%%) | Case 10 voluntary-motion RMS (deg) | Cases meeting 90%% |\n|---|---|---:|---:|---:|---:|---:|\n');
for k=1:height(S)
 fprintf(f,'| %s | %s | %.2f | %.2f | %.2f | %.4f | %d of %d |\n',string(S.arm(k)),string(S.semg_fusion(k)),S.mean_all_scored(k),S.mean_feasible(k),S.worst_feasible(k),S.case10_voluntary_rms_deg(k),S.n_meets_90_of_feasible(k),S.n_feasible(k));
end
fprintf(f,'\n## Every case, every arm\n\n');
fprintf(f,'| Arm | Case | Scenario | Hz | Input amplitude/axis (deg) | Attitude RMS (deg) | Reduction (%%) | Position RMS (mm) | Peak current (A) | Constraints | Inside hardware envelope |\n|---|---|---|---:|---:|---:|---:|---:|---:|---|---|\n');
for k=1:height(T)
 fprintf(f,'| %s | %d | %s | %.0f | %.1f | %.3f | %.1f | %.2f | %.2f | %d | %d |\n',string(T.arm(k)),T.case_id(k),string(T.scenario(k)),T.frequency_Hz(k),T.amplitude_deg(k),T.attitude_rms_deg(k),T.suppression_percent(k),T.position_rms_mm(k),T.current_peak_A(k),T.within_limits(k),T.hardware_feasible(k));
end
fprintf(f,'\nThe final column is the independent hardware-envelope flag. Cases 5, 14 and 20 fail it: the inverse-dynamics demand exceeds the bus voltage (cases 5 and 14) or the current, voltage and continuous copper power together (case 20). A suppression figure on those cases measures the actuator, not the algorithm.\n');
fprintf(f,'\nCases 11/12: 0/15 g payload. Case 13: 5.8 V bus. Case 14: amplitude doubles after 2 s. Case 15: +20%% resistance, -15%% force constant, double guide friction. Case 10: intentional motion only, so reduction is undefined. Case 9 combines intentional motion and tremor. Cases 16-19 test 5/7/9/11 Hz between notch centers; case 20 is a 20 Hz out-of-band test. Cases 14 and 20 use a finer 0.1 ms mechanical step for boundary checks; other cases use 0.5 ms.\n');
fprintf(f,'\nFast versus native Simulink: nominal 8 Hz, 1 deg/axis, %g s; default-step joint difference RMS %.5f deg; reduction difference %.3f percentage points. This validates one nominal case, not every boundary case.\n',d.S.time(end),N.joint_difference_rms_deg(1),N.suppression_difference_points(1));
d=load(project_path('results','control_native.mat'),'S');m=rcflex_control_metrics(d.S);
fprintf(f,'\nNative continuous reference (8 Hz, 1 deg/axis, 5 g, seed %d, scoring 1.5-%.1f s): attitude RMS %.4f deg, reduction %.2f percent, peak current %.3f A, peak stroke %.3f mm. The independent suite above uses longer runs and different noise seeds.\n',d.S.C.seed,d.S.time(end),m.attitude_rms_deg,m.suppression_percent,m.current_peak_A,m.stroke_peak_mm);
fprintf(f,'\nConstraints column checks observed joint angle, stroke, current, estimated side load and average copper power. Short-run passes do not certify long-term temperature, fatigue or robustness. Controller assumes a 2-second stationary level-fixture calibration before startup.\n');
fprintf(f,'\nThese are simulated results with nominal/custom and partly unmeasured parameters, not hardware experimental results. Two controlled axes cannot reject arbitrary translational and rotational disturbance simultaneously; 100%% rejection of arbitrary tremor is not achieved.\n');
end
