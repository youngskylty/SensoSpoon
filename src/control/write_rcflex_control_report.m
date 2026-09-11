function write_rcflex_control_report()
T=readtable(project_path('results','control_benchmark.csv'));N=readtable(project_path('results','control_numerical_verification.csv'));
d=load(project_path('results','control_native.mat'),'S');v=load(project_path('results','control_numerical_verification.mat'),'V');
assert(v.V.duration_s==d.S.time(end)&&isequaln(v.V.C,d.S.C),'Numerical verification must match the latest native run');
f=fopen(project_path('results','CONTROL_RESULTS.md'),'w');cleanup=onCleanup(@()fclose(f));
fprintf(f,'# Closed-loop validation results\n\nFixed five-notch intent filter (4/6/8/10/12 Hz, bandwidth parameter 7 Hz), bounded phase lead and 10 Hz rolloff; attitude bandwidth 35 Hz; 800 Hz outer and 10 kHz inner loop.\n\n');
fprintf(f,'Not all scenarios meet the target: movement tracking, larger amplitudes and out-of-band disturbance remain limitations. Inspect failed cases, not only nominal single-frequency results.\n\n');
fprintf(f,'Attitude scoring includes intention tracking error. Baseline: rigidly attached spoon under identical base input. Steady window: t >= 1.5 s; no scoring filter.\n\n');
fprintf(f,'| Case | Scenario | Hz | Input amplitude/axis (deg) | Attitude RMS (deg) | Reduction (%%) | Position RMS (mm) | Peak current (A) | Constraints |\n|---|---|---:|---:|---:|---:|---:|---:|---|\n');
for k=1:height(T)
 fprintf(f,'| %d | %s | %.0f | %.1f | %.3f | %.1f | %.2f | %.2f | %d |\n',T.case_id(k),string(T.scenario(k)),T.frequency_Hz(k),T.amplitude_deg(k),T.attitude_rms_deg(k),T.suppression_percent(k),T.position_rms_mm(k),T.current_peak_A(k),T.within_limits(k));
end
fprintf(f,'\nCases 11/12: 0/15 g payload. Case 13: 5.8 V bus. Case 14: amplitude doubles after 2 s. Case 15: +20%% resistance, -15%% force constant, double guide friction. Case 10: intentional motion only, so reduction is undefined. Case 9 combines intentional motion and tremor. Cases 16-19 test 5/7/9/11 Hz between notch centers; case 20 is a 20 Hz out-of-band test. Cases 14 and 20 use a finer 0.1 ms mechanical step for boundary checks; other cases use 0.5 ms.\n');
fprintf(f,'\nFast versus native Simulink: nominal 8 Hz, 1 deg/axis, %g s; default-step joint difference RMS %.5f deg; reduction difference %.3f percentage points. This validates one nominal case, not every boundary case.\n',d.S.time(end),N.joint_difference_rms_deg(1),N.suppression_difference_points(1));
d=load(project_path('results','control_native.mat'),'S');m=rcflex_control_metrics(d.S);
fprintf(f,'\nNative continuous reference (8 Hz, 1 deg/axis, 5 g, seed %d, scoring 1.5-%.1f s): attitude RMS %.4f deg, reduction %.2f percent, peak current %.3f A, peak stroke %.3f mm. The independent suite above uses longer runs and different noise seeds.\n',d.S.C.seed,d.S.time(end),m.attitude_rms_deg,m.suppression_percent,m.current_peak_A,m.stroke_peak_mm);
fprintf(f,'\nConstraints column checks observed joint angle, stroke, current, estimated side load and average copper power. Short-run passes do not certify long-term temperature, fatigue or robustness. Controller assumes a 2-second stationary level-fixture calibration before startup.\n');
fprintf(f,'\nThese are simulated results with nominal/custom and partly unmeasured parameters, not hardware experimental results. Two controlled axes cannot reject arbitrary translational and rotational disturbance simultaneously; 100%% rejection of arbitrary tremor is not achieved.\n');
end
