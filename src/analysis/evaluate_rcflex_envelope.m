function T=evaluate_rcflex_envelope(H,writeFiles)
% Inverse-dynamics feasibility bound, NOT an implemented controller or simulation
% of successful suppression. Assumes perfect knowledge and exact attitude tracking.
if nargin<1;H=rcflex_parameters();end
if nargin<2;writeFiles=true;end
rows=[];
% 20 Hz is included because benchmark case 20 sits there: the headline metric
% excludes cases this table calls infeasible, so every benchmark frequency must
% appear here or the exclusion would rest on an assertion instead of a
% measurement.
for food=[.005 .015]
 h=H;h.food_mass_kg=food;
 for hz=[4 8 12 20]
  for amplitude=[1 3 5]
   % Single shared implementation -- see rcflex_case_feasible.m. The benchmark
   % reads the same function per case, so the table and the per-case flag
   % cannot disagree.
   [feasible,c]=rcflex_case_feasible(hz,amplitude,h);
   locked=rcflex_world_metrics(zeros(size(c.q)),c.base,h);ideal=rcflex_world_metrics(c.q,c.base,h);
   rows=[rows;food*1000 hz amplitude c.required_peak_A c.required_peak_V c.max_coil_mean_W c.peak_stroke_mm c.peak_side_load_N locked.position_rms_mm ideal.position_rms_mm feasible]; %#ok<AGROW>
  end
 end
end
T=array2table(rows,'VariableNames',{'payload_g','frequency_Hz','amplitude_deg','required_peak_A','required_peak_V','max_coil_mean_W','peak_stroke_mm','peak_side_load_N','locked_position_rms_mm','ideal_attitude_position_rms_mm','nominal_feasible'});
if ~writeFiles;return;end
writetable(T,project_path('results','tremor_hardware_envelope.csv'));disp(T);
f=figure('Color','w','Visible','off','Position',[100 100 1300 520]);if isprop(f,'Theme');f.Theme='light';end
tiledlayout(1,2);nexttile;hold on;
for food=[5 15];idx=T.payload_g==food&T.amplitude_deg==3;plot(T.frequency_Hz(idx),T.required_peak_A(idx),'-o','DisplayName',sprintf('%g g payload',food));end
yline(H.current_limit_A,'--','current limit','DisplayName','Current limit');xlabel('Frequency (Hz)');ylabel('Required peak current (A)');legend('Location','best');title('Perfect attitude tracking: inverse bound');grid on;
nexttile;idx=T.payload_g==5&T.frequency_Hz==8;bar(T.amplitude_deg(idx),[T.locked_position_rms_mm(idx) T.ideal_attitude_position_rms_mm(idx)]);xlabel('Input amplitude (deg)');ylabel('Bowl position RMS (mm)');legend('Locked spoon','Ideal attitude tracking');title('Level does not mean position-stationary');grid on;
exportgraphics(f,project_path('figures','tremor_hardware_envelope.png'),'Resolution',160);close(f);
end
