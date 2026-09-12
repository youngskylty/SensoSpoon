function T=evaluate_rcflex_envelope(H,writeFiles)
% Inverse-dynamics feasibility bound, NOT an implemented controller or simulation
% of successful suppression. Assumes perfect knowledge and exact attitude tracking.
if nargin<1;H=rcflex_parameters();end
if nargin<2;writeFiles=true;end
rows=[];
for food=[.005 .015]
 h=H;h.food_mass_kg=food;B=rcflex_body_properties(h);
 for hz=[4 8 12]
  for amplitude=[1 3 5]
   t=linspace(0,1/hz,121)';base=rcflex_tremor_input(t,hz,amplitude,1);
   q=-base(:,[2 1]);qd=-base(:,[4 3]);qdd=-base(:,[6 5]);
   ii=zeros(numel(t),2);sd=ii;stroke=ii;side=ii;
   for k=1:numel(t)
    [iReq,rc]=rcflex_required_current(q(k,:)',qd(k,:)',qdd(k,:)',base(k,1:2)',base(k,3:4)',base(k,5:6)',base(k,7:15)',h,B);
    ii(k,:)=iReq.';stroke(k,:)=rc.stroke.';side(k,:)=rc.side_load_N.';sd(k,:)=(rc.L.J*qd(k,:)').';
   end
   di=[gradient(ii(:,1),t) gradient(ii(:,2),t)];
   K=h.vca_force_constant_N_A*max(1-.27*(stroke/(h.vca_stroke_mm/1000)).^2,.5);
   voltage=(h.coil_R_ohm+h.driver_R_ohm)*ii+h.coil_L_H*di+K.*sd;
   bus=h.bus_voltage_V-h.battery_R_ohm*sum(abs(ii),2);
   copper=max(mean(h.coil_R_ohm*ii.^2,1));
   feasible=max(abs(ii),[],'all')<=h.current_limit_A && all(abs(voltage)<=bus,'all') && copper<=h.coil_continuous_power_W && max(side,[],'all')<=h.shaft_side_load_N && max(abs(stroke),[],'all')<=h.vca_stroke_mm/1000;
   locked=rcflex_world_metrics(zeros(size(q)),base,h);ideal=rcflex_world_metrics(q,base,h);
   rows=[rows;food*1000 hz amplitude max(abs(ii),[],'all') max(abs(voltage),[],'all') copper max(abs(stroke),[],'all')*1000 max(side,[],'all') locked.position_rms_mm ideal.position_rms_mm feasible]; %#ok<AGROW>
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
