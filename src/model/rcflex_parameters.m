function H=rcflex_parameters()
%RCFLEX_PARAMETERS Drawing dimensions in mm; dynamics in SI.
% Source: supplied RC-Flex illustration and Hardware Feasibility PDF pp4,6,13.
% This is a nominal reconstruction, not manufacturing CAD or a validated RCM.
H.revision='RCFlex_curved_neck_v17';
H.total_length_mm=271;
H.handle_mm=[168 30 24]; % base section scaling; actual shell profile is below
H.bowl_mm=[62 40 12]; % current deeper bowl; outer total depth
H.bowl_tip_x_mm=-103;
H.bowl_widest_station_mm=37.2;
H.bowl_front_width_mm=28;
H.bowl_front_station_mm=37.2*(1-sqrt(1-(28/40)^2)); % inferred ellipse section; undimensioned in drawing
H.bowl_connection_width_mm=10;
H.bowl_bottom_thickness_mm=1.8;
H.bowl_measurement_x_mm=-65.8;
H.bowl_rim_z_mm=-4;
H.neck_length_mm=31;
H.gimbal_mm=[25 33 19]; % open support envelope, including handle mounting roots
H.pushrod_diameter_mm=2;
H.physical_travel_deg=15; % conservative end of drawing's +/-15-20 degrees
% Origin: equivalent flexure centre; rear x=168, spoon tip x=-103 mm.
H.grip_x_mm=65;                  % ASSUMED, not measured hand kinematics
H.remote_center_x_mm=NaN;        % UNKNOWN: Document 1 is required
H.equivalent_center_x_mm=0;      % explicit local-joint fallback, NOT RCM claim
H.shell_wall_mm=1.4;             % ASSUMED
H.shell_x_mm=[0 18 35 55 80 100 130 150 160 168];
H.shell_y_scale=[21 20 18 16.5 16.3 16.6 17.2 16.3 12 2]/15;
H.shell_z_scale=[21 20 17.8 14 13.5 14 15 14 11 2]/12;
H.shell_center_z_mm=[0 0 1.5 4 6 6 6 6 6 6];
H.ball_outer_radius_mm=17;H.ball_inner_radius_mm=16.2;
H.ball_x_limits_mm=[-15.5 -3];
H.bezel_outer_radius_mm=21;H.bezel_inner_radius_mm=19.8;
H.bezel_x_limits_mm=[-16 0];
H.front_cap_x_mm=18;
H.front_rod_windows_mm=[-6.5 -2 9 18;6.5 -2 9 18]; % yc,zc,width,height
H.bowl_wall_mm=1.2; % edge thickness per supplied reference
H.vca_length_mm=23.9; H.vca_diameter_mm=15.9; % body + rear bearing cap
H.vca_centers_mm=[55.5 -5 6;87.5 5 6];
H.vca_stroke_mm=6.35;            % half of datasheet's 12.7 mm TOTAL stroke
H.vca_force_peak_N=4.6;          % GVCM-016-019-01M: only at 10% duty cycle
H.vca_force_cont_N=1.5;
H.vca_force_constant_N_A=1.5;    % same numeric back-EMF constant in SI
H.steel_E_Pa=200e9;              % ASSUMED spring-steel modulus
H.flexure_pivot_parts={'C-Flex B-10 x2 (pitch)','C-Flex D-10 x1 (roll)'};
H.flexure_stiffness_Nm_rad=[2*.0004;.0018]*.112984829*(180/pi); % in*lbf/deg -> Nm/rad
H.flexure_stiffness_tolerance=.10; % manufacturer zero-load spring-rate tolerance
H.flexure_damping_Nms_rad=2e-4;   % ASSUMED, requires identification
H.flexure_cubic_ratio=0; % catalog nominal torsional rates; no invented hardening
H.fs=1000; H.duration=20;
% Direct pushrods; geometry in SI, q=[pitch;roll].
H.link.pivot=[.012 -.005 .0075;.012 .005 .0075]'; % only y defines slider axes
H.link.mode='direct';
H.link.attachment=[.006 -.0085 -.010;.006 .0085 -.010]';
H.link.slider_x=[.030 .030];H.link.slider_z=.006;
H.link.input_length=sqrt((H.link.slider_x-H.link.attachment(1,:)).^2+...
 (H.link.pivot(2,:)-H.link.attachment(2,:)).^2+(H.link.slider_z-H.link.attachment(3,:)).^2);
H.link.output_length=0; % no output link or bellcrank in direct version
% Selected motor/driver parameters; thermal and friction assumptions are marked.
H.coil_R_ohm=4.6;H.coil_L_H=.0008;H.bus_voltage_V=7.4;
H.current_limit_A=1.5;H.coil_alpha_K=.00393; % chosen driver threshold
H.coil_mass_kg=.011;H.motor_body_mass_kg=.017;
H.coil_continuous_power_W=4.4;H.shaft_side_load_N=2.2;
H.driver_R_ohm=.7; % DRV8876 typical high + low MOSFET resistance
H.driver_pwm_hz=20000;H.driver_min_voltage_V=4.5;
H.battery_capacity_Ah=.4;H.battery_R_ohm=.15; % resistance not published: estimate
H.battery_mass_kg=.018;H.battery_mm=[41 20 13];
H.motor_part='Moticont GVCM-016-019-01M';H.driver_part='TI DRV8876PWPR';
H.guide_friction_N=.03;H.guide_viscous_Ns_m=.03; % not published: sensitivity inputs
H.imu_part='Bosch BMI270';H.imu_gyro_noise_dps_sqrtHz=.007;
H.imu_acc_noise_g_sqrtHz=160e-6;H.imu_gyro_offset_dps=.5;
H.imu_rate_hz=800;
H.ambient_C=25;H.thermal_capacity_J_K=8;H.thermal_resistance_K_W=18;
H.stop_stiffness_Nm_rad=8;H.stop_damping_Nms_rad=.025;
H.slider_stop_N_m=10000;H.slider_stop_Ns_m=2;
H.food_mass_kg=.005; % removable payload at bowl measurement point
end


