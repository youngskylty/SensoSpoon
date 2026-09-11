function C=rcflex_control_parameters()
% Frozen baseline gains selected on the declared training cases.
C.outer_hz=800;C.current_hz=10000;
C.intent_filter='notch';C.notch_frequencies_Hz=[4 6 8 10 12];C.notch_bandwidth_Hz=7;
C.intent_lead_s=.14;C.intent_lead_pole_Hz=1;C.intent_rolloff_Hz=10;
C.intent_hz=1;C.complement_hz=.05;
C.natural_hz=35;C.damping=.85;C.integral_gain=80;
C.current_kp=.8;C.current_ki=2000;C.current_aw=1500;
C.joint_limit=deg2rad(14);C.reference_rate=deg2rad(180);
C.gyro_bias=zeros(6,1);C.accel_bias=zeros(6,1);
C.current_quantum=7/4096;C.current_offset=[.001;-.001];
C.current_noise_A=.0005;C.power_window_s=2;
C.seed=1701;C.name='simple_pid';
end
