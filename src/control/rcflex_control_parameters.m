function C=rcflex_control_parameters(algorithm)
% Frozen baseline gains selected on the declared training cases.
% algorithm: 'simple_pid' (default, fixed notch chain) | 'ekf_predictive'
% (13-state EKF intent/tremor separator, see docs/CONTROL_IMPROVEMENT_PLAN.md).
% Flipping algorithms requires changing only this argument, nothing else.
if nargin<1;algorithm='simple_pid';end
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
C.seed=1701;
% --- EKF intent/tremor separator (used only when algorithm='ekf_predictive') ---
C.ekf_harmonics=2;
C.ekf_q_intent=1e-4;     % rad^2/s^3 -- voluntary motion agility
C.ekf_q_tremor=1e-5;     % rad^2     -- fundamental (h=1) tremor amplitude drift
C.ekf_q_tremor2=1e-9;    % rad^2     -- 2nd harmonic (h=2) amplitude drift, deliberately
                         % much tighter than h1: a shared-omega two-harmonic oscillator
                         % is degenerate for a pure tone (h=1,omega=F and h=2,omega=F/2
                         % explain it identically), so a tight prior on the harmonic
                         % that real tremor uses least breaks the tie toward the
                         % fundamental. Verified empirically -- see CONTROL_IMPROVEMENT_PLAN.md.
C.ekf_q_omega=2e-2;      % (rad/s)^2 -- frequency drift rate; tuned so a 4->12 Hz
                         % chirp is tracked with <1 Hz lag while pure tones (incl.
                         % 20 Hz) still converge to within 0.01 Hz.
C.ekf_r_meas=(deg2rad(.02))^2;  % attitude estimate noise
C.ekf_omega_min_Hz=2;C.ekf_omega_max_Hz=25;C.ekf_omega_init_Hz=6;C.ekf_omega_p0_Hz=1.5;
C.ekf_lead_s=.0025;      % ~2 sample periods @ 800 Hz
% Trust gate (NIS-based). n_lo/n_hi bracket the expected NIS (=2, measurement dim).
C.ekf_nis_tau_s=.05;C.ekf_nis_lo=4;C.ekf_nis_hi=25;C.ekf_q_boost=50;
% Predictive authority manager.
C.auth_horizon_s=.02;C.auth_margin=.85;C.auth_rate_per_s=5;
C.algorithm=algorithm;C.name=algorithm;
end
