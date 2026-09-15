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
C.ekf_omega_rate_Hz_per_s=50; % max |domega/dt|; guards against low-amplitude false locks
                              % without slowing legitimate fast lock (tuned:
                              % lowest value that still passes the 4-20 Hz
                              % convergence and chirp-tracking unit tests).
C.ekf_lead_s=.0025;      % ~2 sample periods @ 800 Hz
% --- Optional second independent tremor oscillator (ekf_predictive only) -----
% The base model carries two harmonics that share one omega (h=1 at omega,
% h=2 at 2*omega), so it spans {F, 2F} only. Benchmark scenarios 'mixed' and
% 'irregular' superimpose components at F, F-1 and 3 Hz on the pitch axis;
% the off-harmonic content is unmodelled and is absorbed by whichever states
% have the freedom to take it -- in practice the near-constant-velocity intent
% states, which then pass genuine tremor through to the reference as if it
% were voluntary motion. Setting ekf_osc2=true appends a second oscillator
% with its OWN frequency state (states 14:18), giving that content a home so
% it can no longer be mistaken for intent. States 1:13 are unchanged, so
% ekf_osc2=false reproduces the original 13-state filter exactly.
C.ekf_osc2=false;        % single switch: false = 13-state base, true = 18-state widened
C.ekf_q_tremor3=1e-5;    % rad^2 -- second-oscillator amplitude drift
C.ekf_q_omega2=2e-2;     % (rad/s)^2 -- second-oscillator frequency drift
C.ekf_omega2_min_Hz=2;C.ekf_omega2_max_Hz=25;
C.ekf_omega2_init_Hz=3;C.ekf_omega2_p0_Hz=1.5;
C.ekf_omega2_rate_Hz_per_s=50;
% Presence gate for the second oscillator. Without it, omega2 has no
% independent observable of its own and is free to rail against its bound and manufacture a
% phantom low-frequency component out of voluntary intent -- measured at
% 1.1-2.5 deg on the 'combined' scenario, larger than the real 1 deg tremor.
% The gate measures power in the low tremor band only, and when that power is
% absent it both starves the oscillator's process noise and pulls its
% amplitude states to zero, so the filter degrades gracefully to the 13-state
% behaviour instead of inventing signal.
C.ekf_gate_hi_Hz=20;     % 2-pole lowpass corner on the residual. The gate asks
                         % "what does the base model fail to explain", so it must
                         % pass the whole tremor band (roll's unmodelled content
                         % sits at 10-12 Hz, pitch's at 3 Hz); the lowpass is here
                         % only to keep measurement noise out of the power estimate.
C.ekf_gate_tau_s=.25;    % EWMA window on band power
C.ekf_gate_floor_deg=.08;% confidence half-power point on low-band amplitude
C.ekf_gate_sd_deg=.004;  % 1-sigma of the "a3 is zero" pseudo-measurement at
                         % zero confidence; inflated by 1/(1-conf) as the gate opens
% --- Surface EMG sensing (staged: sensed and characterised, NOT yet fused) ---
% Single switch. false = the IMU-only baseline, byte-identical to the system
% before sEMG existed: no sEMG is generated, no extra state is carried, and the
% sEMG diagnostic slots read zero. true = the sEMG stream is generated with the
% scenario, preprocessed causally in the controller, and published to the
% diagnostic vector for characterisation -- but it still does NOT influence the
% reference, the EKF measurement model or the state vector. Deciding how it
% should influence the estimator is the NEXT stage, and that decision is meant
% to be made from the characterisation evidence rather than assumed now.
C.semg_enabled=false;
C.semg_source='synthetic';  % 'synthetic' | 'file' | 'device'. The one place the
                         % raw stream comes from; see rcflex_semg_source.m for
                         % the contract every source must satisfy. Only
                         % 'synthetic' is implemented -- the other two are
                         % declared so the seam exists and the eventual loader
                         % and physical front end drop in without touching the
                         % preprocessing or feature pipeline.
C.semg_hz=4000;          % 5 x C.outer_hz exactly. The integer ratio is the
                         % whole synchronisation strategy: every 800 Hz control
                         % tick contains exactly 5 sEMG samples, so the streams
                         % are aligned by construction and never need resampling
                         % or timestamp matching. rcflex_semg_init.m asserts it.
C.semg_channels=4;       % [roll agonist, roll antagonist, pitch agonist, pitch antagonist]
% Physiology of the synthetic source (rcflex_semg_truth.m).
C.semg_emd_s=.040;       % electromechanical delay: EMG LEADS motion by 30-50 ms
                         % (Cavanagh & Komi 1979). The single property that makes
                         % sEMG worth adding, so it is modelled explicitly.
C.semg_intent_ref_deg=5; % voluntary angle giving unit voluntary drive
C.semg_tremor_ref_deg=1;C.semg_tremor_ref_Hz=8;  % tremor giving unit tremor drive
C.semg_intent_gain=1;C.semg_tremor_gain=1;
C.semg_cocontraction=.05;C.semg_cocontraction_gain=.15;
C.semg_activation_max=1;
C.semg_mvc_V=1.5e-3;     % electrode-referred amplitude at full activation
C.semg_band_lo_Hz=20;C.semg_band_hi_Hz=450;  % motor-unit interference bandwidth
% Instrumentation (rcflex_semg_sample.m).
C.semg_offset_V=15e-3;   % electrode half-cell offset, >>signal: forces the highpass
C.semg_mains_Hz=50;C.semg_mains_V=.2e-3;     % NZ mains, inside the EMG band
C.semg_artifact_V=.12e-3;C.semg_artifact_Hz_range=[2 9];
                         % motion artefact placed deliberately IN the tremor band,
                         % so characterisation cannot flatter the signal by
                         % measuring an artefact-free case.
C.semg_noise_V=8e-6;     % amplifier/electrode broadband floor
C.semg_oversample=4;     % The analogue domain is modelled at C.semg_hz*4 =
                         % 16 kHz and decimated after the anti-alias filter.
                         % Generating straight at 4 kHz would assume the band
                         % limiting the anti-alias stage exists to enforce, and
                         % the filter could then never be shown to earn its
                         % place. 4x is the smallest ratio that leaves real
                         % content above the 2 kHz final Nyquist.
C.semg_aa_Hz=480;        % analogue anti-alias corner, just above the 450 Hz
                         % signal band edge
C.semg_aa_poles=2;       % simplest filter consistent with 20-450 Hz and 4x
                         % oversampling; a Sallen-Key section in hardware
C.semg_ac_couple_Hz=10;  % analog highpass AHEAD of the gain stage. Without it
                         % the electrode offset alone demands ~7.5 V of a 1.65 V
                         % converter and rails the channel; see
                         % rcflex_semg_sample.m.
C.semg_gain=200;         % Set from the PEAK, not the rms: surface EMG is an
                         % interference pattern with a crest factor of 7-10, so
                         % a gain chosen from the rms clips the peaks. The bound
                         % is 1.65 V / 7.13 mV worst-case peak = 231; 200 leaves
                         % 1.6-7.5 dB of headroom with zero clipping across all
                         % scenario/seed combinations, measured by
                         % rcflex_semg_quality. See docs/SEMG_SUBSYSTEM.md 1.4.
C.semg_adc_range_V=1.65;C.semg_adc_bits=12;
% Causal preprocessing (rcflex_semg_init.m / rcflex_semg_preprocess.m).
C.semg_hp_Hz=20;         % must lead the chain: removes the half-cell offset
C.semg_notch_bw_Hz=4;    % mains notch width
C.semg_env_Hz=3;         % linear-envelope lowpass -> voluntary effort level
C.semg_burst_band_Hz=[2 20]; % envelope bandpass -> rhythmic tremor bursting
% --- sEMG -> EKF fusion (experiment E1). THE SINGLE SWITCH ------------------
% One field selects the arm; nothing else needs editing to change behaviour.
%   'off'    IMU-only. Bit-identical to the pre-fusion code.
%   'semg'   burst-band frequency observable as a scalar pseudo-measurement on
%            the EKF's omega state (13). No new EKF states.
%   'oracle' the TRUE tremor frequency fed to the same input. Diagnostic upper
%            bound only -- it reads simulation ground truth and is NOT a
%            realisable mode. It exists so a null result can be attributed:
%            if the oracle does not help either, omega was never the binding
%            constraint and the fusion premise itself is wrong.
% Requires C.semg_enabled=true and C.algorithm='ekf_predictive'; otherwise it
% is inert, so the default configuration is unaffected.
C.semg_fusion='off';
C.semg_fuse_D=16;        % autocorrelation lag in OUTER ticks (20 ms @ 800 Hz).
                         % Chosen so omega*D*dt stays inside (0,pi) across the
                         % whole 2-25 Hz clamp band -- at 25 Hz it is exactly
                         % pi, so acos stays single-valued and the estimate
                         % cannot alias. Near 8 Hz it sits at ~1.0 rad, close
                         % to the pi/2 point of maximum sensitivity.
C.semg_fuse_tau_s=.15;   % EWMA window on the autocorrelation accumulators
C.semg_fuse_r_omega=(2*pi*.6)^2;  % (rad/s)^2. Set from the MEASURED spread of
                         % the burst dominant frequency on the synthetic source
                         % (7.81-9.38 Hz against a true 8 Hz, SEMG_IMPLEMENTATION
                         % 11) -- i.e. the observable is trusted to about
                         % +/-0.6 Hz. Not tuned against the experiment outcome.
C.semg_fuse_floor=1.5e-7;% burst-power half-power point for the confidence gate.
                         % Measured: tremor-bearing scenarios carry 2.7e-7 to
                         % 6.3e-7, tremor-free 'intent' carries 5.3e-8, so the
                         % floor sits between them. Without this gate a rest or
                         % intent-only segment would inject the arccos of a
                         % noise ratio into an omega state that is otherwise
                         % uninformed -- the gate makes R_eff -> inf instead.
C.semg_fusion_oracle_Hz=8;  % only read when C.semg_fusion='oracle'
% Trust gate (NIS-based). n_lo/n_hi bracket the expected NIS (=2, measurement dim).
C.ekf_nis_tau_s=.05;C.ekf_nis_lo=4;C.ekf_nis_hi=25;C.ekf_q_boost=50;
% Predictive authority manager.
C.auth_horizon_s=.02;C.auth_margin=.85;C.auth_rate_per_s=5;
C.algorithm=algorithm;C.name=algorithm;
end
