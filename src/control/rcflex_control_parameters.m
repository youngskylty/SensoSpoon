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
% --- Intent sub-model. THE SINGLE SWITCH ------------------------------------
% One field selects the intent process model; nothing else needs editing.
%   'cv_legacy' constant-VELOCITY covariance (F carries dt) but the state mean
%               is NOT propagated -- xpred keeps the previous angle. This is
%               the original behaviour and stays the default so every existing
%               result and test is reproduced exactly. It is inconsistent: the
%               covariance is advanced as though theta <- theta + dt*thetadot
%               while the mean is not.
%   'cv'        the same constant-velocity model with the mean propagated to
%               match the covariance. The minimal repair of that inconsistency.
%   'ca'        constant-ACCELERATION: one extra state per intent axis, so the
%               model can represent sustained angular acceleration instead of
%               dumping it into the tremor oscillator (states 1:13 keep their
%               indices; the new states are appended after omega).
C.ekf_intent_model='cv_legacy';
% Jerk process noise for 'ca'. NOT tuned against any result. Chosen by one
% stated rule: preserve the existing model's one-step VELOCITY increment
% allowance exactly, now routed through acceleration. A one-step velocity
% increment of dt*sqrt(q_jerk) equal to sqrt(C.ekf_q_intent) gives
% q_jerk = C.ekf_q_intent/dt^2. The CA model therefore gets no extra agility
% than the CV model already had -- only the ability to sustain it.
C.ekf_q_intent_jerk=C.ekf_q_intent*C.outer_hz^2;
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
%
% E1 RESULT: NEGATIVE, and kept in the tree as the record. Perfect omega bought
% only +7.9 points on case 9, the burst-band frequency observable is biased
% (9.15 Hz against a true 8 Hz), and 4 of 5 in-band sine cases regressed. Omega
% was never the binding constraint. See the commit "Add the sEMG->omega fusion
% experiment (E1)".
%
% --- Experiment E2: sEMG -> EKF INTENT PROCESS NOISE ------------------------
% The arms the SHIPPED system uses. Same single switch.
%   'intent_gate'       the sEMG+IMU system. Per-axis voluntary-drive
%                       confidence from the differential envelope
%                       (rcflex_semg_intent_gate) scales the intent process
%                       noise, i.e. the estimator's licence to call motion
%                       voluntary. Requires C.semg_enabled=true.
%   'intent_gate_fixed' the IMU-ONLY ABLATION of that mechanism: identical Q
%                       scaling held at the constant C.semg_gate_fixed_conf,
%                       with no sEMG stream at all. This is the control group
%                       that isolates the sensor's contribution from the
%                       mechanism's, and it is why the sEMG claim can be made
%                       quantitatively. It is the classic "just lower q_intent"
%                       fix, and it fails exactly as predicted -- see
%                       docs/SEMG_INTENT_FUSION.md Sec 2.
% Requires C.algorithm='ekf_predictive'; otherwise inert, so the default
% configuration is unaffected.
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
% --- E2 intent gate (read only by the 'intent_gate*' arms) ------------------
C.semg_gate_fc_Hz=1;     % 2-pole lowpass on the agonist-antagonist envelope
                         % difference. Muscle force follows tremor
                         % ACCELERATION, which grows as omega^2, so tremor-
                         % driven activation leaks into the envelope worst at
                         % high tremor frequency. Selected by sweep over
                         % 0.6-5 Hz as the corner below which the measured
                         % tremor-only drive stops falling appreciably, while
                         % the 0.2-0.25 Hz voluntary content is still passed.
C.semg_gate_fast_fc_Hz=4;% corner of the gate's FAST path, which can only ever
                         % OPEN the gate (the two paths are combined with max;
                         % see rcflex_semg_intent_gate.m). 4 Hz costs ~56 ms of
                         % group delay instead of the slow path's ~225 ms, which
                         % is what lets the gate open during a voluntary
                         % movement rather than after it. It exists because the
                         % slow path alone degraded the 'reach' scenario to
                         % 0.73-1.15 deg attitude RMS against 0.285 deg
                         % IMU-only, across four sensor seeds.
C.semg_gate_fast_floor_deg=1.17;% half-power point for the fast path, in the same
                         % equivalent-wrist-angle units and set by the SAME
                         % geometric-mean rule as the slow floor, but measured
                         % at 4 Hz where more tremor-driven activation
                         % survives: tremor-only <=0.676 deg, intent-bearing
                         % >=2.016 deg, only 3.0:1 against the slow path's
                         % 6.3:1. Higher than the slow floor precisely because
                         % that extra leakage is real and must be priced in
                         % rather than ignored. The cost is paid in TREMOR
                         % suppression, not in voluntary motion: the worst
                         % tremor-only case now reads 0.063 instead of 0.019,
                         % so the intent process noise closes to about 1/16
                         % rather than 1/53. Re-derived by
                         % calibrate_rcflex_semg_intent.m.
C.semg_gate_attack_s=.05;% ATTACK window on the mean-square drive. Short: the
                         % gate must be open before the limb has moved, and the
                         % 35-40 ms electromechanical lead the sEMG carries is
                         % the entire budget for that, so the attack is kept
                         % inside it.
C.semg_gate_release_s=1; % RELEASE window, deliberately 20x the attack -- see
                         % rcflex_semg_intent_gate.m. Wrongly closing the gate
                         % suppresses voluntary movement; wrongly holding it
                         % open only reverts to the ungated baseline, so the
                         % two errors do not cost the same. It also stops the
                         % confidence collapsing at the zero crossings of a
                         % reciprocating voluntary movement, where drive
                         % magnitude passes through zero but intent VELOCITY is
                         % at its maximum (measured: pitch confidence fell to
                         % 0.40 at each crossing of benchmark case 9).
C.semg_gate_warmup_s=1;  % the gate publishes full confidence for this long
                         % after start-up, so it can never assert an absence of
                         % voluntary drive before its own filters have settled.
                         % Set from the 2 % settling time of the 1 Hz 2-pole
                         % gate lowpass (~3/(pi*fc) = 0.95 s), not from any
                         % result. Its accumulators keep running underneath, so
                         % there is no release lag when it takes over.
C.semg_gate_floor_deg=.83;% half-power point of the confidence, as an equivalent
                         % voluntary wrist angle. Set by the SAME rule as
                         % C.semg_fuse_floor -- the geometric mean of the two
                         % measured classes -- from the drive magnitudes
                         % measured across all 20 benchmark cases: tremor-only
                         % <=0.331 deg, intent-bearing >=2.088 deg (6.3:1),
                         % geometric mean 0.831 deg. Measured BEFORE any
                         % closed-loop run and not fitted to one.
% --- Fast-movement detector (experiment E3). THE SINGLE SWITCH -------------
% Which observable may OPEN the gate early. The SLOW amplitude path is common
% to all three and is the only path that may ever CLOSE the gate, so this
% chooses the early-detection mechanism and nothing else.
%   'amplitude' the shipped two-speed gate: slow amplitude + FAST AMPLITUDE
%               path (C.semg_gate_fast_*). Reference arm, and the default, so
%               the shipped system is reproduced byte-for-byte unless this
%               field is changed.
%   'rate'      slow amplitude + RATE path. The fast amplitude path is
%               replaced by the time derivative of the same filtered drive.
%               Muscle drive RISES before it is LARGE, so a rate detector can
%               in principle fire earlier; it also differentiates noise, which
%               is why it gets its own floor rather than borrowing one.
%   'hybrid'    slow amplitude + fast amplitude + rate, combined with max().
%               Strictly more permissive than either, so it can only lower the
%               latency and can only raise false triggering.
% Compared under identical conditions by src/analysis/run_fast_movement_experiment.m.
C.semg_gate_detector='amplitude';
% The rate path is the DERIVATIVE OF THE EXISTING FAST PATH, deliberately: with
% the same corner frequency, the only difference between arm A's early path and
% arm B's is the differentiator, so a difference in the result is attributable
% to the differentiator and to nothing else. The field exists so the corner can
% be swept, and run_fast_movement_experiment sweeps it; it is not a free
% parameter that was tuned until the answer looked good.
C.semg_gate_rate_fc_Hz=C.semg_gate_fast_fc_Hz;
% Half-power point of the rate path, as an equivalent voluntary wrist ANGULAR
% VELOCITY. Set by the SAME geometric-mean rule as the two amplitude floors --
% sqrt(max tremor-only x min intent-bearing) over the 20 benchmark cases, with
% the 'reach' scenario HELD OUT so the threshold is not fitted to the case it
% is later judged on. Re-derive with calibrate_rcflex_semg_intent.m; it is NaN
% until that has been run, and the gate refuses to use an unset floor.
C.semg_gate_rate_floor_deg_s=NaN;
C.semg_gate_exponent=2;  % see rcflex_semg_intent_gate.m: matches the gate's
                         % output range to the measured 1.7 decades of class
                         % separation.
C.semg_gate_q_min=.01;   % floor on the intent-Q scale. Numerical safeguard
                         % only -- the gate's closed confidence is 0.0187
                         % (1/53) on the worst tremor-only case, so it does not
                         % bind on any measured case.
C.semg_gate_fixed_conf=.0187;% ONLY read by 'intent_gate_fixed'. Default equals
                         % the gate's measured CLOSED confidence, so the
                         % ablation applies the same Q reduction the sEMG arm
                         % applies on a tremor-only case -- but unconditionally,
                         % which is the whole point of the comparison.
% Calibration: envelope-difference volts -> equivalent voluntary joint angle
% (rad), per axis [roll; pitch]. Fitted by least squares on a TREMOR-FREE
% voluntary trial scored against the IMU-observable handle attitude, on a seed
% held out from the benchmark -- a procedure that is realisable on a real
% subject and needs no ground-truth decomposition, which is what keeps the
% interface usable when a real synchronised recording replaces the synthetic
% source. Re-derive with src/analysis/calibrate_rcflex_semg_intent.m.
C.semg_gate_cal_rad_per_V=[130.86;85.98];
% Trust gate (NIS-based). n_lo/n_hi bracket the expected NIS (=2, measurement dim).
C.ekf_nis_tau_s=.05;C.ekf_nis_lo=4;C.ekf_nis_hi=25;C.ekf_q_boost=50;
% Predictive authority manager.
C.auth_horizon_s=.02;C.auth_margin=.85;C.auth_rate_per_s=5;
% Joint-travel branch of the authority manager. THE SINGLE SWITCH for the
% repair recorded in rcflex_authority_manager.m: true is the repaired check,
% false pins it at full authority and reproduces the pre-repair behaviour for
% the controlled comparison in src/analysis/run_joint_travel_experiment.m.
% Nothing else should ever set it false.
C.auth_joint_check=true;
% --- Recorded human tremor input -----------------------------------------
% THE SINGLE SWITCH selecting a real tremor recording as the scenario's tremor
% component. Empty (the default) means the synthetic generator is used and
% nothing about the simulation changes, so every pre-existing result is
% reproduced exactly. Set it to a tremordb record name -- 'g2rof.rit', say --
% and run the 'recorded' scenario, and that recording's waveform drives the
% hand instead of a sine. The conditioning and the claim it supports are
% documented in src/control/rcflex_tremor_record.m; the amplitude is still
% given by the scenario, because the recording supplies shape only.
C.tremor_record='';
% Band the recording is conditioned over, in Hz. The lower edge removes laser
% transducer drift, the upper edge removes noise that integration would
% amplify. Both sit outside the 3-12 Hz range every record's dominant peak
% falls in, so neither edge is shaping the tremor itself.
C.tremor_record_band=[2 25];
% Seconds skipped at each end of a recording before a window is taken, so the
% zero-phase filter's edge transients and the transducer's settling never enter
% a scenario.
C.tremor_record_offset_s=5;
C.algorithm=algorithm;C.name=algorithm;
end
