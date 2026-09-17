function [gain,R]=calibrate_rcflex_semg_intent(writeOutputs)
%CALIBRATE_RCFLEX_SEMG_INTENT Fit the sEMG intent-gate calibration constants.
% Produces C.semg_gate_cal_rad_per_V, and the evidence behind
% C.semg_gate_floor_deg. Re-run this after any change to the sEMG front end,
% the preprocessing chain or C.semg_gate_fc_Hz, and paste the printed constants
% back into rcflex_control_parameters.m -- they are frozen there in the same
% way every other gain in this project is frozen.
%
% THE PROCEDURE IS DELIBERATELY REALISABLE. The gain is fitted on a
% TREMOR-FREE VOLUNTARY trial, scoring the differential envelope against the
% HANDLE ATTITUDE -- a quantity the IMU measures directly. With no tremor
% present the handle attitude IS the voluntary angle, so no ground-truth
% intent/tremor decomposition is needed and nothing here depends on the source
% being synthetic. On a real subject this is one slow guided wrist movement
% with the tremor-free hand, or a rest-and-reach block; the same script runs
% against it once C.semg_source='file'. Contrast the omega oracle in E1, which
% reads simulation truth and is explicitly NOT realisable.
%
% The trial uses a seed held out from the benchmark sensor seeds (9100+k), so
% the constants are not fitted on the data they are later scored against.
%
% gain : 2x1 rad per envelope-difference volt, [roll; pitch].
% R    : table of the drive magnitudes measured across all 20 benchmark cases,
%        which is what sets the confidence floor.
if nargin<1;writeOutputs=true;end
C=rcflex_control_parameters('ekf_predictive');
C.semg_enabled=true;C=rcflex_prepare_control(C);
fs=C.outer_hz;

% --- Calibration trial ------------------------------------------------------
cal=rcflex_control_case('intent',8,8,0,4242,C);
u=drive_volts(cal,C);
t=(1:size(u,1))'/fs;
y=interp1(cal.time,cal.base(:,1:2),t);      % handle attitude == voluntary angle
w=t>1.5;                                     % after the scenario's opening ramp
gain=zeros(2,1);res=zeros(2,1);rr=zeros(2,1);
for a=1:2
 x=u(w,a);gain(a)=(x'*x)\(x'*y(w,a));
 res(a)=rms(y(w,a)-gain(a)*x);rr(a)=corr(x,y(w,a));
end
fprintf('\nCALIBRATION (tremor-free voluntary trial, held-out seed 4242)\n');
fprintf('  C.semg_gate_cal_rad_per_V=[%.2f;%.2f];\n',gain);
fprintf('  fit r = [%.3f %.3f], residual RMS = [%.3f %.3f] deg\n',rr,rad2deg(res));

% --- Drive magnitude across every benchmark case ----------------------------
% This is the measurement C.semg_gate_floor_deg is read from. It is taken on
% the SAME 20 cases and the SAME sensor seeds the benchmark uses, but it scores
% only the open-loop observable -- no controller runs here.
names={'sine','sine','sine','sine','sine','mixed','chirp','irregular','combined','intent','sine','sine','sine','step','sine','sine','sine','sine','sine','sine'};
freq=[4 8 12 6 10 8 8 8 8 8 8 8 8 8 8 5 7 9 11 20];
amp=[1 1 1 3 3 2 2 2 1 0 1 1 1 5 1 1 1 1 1 1];
% Both gate paths are measured, because the gate is two-speed: a SLOW path
% that decides when the gate may close, and a FAST path that can only open it
% (rcflex_semg_intent_gate.m). Each needs its own floor, set by the same
% geometric-mean rule at its own bandwidth -- the fast path passes more
% tremor-driven activation, and that has to be priced in rather than ignored.
rows=zeros(20,7);
for k=1:20
 dur=4;if k==10;dur=8;end
 D=rcflex_control_case(names{k},dur,freq(k),amp(k),9100+k,C);
 uk=drive_volts(D,C,C.semg.gate_lp);
 uf=drive_volts(D,C,C.semg.gate_lp_fast);
 tk=(1:size(uk,1))'/fs;wk=tk>1.5;
 d=rad2deg(uk(wk,:).*gain(:)');df=rad2deg(uf(wk,:).*gain(:)');
 hasIntent=rms(D.intent(D.time>1.5,1:2))>deg2rad(.1);
 rows(k,:)=[k rms(d(:,1)) rms(d(:,2)) rms(df(:,1)) rms(df(:,2)) any(hasIntent) 0];
end
tremorOnly=rows(rows(:,6)==0,2:3);
intentBearing=rows(rows(:,6)==1,2:3);
loMax=max(tremorOnly(:));hiMin=min(intentBearing(:));
floorDeg=sqrt(loMax*hiMin);
loMaxF=max(max(rows(rows(:,6)==0,4:5)));hiMinF=min(min(rows(rows(:,6)==1,4:5)));
floorFastDeg=sqrt(loMaxF*hiMinF);
conf=@(p,fl)(p.^2./(p.^2+fl^2)).^C.semg_gate_exponent;
rows(:,7)=max(conf(max(rows(:,2),rows(:,3)),floorDeg),conf(max(rows(:,4),rows(:,5)),floorFastDeg));
R=array2table(rows,'VariableNames',{'case_id','drive_roll_deg','drive_pitch_deg',...
 'fast_drive_roll_deg','fast_drive_pitch_deg','intent_present','confidence'});
R.scenario=string(names(:));R.frequency_Hz=freq(:);R.amplitude_deg=amp(:);
fprintf('\nDRIVE MAGNITUDE, all 20 benchmark cases (RMS, equivalent wrist angle)\n');
disp(R);
fprintf('SLOW path (%.2f Hz) -- decides when the gate may CLOSE\n',C.semg_gate_fc_Hz);
fprintf('  tremor-only      max = %.4f deg (%d cases)\n',loMax,size(tremorOnly,1));
fprintf('  intent-bearing   min = %.4f deg (%d cases)\n',hiMin,size(intentBearing,1));
fprintf('  separation           = %.1f : 1\n',hiMin/loMax);
fprintf('  geometric mean       = %.4f deg  -> C.semg_gate_floor_deg=%.2f;\n',floorDeg,floorDeg);
fprintf('  confidence: %.4f (closed) / %.4f (open)  => q_intent x %.4f ... %.4f\n',...
 conf(loMax,floorDeg),conf(hiMin,floorDeg),conf(loMax,floorDeg),conf(hiMin,floorDeg));
fprintf('FAST path (%.2f Hz) -- can only ever OPEN the gate\n',C.semg_gate_fast_fc_Hz);
fprintf('  tremor-only      max = %.4f deg\n',loMaxF);
fprintf('  intent-bearing   min = %.4f deg\n',hiMinF);
fprintf('  separation           = %.1f : 1\n',hiMinF/loMaxF);
fprintf('  geometric mean       = %.4f deg  -> C.semg_gate_fast_floor_deg=%.2f;\n',floorFastDeg,floorFastDeg);
fprintf('  confidence: %.4f (closed) / %.4f (open)\n',conf(loMaxF,floorFastDeg),conf(hiMinF,floorFastDeg));

if writeOutputs
 out=project_path('results','semg_intent_calibration.csv');
 writetable(R,out);fprintf('\nWrote %s\n',out);
end
end
function u=drive_volts(D,C,c)
%DRIVE_VOLTS Lowpassed agonist-antagonist envelope difference, per axis, at the
% outer-loop rate. Byte-for-byte the same chain rcflex_semg_intent_gate runs
% online, so the constants fitted here are the constants the controller uses.
% c selects which of the gate's two lowpass sections to apply.
if nargin<3;c=C.semg.gate_lp;end
F=rcflex_semg_features(D.semg,C,0);
d=[F(:,1)-F(:,2), F(:,3)-F(:,4)];
u=zeros(size(d));z=zeros(2,2);
for k=1:size(d,1)
 for a=1:2
  y=c(1)*d(k,a)+z(1,a);
  z(1,a)=c(2)*d(k,a)-c(4)*y+z(2,a);
  z(2,a)=c(3)*d(k,a)-c(5)*y;
  u(k,a)=y;
 end
end
end
