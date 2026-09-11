function C=rcflex_calibrate_control(C,H)
% Two-second stationary, LEVEL fixture calibration before releasing the spoon.
% Estimate from sensor samples, not from known noise/bias constants.
y=repmat([0;0;0;0;0;9.80665],2,1);f=y;data=zeros(12,1600);
for k=1:1600;[data(:,k),f]=rcflex_imu_sample(y,f,k/800,H,C.seed+500000);end
C=rcflex_prepare_control(C);m=mean(data,2);C.gyro_bias=m([1:3 7:9]);C.accel_bias=m([4:6 10:12])-repmat([0;0;9.80665],2,1);
end
