function [measurement,filtered]=rcflex_imu_sample(truth,filtered,t,H,seed)
% Same sampled/filtered/quantized sensor model for Simulink and benchmark.
if nargin<5;seed=20260910;end
fs=800;sigma=repmat([deg2rad(H.imu_gyro_noise_dps_sqrtHz)*ones(3,1);H.imu_acc_noise_g_sqrtHz*9.80665*ones(3,1)],2,1)*sqrt(fs/2);
stream=RandStream('mt19937ar','Seed',mod(seed+round(t*fs),2^32-1));
alpha=exp(-2*pi*100/fs);filtered=alpha*filtered+(1-alpha)*(truth+sigma.*randn(stream,12,1));
bias=repmat([deg2rad([.2;-.15;.1]);9.80665*[.005;-.008;.003]],2,1);
if isfield(H,'sensor_extra_bias');bias=bias+H.sensor_extra_bias;end
quant=repmat([deg2rad(1/65.6)*ones(3,1);9.80665/4096*ones(3,1)],2,1);
range=repmat([deg2rad(500)*ones(3,1);8*9.80665*ones(3,1)],2,1);
measurement=max(min(round((filtered+bias)./quant).*quant,range),-range);
end
