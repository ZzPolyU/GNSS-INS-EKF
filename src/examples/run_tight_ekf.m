clc
clear
close all

% 加载路径
parentDir = fileparts(mfilename('fullpath'));
srcDir = fullfile(parentDir, '../..', 'src');
dataDir = fullfile(parentDir, '../..', 'data');
rtkDir = fullfile(parentDir, '../..', 'third_party/MatRTKLIB');
addpath(genpath(srcDir));
addpath(genpath(rtkDir));

imu_file = fullfile(dataDir, "xsens_imu.csv");
obs_file = fullfile(dataDir, "f9p_navi.obs");
nav_file = fullfile(dataDir, "brdm.rnx");

% 读取数据
imu_data = IMUData(imu_file);
gnss_data = GNSSData(obs_file, nav_file);
gnss_time = gnss_data.get_time_series();
imu_batches = alignImuToGnss(imu_data.get_cell(), gnss_time);

% 初始化
gnss_data.spp();
N = gnss_data.get_epoch();
% p0 = zeros(3, 1);
p0 = [-2.419174545866470e+06;	5.385342020290633e+06;	2.405652618703981e+06];
v0 = zeros(3, 1);
ba0 = zeros(3, 1);
dtr = 0; % m
x0 = [p0; v0; ba0; dtr];
P0 = 100000 * eye(10);

x_est = zeros(10, N);
x_est(:, 1) = x0;

imu_model = IMUModel();
gnss_model = TightGNSSModel();
ekf = EKF(x0, P0, imu_model, gnss_model);

% 主循环
for i = 2:N
    % 获取dt
    imu_batch = imu_batches{i-1};
    gnss_curr = gnss_data.get_tight_measurement(i);
    dt = imu_batch.time(end) - imu_batch.time(1);
    
    % predict
    ekf.predict(imu_batch, dt);

    % update
    ekf.update(gnss_curr, gnss_curr.P);

    % 记录数据
    [x, P] = ekf.getState();
    x_est(:, i) = x;
end