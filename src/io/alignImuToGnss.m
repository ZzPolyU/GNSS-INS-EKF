function imu_batches = alignImuToGnss(imu_cell, obs, leap_seconds)
% splitIMUbyGNSS 自动将 IMU 按 GNSS 观测时间分批
% imu_cell: 结构体，包含 imu_cell{1}.time, .acc, .quat
% obs: GNSS 观测结构数组，包含 .week, .tow
% leap_seconds: 当前年份闰秒数，例如 2021=18, 2025=19

if nargin < 3
    leap_seconds = 18; % 默认值，可修改
end

imu_time = imu_cell{1}.time; % Unix 秒
imu_acc  = imu_cell{1}.acc;
imu_quat = imu_cell{1}.quat;

% 去重并保证严格单调
[imu_time, ia] = unique(imu_time, 'stable'); 
imu_acc  = imu_acc(ia,:);
imu_quat = imu_quat(ia,:);

% 若仍有非严格单调，可微调
for i = 2:length(imu_time)
    if imu_time(i) <= imu_time(i-1)
        imu_time(i) = imu_time(i-1) + 1e-9;
    end
end

% GNSS GPST -> Unix
n = obs.n;
gps_unix_offset = seconds(datetime(1980,1,6) - datetime(1970,1,1));
gnss_unix = zeros(n,1);
for i = 1:n
    gps_seconds = obs.week(i)*7*24*3600 + obs.tow(i);
    gnss_unix(i) = gps_seconds + gps_unix_offset - leap_seconds;
end

% 分批
imu_batches = cell(n-1,1);
for i = 1:n-1
    t_start = gnss_unix(i);
    t_end   = gnss_unix(i+1);
    
    % 找区间 IMU 索引
    idx = find(imu_time >= t_start & imu_time <= t_end);
    
    % 插值加速度
    acc_start = interp1(imu_time, imu_acc, t_start, 'linear');
    acc_end   = interp1(imu_time, imu_acc, t_end, 'linear');
    
    % 插值四元数
    quat_start = slerpQuatAtTime(imu_time, imu_quat, t_start);
    quat_end   = slerpQuatAtTime(imu_time, imu_quat, t_end);
    
    % 拼接批次
    acc_batch  = [acc_start; imu_acc(idx,:); acc_end];
    quat_batch = [quat_start; imu_quat(idx,:); quat_end];
    time_batch = [t_start; imu_time(idx); t_end];
    
    imu_batches{i} = struct('time',time_batch,'acc',acc_batch,'quat',quat_batch);
end

end

%% 四元数 slerp 插值函数
function q_interp = slerpQuatAtTime(time_vec, quat_mat, t_query)
    [~, idx_s] = min(abs(time_vec - t_query));
    if time_vec(idx_s) <= t_query
        idx1 = idx_s; idx2 = min(idx_s+1,length(time_vec));
    else
        idx1 = max(idx_s-1,1); idx2 = idx_s;
    end
    t_frac = (t_query - time_vec(idx1)) / (time_vec(idx2) - time_vec(idx1));
    q1 = quat_mat(idx1,:); q2 = quat_mat(idx2,:);
    dotp = dot(q1,q2);
    if dotp < 0
        q2 = -q2; dotp = -dotp;
    end
    if dotp > 0.9995
        q_interp = (1-t_frac)*q1 + t_frac*q2;
        q_interp = q_interp/norm(q_interp);
        return
    end
    theta_0 = acos(dotp);
    sin_theta_0 = sin(theta_0);
    theta = theta_0 * t_frac;
    sin_theta = sin(theta);
    s0 = cos(theta) - dotp * sin_theta / sin_theta_0;
    s1 = sin_theta / sin_theta_0;
    q_interp = s0*q1 + s1*q2;
end