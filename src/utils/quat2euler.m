function eul = quat2euler(q)
    % q: N x 4, [x y z w]
    x = q(:,1); y = q(:,2); z = q(:,3); w = q(:,4);
    % 计算滚转 roll (x-axis rotation)
    sinr_cosp = 2 .* (w .* x + y .* z);
    cosr_cosp = 1 - 2 .* (x.^2 + y.^2);
    roll = atan2(sinr_cosp, cosr_cosp);
    % 计算俯仰 pitch (y-axis rotation)
    sinp = 2 .* (w .* y - z .* x);
    % 数值稳健处理
    sinp(sinp > 1) = 1;
    sinp(sinp < -1) = -1;
    pitch = asin(sinp);
    % 计算偏航 yaw (z-axis rotation)
    siny_cosp = 2 .* (w .* z + x .* y);
    cosy_cosp = 1 - 2 .* (y.^2 + z.^2);
    yaw = atan2(siny_cosp, cosy_cosp);
    % 返回 N x 3, 按 [roll pitch yaw]
    eul = [roll, pitch, yaw];
end