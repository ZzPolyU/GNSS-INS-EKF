classdef IMUModel < TransitionModel
    
    methods
        function obj = IMUModel()
            
        end
        
        function [x_pred, F, Q] = computePrediction(obj, x_prev, imu_data, dt)
            % imu_data (1*1 struct)
            p_prev = x_prev(1:3);
            v_prev = x_prev(4:6);
            ba_prev = x_prev(7:9);
            dtr_prev = x_prev(10);
            
            % 补偿IMU加速度偏置
            % 只取开始时刻的加速度
            a_B0 = imu_data.acc(1, :)';
            a_B0 = a_B0 - ba_prev;
            q_0 = imu_data.quat(1, :)';
            R_LB = obj.computeRLB(q_0(1), q_0(2), q_0(3), q_0(4));
            R_GL = obj.computeRGL(p_prev(1), p_prev(2), p_prev(3));
            temp = R_LB * a_B0 - [0; 0; 9.8];
            a_G = R_GL * temp;
            
            % 状态预测（欧拉积分）
            p_pred = p_prev + v_prev * dt;  % 位置更新
            v_pred = v_prev + a_G * dt;   % 速度更新
            ba_pred = ba_prev;
            dtr_pred = dtr_prev;
            
            % 输出
            x_pred = [p_pred; v_pred; ba_pred; dtr_pred];
            F = obj.computeF(R_GL*R_LB, dt);
            Q = obj.computeQ(dt);
        end
    end

    methods (Static)
        function R_LB = computeRLB(q_x, q_y, q_z, q_w)
            % R_LB
            q = [q_w, q_x, q_y, q_z];  % [w, x, y, z]
            q = q / norm(q);
            w = q(1); x = q(2); y = q(3); z = q(4);

            R_LB = [
                1-2*y^2-2*z^2,   2*x*y - 2*z*w,   2*x*z + 2*y*w;
                2*x*y + 2*z*w,   1-2*x^2-2*z^2,   2*y*z - 2*x*w;
                2*x*z - 2*y*w,   2*y*z + 2*x*w,   1-2*x^2-2*y^2
            ];
        end

        function R_GL = computeRGL(x_ecef, y_ecef, z_ecef)
            % R_GB = R_GL * R_LB
            % 输入：
            %   x_ecef, y_ecef, z_ecef - ECEF坐标 (m)
            %   q_x, q_y, q_z, q_w     - ROS orientation四元数分量（单位四元数）
            % 输出：
            %   R_GB                   - 3×3矩阵，Body→ECEF的转换矩阵
            
            % 计算R_GL矩阵
            a = 6378137.0;
            f = 1/298.257223563;
            e2 = 2*f - f^2;

            lon = atan2(y_ecef, x_ecef);
            rho = sqrt(x_ecef^2 + y_ecef^2);
            lat = atan2(z_ecef, rho);
            tol = 1e-8;
            for i = 1:10
                sin_lat = sin(lat);
                N = a / sqrt(1 - e2 * sin_lat^2);
                lat_new = atan2(z_ecef + N*e2*sin_lat, rho);
                if abs(lat_new - lat) < tol, lat = lat_new; break; end
                lat = lat_new;
            end

            sin_lon = sin(lon);
            cos_lon = cos(lon);
            sin_lat = sin(lat);
            cos_lat = cos(lat);
            R_GL = [
                -sin_lon, -sin_lat*cos_lon,  cos_lat*cos_lon;
                cos_lon,  -sin_lat*sin_lon,  cos_lat*sin_lon;
                0,        cos_lat,           sin_lat
            ];
        end

        function F = computeF(R_GB, dt)
            F = eye(10);
            F(1:3, 4:6) = eye(3) * dt;
            F(4:6, 7:9) = -R_GB * dt;
        end
        
        function Q = computeQ(dt)
            noise = [0.3; 0.3; 0.3; 0.15; 0.15; 0.15; 0.01; 0.01; 0.01; 0];
            noise = noise.^2;
            Q = diag(noise) * dt;
        end
    end
end
