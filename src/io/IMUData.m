classdef IMUData < handle
    properties
        timestamp           % 时间戳数组
        orientation         % 姿态四元数 [x, y, z, w]
        angular_velocity    % 角速度 [x, y, z]
        linear_acceleration % 线性加速度 [x, y, z]
        imu_data            % IMU数据结构体 [time, quat, gyro, acc]
    end
    
    methods
        function obj = IMUData(csv_file)
            obj.load_csv_data(csv_file);
        end

        function imu_data = get_imu_data(obj)
            imu_data = obj.imu_data;
        end
        
        function show_basic_info(obj)
            imu_cell = obj.imu_data;

            time = imu_cell.time;
            time = time - time(1);

            eul = quat2euler(imu_cell.quat);
            eul = rad2deg(eul);
            IMUData.plot31(time, eul, "time(s)", ["roll", "pitch", "yaw"], "姿态输出");
            
            acc = imu_cell.acc;
            IMUData.plot31(time, acc, "time(s)", ["acc_x", "acc_y", "acc_z"], "加速度计输出");
        end
    end

    methods (Access = private)
        function load_csv_data(obj, csv_file)
            fprintf('正在读取IMU数据文件: %s\n', csv_file);
            try
                % 读取CSV文件
                data = readtable(csv_file);
                fprintf('找到 %d 条IMU数据记录\n', height(data));
                
                % 提取时间戳
                obj.timestamp = data.x_time * 1e-09;
                fprintf('平均采样频率: %.1f Hz\n', 1/mean(diff(obj.timestamp)));
                
                % 提取姿态四元数 [x, y, z, w] 顺序
                obj.orientation = [...
                    data.field_orientation_x, ...
                    data.field_orientation_y, ...
                    data.field_orientation_z, ...
                    data.field_orientation_w];
                
                % 提取角速度 [x, y, z]
                obj.angular_velocity = [...
                    data.field_angular_velocity_x, ...
                    data.field_angular_velocity_y, ...
                    data.field_angular_velocity_z];
                
                % 提取线性加速度 [x, y, z]  
                obj.linear_acceleration = [...
                    data.field_linear_acceleration_x, ...
                    data.field_linear_acceleration_y, ...
                    data.field_linear_acceleration_z];
                
                fprintf('IMU数据读取完成!\n');
            catch ME
                error('读取IMU数据失败: %s', ME.message);
            end
            
            % 用结构体表示所有数据
            imu_struct = struct();
            imu_struct.time = obj.timestamp;
            imu_struct.quat = obj.orientation;
            imu_struct.gyro = obj.angular_velocity;
            imu_struct.acc = obj.linear_acceleration;
            obj.imu_data = imu_struct;
        end
    end

    methods (Static)
        function plot31(x, y, xlab, ylab, title)
            % 判断y的形状
            [rows, cols] = size(y);
            if cols < 3 || rows == 0
                error('y的形状不是N×3');
            end

            figure;
            subplot(311);
            plot(x, y(:, 1)); ylabel(ylab(1)); grid on;
            subplot(312);
            plot(x, y(:, 2)); ylabel(ylab(2)); grid on;
            subplot(313);
            plot(x, y(:, 3)); ylabel(ylab(3)); grid on;
            xlabel(xlab); 
            sgtitle(title);
        end
    end
end