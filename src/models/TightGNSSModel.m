classdef TightGNSSModel < ObservationModel
    properties
        cfg;
    end

    methods
        function [z_pred, H, R] = computeObservation(obj, x_pred, z_meas)
            % x_pred: [x;y;z;Delta]  % Delta=接收机钟差(米)
            % z_meas: struct with fields (来自 GNSSData.get_tight_measurement)
            %   .P     (M x 1)   % 已预处理的伪距：P* = rho + resPc - TGD
            %   .SV    (3 x M)   % 卫星ECEF（米）
            %   .elev  (M x 1)   % 仰角（弧度）
            %   .cn0   (M x 1)   % C/N0（dB-Hz）
            %   .tgd   (M x 1)   % TGD（米） — 已在上游扣除，这里不再使用
            %   .sys   (M x 1)   % 系统码（若后续要做ISB可用；此处未使用）
            % SV = z_meas.SV;
            % xr  = x_pred(1:3);
            % delta = x_pred(4);
            % M = size(SV, 2);
            % H = zeros(M, 10);
            % z_pred = zeros(M, 1);
            % for i = 1 : M
            %     sv = SV(:, i);
            %     rho = sqrt(sum((sv - xr).^2));
            %     z_pred(i, 1) = rho + delta;
            %     H(i, 1:3) = - (sv - xr) / rho;
            %     H(i, 10) = 1;
            % end

            X      = x_pred(1:3);
            Delta  = x_pred(4);
            SV     = z_meas.SV;                 % 3xM
            M      = size(SV,2);

            % ---------- 预测量测 z_pred ----------
            % h_i = ||SV_i - X|| + Delta
            X = X(:);
            d   = SV - X;                       % 3xM
            rho = sqrt(sum(d.^2,1)).';          % Mx1
            z_pred = rho + Delta;

            % ---------- 雅可比 H ----------
            % ∂h/∂X = - (SV_i - X)/rho_i
            rho_safe = max(rho, 1e-6);
            u = (d ./ rho_safe.').';            % Mx3, 每行=(SV_i - X)/rho_i
            H = zeros(M, 10);
            H(:,1:3) = -u;                      % 注意与残差定义一致：r = z - h
            H(:,10)   = 1;                      % 钟差(米)
            
            % ---------- 噪声协方差 R (Herrera加权：sigma_i^2 = 1/W_i) ----------
            % 若 cfg 未提供，给出合理默认参数
            Tref = obj.getfield_or(obj.cfg, "W_T", 45);   % dB-Hz
            a    = obj.getfield_or(obj.cfg, "W_a", 10);
            A    = obj.getfield_or(obj.cfg, "W_A", 30);
            Fref = obj.getfield_or(obj.cfg, "W_F", 10);
            
            Wi = zeros(M,1);
            for i = 1:M
                el  = z_meas.elev(i);
                cn0 = z_meas.cn0(i);
                if ~isfinite(el), el = 1e-3; end
                if ~isfinite(cn0), cn0 = Tref; end
            
                term1 = 1 / (sin(max(el,1e-3))^2);
                term2 = 10^(-(cn0 - Tref)/a);
                kappa = (A * 10^(-(Fref - Tref)/a) - 1);
                term3 = (kappa * (cn0 - Tref) / max(Fref - Tref, 1e-6) + 1);
            
                Wi(i) = term1 * term2 * term3;
            end
            sigma2 = 1 ./ max(Wi, eps);
            R = diag(sigma2);
        end
    end

    methods (Static)
        function v = getfield_or(s, name, default)
            if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                v = s.(name);
            else
                v = default;
            end
        end
    end
end