function [v, p, v_raw] = integrate_imu_with_drift(data, fs)
% INTEGRATE_IMU_WITH_DRIFT
% 以梯形積分由加速度得到速度，再做線性漂移修正，最後再積分得到位置。
%
%   [v, p] = integrate_imu_with_drift(data, fs)
%
% 輸入:
%   data : n×6，欄位 [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
%          假設加速度單位為 m/s^2，且已完成必要的前處理 (如去重力/濾波)
%   fs   : 取樣頻率 (Hz)，預設 50
%
% 輸出:
%   v : n×3，三軸速度 (m/s)（經線性漂移修正）
%   p : n×3，三軸位置 (m)（由修正後速度積分）
%
% 方法:
% 1) v_raw  = ∫ a dt (梯形積分)
% 2) 令終點速度 v(t_end)=0，做線性漂移扣除
% 3) p      = ∫ v_corr dt (梯形積分)

    if nargin < 2 || isempty(fs)
        fs = 50;
    end
    if size(data,2) < 3
        error('data 需要至少 3 欄 (acc_x, acc_y, acc_z)');
    end

    acc = data(:,1:3);          % 使用前三欄加速度
    n = size(acc, 1);
    if n < 2
        v = zeros(n,3);
        p = zeros(n,3);
        return;
    end

    dt = 1/fs;
    t  = (0:n-1)' * dt;

    % -------- 4-1 速度 (梯形積分) --------
    v_raw = zeros(n,3);
    for i = 2:n
        v_raw(i,:) = v_raw(i-1,:) + 0.5*(acc(i,:) + acc(i-1,:)) * dt;
    end

    % -------- 4-2 線性漂移修正 --------
    % 假設從 t_ss=0 開始，終點速度應回到 0
    t_ss  = 0;                  % 若你有靜止校正秒數，可改成 calib_sec
    t_se  = t(end);
    v_e   = v_raw(end,:);               % 終點速度誤差
    slope = v_e / (t_se - t_ss);        % 1×3
    td    = t - t_ss;                   % n×1
    td(td < 0) = 0;

    % 產生 n×3 的線性漂移以扣除
    v_drift = td .* slope;              % 隱式展開：n×1 .* 1×3 → n×3
    v = v_raw - v_drift;                % 修正後速度

    % -------- 4-3 位置 (梯形積分) --------
    p = zeros(n,3);
    for i = 2:n
        p(i,:) = p(i-1,:) + 0.5*(v(i,:) + v(i-1,:)) * dt;
    end
end
