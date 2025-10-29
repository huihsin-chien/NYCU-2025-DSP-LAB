function [distances, segments, results] = demo1_car_imu_distance_estimation(data, Fs)
% IMU 直線距離估算 - 僅限 Y 軸 (第二欄)
% 輸入:
%   data - n×6 array [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
%   Fs   - 取樣頻率 (Hz)，預設 50
% 輸出:
%   distances - 各段距離 (m)
%   segments  - 各段起始與結束索引 [start_idx, end_idx]
%   results   - 詳細結果結構
if nargin < 2
    Fs = 50;
end
%% 1. 資料準備
t = (0:size(data,1)-1)' / Fs;
acc = data(:,1:3);
gyro = data(:,4:6);
fprintf('=== IMU 距離估算 (僅限 Y 軸) ===\n');
fprintf('資料長度: %.2f 秒\n', t(end));
%% 2. 低通濾波
fc = 5;
[b,a] = butter(3, fc/(Fs/2), 'low');
acc_filt = filtfilt(b,a, acc);
gyro_filt = filtfilt(b,a, gyro);  % 角速度也要濾波
%% 3. 偵測靜止段 (用角速度更準確)
gyro_mag = sqrt(sum(gyro_filt.^2, 2));
gyro_smooth = movmean(gyro_mag, round(0.2*Fs));
% 靜止閾值 (角速度)
gyro_thresh = 0.2;  % rad/s
is_static = gyro_smooth < gyro_thresh;
% 加速度變化也要小
acc_var = movstd(sqrt(sum(acc_filt.^2, 2)), round(0.3*Fs));
acc_thresh = 2;
is_static = is_static & (acc_var < acc_thresh);
% 形態學處理：去除短暫雜訊
min_static_samples = round(0.5*Fs);  % 至少0.5秒靜止
is_static = medfilt1(double(is_static), round(0.2*Fs)) > 0.5;
%% 4. 找出運動段 (靜止段之間就是運動段)
is_moving = ~is_static;
% 找運動段邊界
d = diff([0; is_moving; 0]);
start_idx = find(d == 1);
end_idx = find(d == -1) - 1;
% 過濾太短的運動段
min_move_samples = round(0.6*Fs);
valid = (end_idx - start_idx) >= min_move_samples;
start_idx = start_idx(valid);
end_idx = end_idx(valid);
fprintf('初步偵測到 %d 個運動段\n', numel(start_idx));
%% 5. 合併相近的運動段
min_gap = round(1.0*Fs);
merged_s = [];
merged_e = [];
i = 1;
while i <= numel(start_idx)
    s = start_idx(i);
    e = end_idx(i);
    
    while i < numel(start_idx) && start_idx(i+1) - e < min_gap
        i = i + 1;
        e = end_idx(i);
    end
    
    merged_s = [merged_s; s];
    merged_e = [merged_e; e];
    i = i + 1;
end
start_idx = merged_s;
end_idx = merged_e;
fprintf('合併後剩餘 %d 個運動段\n\n', numel(start_idx));
segments = [start_idx, end_idx];
%% 6. 視覺化分割結果
figure('Color','w', 'Position', [100 100 1200 800]);
% 加速度大小
subplot(3,1,1);
acc_mag = sqrt(sum(acc_filt.^2, 2));
plot(t, acc_mag, 'b', 'LineWidth', 1); hold on;
for k = 1:numel(start_idx)
    patch([t(start_idx(k)) t(end_idx(k)) t(end_idx(k)) t(start_idx(k))], ...
          [min(acc_mag) min(acc_mag) max(acc_mag) max(acc_mag)], ...
          'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end
ylabel('加速度大小 (m/s²)');
title('運動段偵測結果');
grid on; legend('加速度', '運動段');
% 角速度大小
subplot(3,1,2);
plot(t, gyro_smooth, 'r', 'LineWidth', 1); hold on;
yline(gyro_thresh, 'k--', '靜止閾值');
for k = 1:numel(start_idx)
    patch([t(start_idx(k)) t(end_idx(k)) t(end_idx(k)) t(start_idx(k))], ...
          [0 0 max(gyro_smooth) max(gyro_smooth)], ...
          'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end
ylabel('角速度大小 (rad/s)');
title('角速度 (用於靜止偵測)');
grid on;
% 加速度變化
subplot(3,1,3);
plot(t, acc_var, 'm', 'LineWidth', 1); hold on;
yline(acc_thresh, 'k--', '變化閾值');
for k = 1:numel(start_idx)
    patch([t(start_idx(k)) t(end_idx(k)) t(end_idx(k)) t(start_idx(k))], ...
          [0 0 max(acc_var) max(acc_var)], ...
          'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end
ylabel('加速度標準差');
xlabel('時間 (s)');
title('加速度變化');
grid on;
%% 7. 計算各段距離 (僅限 Y 軸)
distances = zeros(numel(start_idx), 1);
results = struct('segment', {}, 'time', {}, 'distance', {}, 'pos', {}, 'vel', {}, 'acc_lin', {});
for k = 1:numel(start_idx)
    idx1 = start_idx(k);
    idx2 = end_idx(k);
    range = idx1:idx2;
    
    fprintf('=== 處理段落 %d (僅 Y 軸) ===\n', k);
    fprintf('時間範圍: %.2f - %.2f 秒 (%.2f 秒)\n', t(idx1), t(idx2), t(idx2)-t(idx1));
    
    %% 取前方靜止段作為校正基準
    static_win = round(1.0*Fs);
    static_start = max(1, idx1 - static_win);
    static_end = idx1 - 1;
    
    if static_end < static_start
        fprintf('  警告: 段落 %d 前方無靜止段，跳過\n', k);
        continue;
    end
    
    static_range = static_start:static_end;
    
    %% 計算靜止段的重力向量和偏移 (3D 仍然需要用於正確校準 Y 軸)
    acc_static = acc_filt(static_range, :);
    gravity_vec = mean(acc_static, 1);  % 1x3
    acc_bias = gravity_vec;
    
    fprintf('  靜止段重力 (3D): [%.3f, %.3f, %.3f], 大小=%.3f m/s²\n', ...
            gravity_vec(1), gravity_vec(2), gravity_vec(3), norm(gravity_vec));
    
    % *** NEW: 專注於 Y 軸 (第二欄) ***
    acc_segment_y = acc_filt(range, 2); % 僅取 Y 軸
    acc_bias_y = acc_bias(2);           % Y 軸的靜止偏移
    
    %% 去除偏移和重力 (僅 Y 軸)
    acc_corrected_y = acc_segment_y - acc_bias_y;  % 去除靜止偏移
    
    % 去除重力（假設重力方向不變）
    g_norm = gravity_vec / norm(gravity_vec);
    g_norm_y = g_norm(2); % Y 軸重力單位分量
    acc_lin_y = acc_corrected_y - 9.81 * g_norm_y;
    
    fprintf('  Y 軸靜止偏移: %.3f m/s²\n', acc_bias_y);
    fprintf('  Y 軸重力分量: %.3f * 9.81 m/s²\n', g_norm_y);
    
    %% 高通濾波去除低頻漂移 (僅 Y 軸)
    fc_hp = 0.08;
    [b_hp, a_hp] = butter(2, fc_hp/(Fs/2), 'high');
    acc_lin_hp_y = filtfilt(b_hp, a_hp, acc_lin_y);
    
    %% 雙重積分 (僅 Y 軸)
    t_seg = (0:numel(range)-1)' / Fs;
    
    % 速度積分 (Y 軸)
    vel_y = cumtrapz(t_seg, acc_lin_hp_y);
    
    % 速度漂移校正 (Y 軸 - 假設起終點速度為0)
    vel_y = vel_y - linspace(0, vel_y(end), size(vel_y,1))';
    
    % 位置積分 (Y 軸)
    pos_y = cumtrapz(t_seg, vel_y);
    
    % 位置漂移校正 (Y 軸 - 假設起終點位置變動為0)
    pos_y = pos_y - linspace(0, pos_y(end), size(pos_y,1))';

    % *** 為了保持 results 結構一致性，將 Y 軸結果存為 3D 向量 (X=0, Z=0) ***
    vel = [zeros(size(vel_y, 1), 1), vel_y, zeros(size(vel_y, 1), 1)];
    pos = [zeros(size(pos_y, 1), 1), pos_y, zeros(size(pos_y, 1), 1)];
    acc_lin_hp = [zeros(size(acc_lin_hp_y, 1), 1), acc_lin_hp_y, zeros(size(acc_lin_hp_y, 1), 1)];
    
    %% 計算總距離 (Y 軸上的位移總和)
    dist_y = abs(diff(pos_y)); % Y-axis step displacement magnitude
    total_dist = sum(dist_y); % 路徑距離，為 Y 軸位移的絕對值總和
    
    % 直線距離（起點到終點）
    straight_dist = abs(pos_y(end) - pos_y(1));

    distances(k) = total_dist;
    
    fprintf('  Y 軸路徑距離 (移動量): %.3f m\n', total_dist);
    fprintf('  Y 軸直線距離 (位移): %.3f m\n', straight_dist);
    fprintf('  路徑效率: %.1f%%\n\n', 100*straight_dist/max(total_dist,0.001));
    
    % 儲存結果
    results(k).segment = k;
    results(k).time = [t(idx1), t(idx2)];
    results(k).distance = total_dist;
    results(k).pos = pos;
    results(k).vel = vel;
    results(k).acc_lin = acc_lin_hp;
end
%% 8. 輸出總結
fprintf('\n=== 距離估算結果 (僅 Y 軸) ===\n');
for k = 1:numel(distances)
    fprintf('段落 %d: %.3f m\n', k, distances(k));
end
fprintf('總距離: %.3f m\n', sum(distances));
%% 9. 繪製軌跡圖
if numel(results) > 0
    figure('Color','w', 'Position', [150 150 1000 800]);
    
    for k = 1:numel(results)
        if isempty(results(k).pos), continue; end
        
        % 3D 軌跡 (實際僅 Y 軸有變化)
        subplot(2,2,1);
        plot3(results(k).pos(:,1), results(k).pos(:,2), results(k).pos(:,3), ...
              'LineWidth', 2); hold on;
        plot3(results(k).pos(1,1), results(k).pos(1,2), results(k).pos(1,3), ...
              'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot3(results(k).pos(end,1), results(k).pos(end,2), results(k).pos(end,3), ...
              'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        xlabel('X (m) [固定為 0]'); ylabel('Y (m)'); zlabel('Z (m) [固定為 0]');
        title('Y 軸位移 (3D 視覺化)');
        grid on; axis equal;
        
        % XY 平面軌跡 (固定在 Y 軸上)
        subplot(2,2,2);
        plot(results(k).pos(:,1), results(k).pos(:,2), 'LineWidth', 2); hold on;
        plot(results(k).pos(1,1), results(k).pos(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot(results(k).pos(end,1), results(k).pos(end,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        xlabel('X (m) [固定為 0]'); ylabel('Y (m)');
        title('Y 軸位移 (XY 平面)');
        grid on; axis equal;
        
        % Y 軸速度
        subplot(2,2,3);
        vel_y = results(k).vel(:, 2); % 只取 Y 軸速度
        plot((0:numel(vel_y)-1)/Fs, vel_y, 'b', 'LineWidth', 1.5); hold on;
        xlabel('時間 (s)'); ylabel('速度 (m/s)');
        title(sprintf('段落 %d Y 軸速度', k));
        grid on;
        
        % Y 軸線性加速度
        subplot(2,2,4);
        acc_lin_y = results(k).acc_lin(:, 2); % 只取 Y 軸線性加速度
        plot((0:size(acc_lin_y,1)-1)/Fs, acc_lin_y, 'b', 'LineWidth', 1);
        xlabel('時間 (s)'); ylabel('加速度 (m/s²)');
        title(sprintf('段落 %d Y 軸線性加速度', k));
        grid on;
    end
    
    subplot(2,2,1); hold off;
    subplot(2,2,2); hold off;
    subplot(2,2,3); hold off;
end
end
