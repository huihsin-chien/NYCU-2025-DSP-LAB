function [distances, segments, results] = imu_distance_estimation(data, Fs)
% IMU 直線距離估算 - 僅限 Y 軸 (第二欄)
% 輸入:
%   data - n×6 array [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
%   Fs   - 取樣頻率 (Hz)，預設 50
% 輸出:
%   distances - 各段距離 (m)
%   segments  - 各段起始與結束索引 [start_idx, end_idx]
%   results   - 詳細結果結構 (包含未校正的速度和位移數據)
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
acc_thresh = 0.5;
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
min_move_samples = round(0.9*Fs);
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
%% 6. 視覺化分割結果 (完整時間軸)
figure('Color','w', 'Position', [100 100 1200 800]);

% Y 軸加速度
subplot(2,1,1);
acc_y = acc_filt(:, 2); % 僅取 Y 軸加速度
plot(t, acc_y, 'b', 'LineWidth', 1); hold on;
y_min = min(acc_y);
y_max = max(acc_y);
for k = 1:numel(start_idx)
    patch([t(start_idx(k)) t(end_idx(k)) t(end_idx(k)) t(start_idx(k))], ...
          [y_min y_min y_max y_max], ...
          'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end
ylabel('Y 軸加速度 (m/s²)');
title('Y 軸加速度與運動段偵測結果');
grid on; legend('Y 軸加速度', '運動段');

% 角速度大小
subplot(2,1,2);
plot(t, gyro_smooth, 'r', 'LineWidth', 1); hold on;
yline(gyro_thresh, 'k--', '靜止閾值');
for k = 1:numel(start_idx)
    patch([t(start_idx(k)) t(end_idx(k)) t(end_idx(k)) t(start_idx(k))], ...
          [0 0 max(gyro_smooth) max(gyro_smooth)], ...
          'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end
ylabel('角速度大小 (rad/s)');
title('角速度 (用於靜止偵測)');
xlabel('時間 (s)');
grid on;
%% 7. 計算各段距離 (僅限 Y 軸)
distances = zeros(numel(start_idx), 1);
results = struct('segment', {}, 'time', {}, 'distance', {}, 'pos', {}, 'vel', {}, 'acc_lin', {}, 'vel_uncorrected', {}, 'pos_uncorrected', {});
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
    
    % *** 專注於 Y 軸 (第二欄) ***
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
    
    % 1. 速度積分 (Y 軸)
    vel_y_uncorrected = cumtrapz(t_seg, acc_lin_hp_y);
    
    % 2. 速度漂移校正 (Y 軸 - 假設起終點速度為0, ZUPT)
    drift_line_vel = linspace(0, vel_y_uncorrected(end), size(vel_y_uncorrected,1))'; % ZUPT 校正線
    vel_y_corrected = vel_y_uncorrected - drift_line_vel; % 減去線性漂移
    vel_y = vel_y_corrected; % 使用校正後的速度進行下一階段積分
    
    % 3. 位置積分 (Y 軸) - 使用校正後的速度
    pos_y_uncorrected = cumtrapz(t_seg, vel_y); 
    
    % 4. 位置漂移校正 (Y 軸 - 假設起終點位置變動為0)
    drift_line_pos = linspace(0, pos_y_uncorrected(end), size(pos_y_uncorrected,1))'; % ZUPT 校正線
    pos_y_corrected = pos_y_uncorrected - drift_line_pos; % 減去線性漂移
    pos_y = pos_y_corrected; % 最終校正後的位置

    % *** 為了保持 results 結構一致性，將 Y 軸結果存為 3D 向量 (X=0, Z=0) ***
    vel = [zeros(size(vel_y, 1), 1), vel_y, zeros(size(vel_y, 1), 1)];
    pos = [zeros(size(pos_y, 1), 1), pos_y, zeros(size(pos_y, 1), 1)];
    acc_lin_hp = [zeros(size(acc_lin_hp_y, 1), 1), acc_lin_hp_y, zeros(size(acc_lin_hp_y, 1), 1)];
    
    % 新增未校正的數據到 results 結構 (存為 3D 向量)
    vel_uncorrected = [zeros(size(vel_y_uncorrected, 1), 1), vel_y_uncorrected, zeros(size(vel_y_uncorrected, 1), 1)];
    pos_uncorrected = [zeros(size(pos_y_uncorrected, 1), 1), pos_y_uncorrected, zeros(size(pos_y_uncorrected, 1), 1)];
    
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
    results(k).vel_uncorrected = vel_uncorrected; % 未校正的速度
    results(k).pos_uncorrected = pos_uncorrected; % 未校正的位移
end
%% 8. 輸出總結
fprintf('\n=== 距離估算結果 (僅 Y 軸) ===\n');
for k = 1:numel(distances)
    fprintf('段落 %d: %.3f m\n', k, distances(k));
end
fprintf('總距離: %.3f m\n', sum(distances));

%% 9. 繪製漂移校正比較圖與最終結果 (新的圖表佈局)
if numel(results) > 0
    % 找到第一個有有效數據的段落
    k_first = find(~cellfun('isempty', {results.pos}), 1, 'first');

    if isempty(k_first)
        fprintf('無有效運動段落可供繪圖。\n');
        return; 
    end
    
    % 1. 準備完整的速度和位移向量 (將各段校正後的數據在總時間軸上拼接)
    vel_full_c_y = zeros(size(t));
    pos_full_c_y = zeros(size(t));

    for m = 1:numel(results)
        if ~isempty(results(m).pos)
            range_m = segments(m, 1):segments(m, 2);
            vel_full_c_y(range_m) = results(m).vel(:, 2); 
            pos_full_c_y(range_m) = results(m).pos(:, 2);
        end
    end

    % *** 僅繪製第一個運動段的漂移校正比較圖 (k=k_first) ***
    k = k_first;
    
    % 數據準備 (僅限第一個有效段落)
    vel_y_uc = results(k).vel_uncorrected(:, 2);
    vel_y_c = results(k).vel(:, 2);
    pos_y_uc = results(k).pos_uncorrected(:, 2);
    pos_y_c = results(k).pos(:, 2);
    t_seg = (0:size(vel_y_c,1)-1)'/Fs; % 繪圖時間軸 (相對運動段開始時間)
    
    figure('Color','w', 'Position', [150 150 1200 800]); 
    sgtitle(sprintf('IMU Y 軸漂移校正', k), 'FontSize', 14, 'FontWeight', 'bold'); % 總標題
    
    % 1. ZUPT 速度校正比較 (原始漂移 + 校正線 + 最終歸零)
    subplot(2,1,1);
    plot(t_seg, vel_y_uc, 'Color', [0.1 0.5 0.8], 'LineStyle', '-', 'LineWidth', 1.5); hold on; 
    
    % 繪製校正線 (linear drift component that was subtracted)
    drift_line = linspace(0, vel_y_uc(end), size(vel_y_uc, 1))';
    plot(t_seg, drift_line, 'k--', 'LineWidth', 1); 
    
    % 校正後 (紅色，強調歸零)
    plot(t_seg, vel_y_c, 'Color', [0.8 0.1 0.1], 'LineStyle', '-', 'LineWidth', 2); 
    
    yline(0, 'k-', 'LineWidth', 0.5); % 零線
    
    title(sprintf('段落 %d Y 軸速度漂移校正 (ZUPT)', k));
    xlabel('相對時間 (s)'); ylabel('速度 (m/s)');
    grid on;
    legend('原始積分 (帶漂移)', '線性漂移校正線', '校正後 (速度歸零)', 'Location', 'best');
    
    % 2. ZUPT 位移校正比較 (原始漂移 + 校正線 + 最終歸零)
    %{
    subplot(2,2,2);
    plot(t_seg, pos_y_uc, 'Color', [0.1 0.5 0.8], 'LineStyle', '-', 'LineWidth', 1.5); hold on;
    
    % 繪製校正線
    drift_line_pos = linspace(0, pos_y_uc(end), size(pos_y_uc, 1))';
    plot(t_seg, drift_line_pos, 'k--', 'LineWidth', 1); 
    
    % 校正後 (紅色，強調歸零)
    plot(t_seg, pos_y_c, 'Color', [0.8 0.1 0.1], 'LineStyle', '-', 'LineWidth', 2); 
    
    yline(0, 'k-', 'LineWidth', 0.5); % 零線
    
    title(sprintf('段落 %d Y 軸位移漂移校正', k));
    xlabel('相對時間 (s)'); ylabel('位移 (m)');
    grid on;
    legend('原始積分 (帶漂移)', '線性漂移校正線', '校正後 (位移歸零)', 'Location', 'best');
    %}

    % 3. 完整時間軸：Y 軸最終校正速度 (所有段落歸零後的總覽)
    subplot(2,1,2);
    plot(t, vel_full_c_y, 'b', 'LineWidth', 1.5); hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    
    % 標記所有運動段的起點和終點
    for m = 1:size(segments, 1)
        idx1 = segments(m, 1);
        idx2 = segments(m, 2);
        plot(t(idx1), vel_full_c_y(idx1), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        plot(t(idx2), vel_full_c_y(idx2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    end

    title('完整時間軸 Y 軸速度 (ZUPT 校正後)');
    xlabel('總時間 (s)'); ylabel('速度 (m/s)');
    grid on;
    
    %{
    % 4. 完整時間軸：Y 軸最終校正位移
    subplot(2,2,4);
    plot(t, pos_full_c_y, 'k', 'LineWidth', 1.5); hold on;
    yline(0, 'k--', 'LineWidth', 1);

    % 標記所有運動段的起點和終點
    for m = 1:size(segments, 1)
        idx1 = segments(m, 1);
        idx2 = segments(m, 2);
        plot(t(idx1), pos_full_c_y(idx1), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        plot(t(idx2), pos_full_c_y(idx2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r'); 
    end
    
    title(sprintf('完整時間軸 Y 軸位移最終結果 (總距離: %.3f m)', sum(distances)));
    xlabel('總時間 (s)'); ylabel('位移 (m)');
    grid on;
    %}
end
end
