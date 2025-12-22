function [distances, segments, results] = demo1_car_imu_distance_estimation(data, Fs)
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

% 繪製未濾波資料fft
% 標籤
labels = {'a_y (m/s^2)', '\omega_y (rad/s)'};

% 資料長度
N = length(data(:,2));

% 計算 FFT 並中心化
Y_acc = fftshift(fft(data(:,2)));
Y_gyro = fftshift(fft(data(:,5)));

% 頻率軸：從 -Fs/2 到 Fs/2
f = (-N/2:N/2-1)*(Fs/N);
half_N = floor(N/2);
f = f(1:half_N);

% 畫圖
figure();

subplot(2, 1, 1);
plot(-f, abs(Y_acc(1:half_N))/N, 'r');
grid on;
xlabel('Frequency (Hz)');
ylabel(labels{1});
title('FFT of Unfiltered Acceleration Data');

subplot(2, 1, 2);
plot(-f, abs(Y_gyro(1:half_N))/N, 'b');
grid on;
xlabel('Frequency (Hz)');
ylabel(labels{2});
title('FFT of Unfiltered Gyroscope Data');


%% 2. 低通濾波
fc=10;
[b,a] = butter(20, fc/(Fs/2), 'low');
acc_filt = filtfilt(b,a, acc);
gyro_filt = filtfilt(b,a, gyro);  % 角速度也要濾波

figure();
subplot(2,1,1);
plot(acc(:,2), 'r','DisplayName','acc before lpf');
hold on;
plot(acc_filt(:,2), "b","DisplayName","acc after lpf");
grid on;
legend show;
title('Accelerometer y-axis');
subplot(2,1,2);
plot(gyro(:,2), 'r', 'DisplayName', 'gyro before lpf'); 
hold on;
plot(gyro_filt(:,2), 'b', 'DisplayName', 'gyro after lpf');
grid on;
legend show;
title('Gyroscope y-axis');

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
figure();

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

%% 7. 計算各段距離 (僅限 Y 軸) - 加強視覺化
distances = zeros(numel(start_idx), 1);
results = struct('segment', {}, 'time', {}, 'distance', {}, 'pos', {}, 'vel', {}, 'acc_lin', {}, 'vel_uncorrected', {}, 'pos_uncorrected', {});

% 修正：改為儲存完整時間軸的速度數據
vel_y_before_zupt = zeros(size(t));  % 初始化為完整時間軸長度
vel_y_after_zupt = zeros(size(t));   % 初始化為完整時間軸長度
pos_y_full = zeros(size(t));         % 儲存完整時間軸的位移

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
    
    %% 去除偏移 (僅 Y 軸)
    acc_corrected_y = acc_segment_y - acc_bias_y;  % 去除靜止偏移
    
    %% 高通濾波去除低頻漂移 (僅 Y 軸)
    fc_hp = 0.08;
    [b_hp, a_hp] = butter(2, fc_hp/(Fs/2), 'high');
    acc_lin_hp_y = filtfilt(b_hp, a_hp, acc_corrected_y);
    
    %% 雙重積分 (僅 Y 軸)
    t_seg = (0:numel(range)-1)' / Fs;
    
    % 1. 速度積分 (Y 軸)
    vel_y_uncorrected = cumtrapz(t_seg, acc_lin_hp_y);
    
    % 修正：儲存到完整時間軸
    vel_y_before_zupt(range) = vel_y_uncorrected;
    
    % 2. 速度漂移校正 (Y 軸 - 假設起終點速度為0, ZUPT)
    drift_line_vel = linspace(0, vel_y_uncorrected(end), size(vel_y_uncorrected,1))'; % ZUPT 校正線
    vel_y_corrected = vel_y_uncorrected - drift_line_vel; % 減去線性漂移
    vel_y = vel_y_corrected; % 使用校正後的速度進行下一階段積分
    
    % 修正：儲存到完整時間軸
    vel_y_after_zupt(range) = vel_y_corrected;
    
    % 3. 位置積分 (Y 軸) - 使用校正後的速度
    pos_y_uncorrected = cumtrapz(t_seg, vel_y); 
    pos_y = pos_y_uncorrected;
    
    % 儲存到完整時間軸
    pos_y_full(range) = pos_y;

    %% ========== 新增：繪製段落處理詳細流程 ========== %%
    figure('Name', sprintf('段落 %d 處理流程', k), 'Position', [100, 100, 1200, 900]);
    
    % 1. 原始加速度 vs 靜止段校正 vs 高通濾波 
    subplot(3,1,1);
    plot(t_seg, acc_segment_y, 'Color', [0.7 0.7 0.7], 'LineWidth', 1); hold on;
    plot(t_seg, acc_corrected_y, 'b', 'LineWidth', 1.2);
    plot(t_seg, acc_lin_hp_y, 'r', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 0.5);
    yline(acc_bias_y, 'g--', 'LineWidth', 0.5, 'DisplayName', '靜止偏移');
    grid on;
    title(sprintf('加速度靜止段校正與高通濾波 (段落 %d)', k));
    xlabel('時間 (s)'); ylabel('加速度 (m/s²)');
    legend('原始 Y 軸', '靜止段校正', '靜止段校正+高通濾波後', 'Location', 'best');
    
    % 4. 速度積分 (未校正 vs 校正)
    subplot(3,1,2);
    plot(t_seg, -vel_y_uncorrected, 'Color', [0.8 0.5 0.2], 'LineWidth', 1.5); hold on;
    plot(t_seg, -drift_line_vel, 'k--', 'LineWidth', 1);
    plot(t_seg, -vel_y_corrected, 'b', 'LineWidth', 2);
    yline(0, 'k-', 'LineWidth', 0.5);
    grid on;
    title('步驟3: 速度積分與 ZUPT 校正');
    xlabel('時間 (s)'); ylabel('速度 (m/s)');
    legend('原始積分', '漂移線', 'ZUPT 校正後', 'Location', 'best');
    
    % 6. 位置積分 (未校正 vs 校正)
    subplot(3,1,3);
    plot(t_seg, -pos_y_uncorrected, 'Color', [0.8 0.5 0.2], 'LineWidth', 1.5); hold on;
    yline(0, 'k-', 'LineWidth', 0.5);
    grid on;
    title('步驟4: 位置積分與 ZUPT 校正');
    xlabel('時間 (s)'); ylabel('位移 (m)');
    legend('原始積分','Location', 'best');

    %% ========== 視覺化結束 ========== %%

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

%% 8. 繪製多段速度比較圖與總位移累積圖（類似參考圖風格）
if numel(results) > 0
    figure('Name', '多段速度比較與總位移累積', 'Position', [100, 100, 1400, 800]);
    
    % 準備完整時間軸的數據
    vel_uncorrected_full = zeros(size(t));
    vel_corrected_full = zeros(size(t));
    
    for m = 1:numel(results)
        if ~isempty(results(m).pos)
            range_m = segments(m, 1):segments(m, 2);
            vel_uncorrected_full(range_m) = results(m).vel_uncorrected(:, 2);
            vel_corrected_full(range_m) = results(m).vel(:, 2);
        end
    end
    
    % 1. 速度校正前 - 標示漂移線（類似參考圖左）
    subplot(3,1,1);
    plot(t, vel_uncorrected_full, 'b', 'LineWidth', 1.5); hold on;
    yline(0, 'k-', 'LineWidth', 0.8);
    
    % 為每段繪製漂移趨勢線（紅色箭頭線）
    for m = 1:numel(results)
        if ~isempty(results(m).pos)
            idx1 = segments(m, 1);
            idx2 = segments(m, 2);
            
            % 計算該段的漂移線
            vel_start = vel_uncorrected_full(idx1);
            vel_end = vel_uncorrected_full(idx2);
            
            % 繪製紅色漂移趨勢線
            plot([t(idx1), t(idx2)], [vel_start, vel_end], 'r-', 'LineWidth', 2);
            
            % 在終點加箭頭標示
            arrow_size = 0.15;
            if vel_end > vel_start
                plot(t(idx2), vel_end, 'rv', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            else
                plot(t(idx2), vel_end, 'r^', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            end
            
            % 標註段落編號
            text(t(idx1) + (t(idx2)-t(idx1))*0.1, max(vel_uncorrected_full)*0.9, ...
                 sprintf('S%d', m), 'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
        end
    end
    
    title('速度校正前 - 顯示漂移趨勢（假設速度為0）', 'FontSize', 12);
    xlabel('時間 (s)'); ylabel('速度 (m/s)');
    grid on;
    xlim([0, t(end)]);
    legend('原始速度積分', '零線', '漂移趨勢線', 'Location', 'best');
    
    % 2. 速度校正後 - 歸零效果（類似參考圖右）
    subplot(3,1,2);
    plot(t, vel_corrected_full, 'b', 'LineWidth', 1.5); hold on;
    yline(0, 'k-', 'LineWidth', 0.8);
    
    % 標記各段起終點（應該都為0）
    for m = 1:numel(results)
        if ~isempty(results(m).pos)
            idx1 = segments(m, 1);
            idx2 = segments(m, 2);
            
            % 用垂直線分隔各段
            if m > 1
                xline(t(idx1), 'k--', 'LineWidth', 0.5, 'Alpha', 0.3);
            end
            
            % 標註段落編號與距離
            mid_idx = round((idx1 + idx2) / 2);
            max_vel = max(vel_corrected_full(idx1:idx2));
            if max_vel > 0.1
                text(t(mid_idx), max_vel*0.8, sprintf('S%d\n%.2fm', m, results(m).distance), ...
                     'FontSize', 9, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            end
        end
    end
    
    title('速度校正後 - ZUPT 歸零（起終點速度 ≈ 0）', 'FontSize', 12);
    xlabel('時間 (s)'); ylabel('速度 (m/s)');
    grid on;
    xlim([0, t(end)]);
    legend('ZUPT 校正後速度', '零線', 'Location', 'best');
    
    % 3. 階梯式累積位移圖
    subplot(3,1,3);
    
    % 建立階梯式累積位移
    cumulative_distance = 0;
    stair_time = [];
    stair_dist = [];
    
    % 初始點
    stair_time = [stair_time; 0];
    stair_dist = [stair_dist; 0];
    
    for m = 1:numel(results)
        if ~isempty(results(m).pos)
            idx1 = segments(m, 1);
            idx2 = segments(m, 2);
            
            % 靜止段（水平線）
            if m > 1 || idx1 > 1
                stair_time = [stair_time; t(idx1)];
                stair_dist = [stair_dist; cumulative_distance];
            end
            
            % 運動段結束，跳升一階
            cumulative_distance = cumulative_distance + results(m).distance;
            stair_time = [stair_time; t(idx2)];
            stair_dist = [stair_dist; cumulative_distance];
            
            % 標註該段貢獻
            mid_time = (t(idx1) + t(idx2)) / 2;
            mid_dist = cumulative_distance - results(m).distance / 2;
            text(mid_time, mid_dist, sprintf('  S%d: +%.2fm', m, results(m).distance), ...
                 'FontSize', 9, 'Color', [0.1 0.5 0.1], 'FontWeight', 'bold');
        end
    end
    
    % 最後補上結尾水平線
    stair_time = [stair_time; t(end)];
    stair_dist = [stair_dist; cumulative_distance];
    
    % 繪製階梯圖
    stairs(stair_time, stair_dist, 'b', 'LineWidth', 2); hold on;
    
    % 標記各段起終點
    for m = 1:numel(results)
        if ~isempty(results(m).pos)
            idx2 = segments(m, 2);
            plot(t(idx2), stair_dist(find(stair_time == t(idx2), 1)), ...
                 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        end
    end
    
    % 標註總距離
    text(t(end)*0.02, cumulative_distance*0.95, ...
         sprintf('總距離: %.3f m', sum(distances)), ...
         'FontSize', 11, 'Color', 'k', 'FontWeight', 'bold', ...
         'BackgroundColor', [1 1 0.8], 'EdgeColor', 'k');
    
    title('階梯式累積位移圖', 'FontSize', 12);
    xlabel('時間 (s)'); ylabel('累積位移 (m)');
    grid on;
    xlim([0, t(end)]);
    ylim([0, cumulative_distance*1.1]);
    legend('累積位移', '段落終點', 'Location', 'northwest');
    
    sgtitle(sprintf('多段運動分析總覽 (共 %d 段，總距離 %.3f m)', numel(results), sum(distances)), ...
            'FontSize', 14, 'FontWeight', 'bold');
end

%% 9. 輸出總結
fprintf('\n=== 距離估算結果 (僅 Y 軸) ===\n');
for k = 1:numel(distances)
    fprintf('段落 %d: %.3f m\n', k, distances(k));
end
fprintf('總距離: %.3f m\n', sum(distances));

%% 10. 繪製漂移校正比較圖與最終結果 (新的圖表佈局)
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
    
    figure(); 
    sgtitle(sprintf('IMU Y 軸漂移校正'), 'FontSize', 14, 'FontWeight', 'bold'); % 總標題
    
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
    
    % 2. 完整時間軸：Y 軸最終校正速度 (所有段落歸零後的總覽)
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
end

%% 11. 新增：繪製總體速度與位移關係圖
if numel(results) > 0
    figure('Name', '總體速度與位移分析', 'Position', [100, 100, 1400, 900]);
    
    % 子圖 1: 校正前後速度比較 (完整時間軸)
    subplot(3,1,1);
    plot(t, vel_y_before_zupt, 'r', 'LineWidth', 1.5, 'DisplayName', '校正前速度'); hold on;
    plot(t, vel_y_after_zupt, 'b', 'LineWidth', 1.5, 'DisplayName', '校正後速度');
    yline(0, 'k--', 'LineWidth', 1);
    
    % 標記運動段
    for m = 1:size(segments, 1)
        idx1 = segments(m, 1);
        idx2 = segments(m, 2);
        patch([t(idx1) t(idx2) t(idx2) t(idx1)], ...
              [min(vel_y_before_zupt)*1.1 min(vel_y_before_zupt)*1.1 max(vel_y_before_zupt)*1.1 max(vel_y_before_zupt)*1.1], ...
              'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    
    title('Y 軸速度比較 (校正前 vs 校正後)', 'FontSize', 12);
    xlabel('時間 (s)'); ylabel('速度 (m/s)');
    grid on;
    legend('Location', 'best');
    xlim([0, t(end)]);
    
    % 子圖 2: 校正後速度 (放大顯示)
    subplot(3,1,2);
    plot(t, -vel_y_after_zupt, 'b', 'LineWidth', 2); hold on;
    yline(0, 'k--', 'LineWidth', 1);
    
    % 標記各段起終點
    %{
    for m = 1:size(segments, 1)
        idx1 = segments(m, 1);
        idx2 = segments(m, 2);
        plot(t(idx1), vel_y_after_zupt(idx1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'HandleVisibility', 'off');
        plot(t(idx2), vel_y_after_zupt(idx2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
        
        % 標註段落編號
        mid_idx = round((idx1 + idx2) / 2);
        max_vel_seg = max(abs(vel_y_after_zupt(idx1:idx2)));
        if max_vel_seg > 0.05
            text(t(mid_idx), max_vel_seg*0.7, sprintf('S%d', m), ...
                 'FontSize', 10, 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', [0 0.5 0]);
        end
    end
    %}
    
    title('Y 軸速度 (經ZUPT 校正)', 'FontSize', 12);
    xlabel('時間 (s)'); ylabel('速度 (m/s)');
    grid on;
    xlim([0, t(end)]);
    legend('校正後速度', 'Location', 'best');
    
    % 子圖 3: 總體位移隨時間變化
    subplot(3,1,3);
    
    % 計算累積位移 (連續型)
    cumulative_pos = zeros(size(t));
    total_displacement = 0;
    
    for m = 1:numel(results)
        if ~isempty(results(m).pos)
            idx1 = segments(m, 1);
            idx2 = segments(m, 2);
            range_m = idx1:idx2;
            
            % 該段的相對位移
            segment_pos = results(m).pos(:, 2);
            
            % 加上前面所有段落的總位移作為偏移
            cumulative_pos(range_m) = segment_pos + total_displacement;
            
            % 更新總位移 (該段結束時的累積值)
            total_displacement = cumulative_pos(idx2);
            
            % 靜止段保持不變
            if m < numel(results)
                next_idx1 = segments(m+1, 1);
                if next_idx1 > idx2 + 1
                    cumulative_pos(idx2+1:next_idx1-1) = total_displacement;
                end
            end
        end
    end
    
    % 最後一段之後也保持不變
    if numel(results) > 0 && ~isempty(results(end).pos)
        last_idx = segments(end, 2);
        if last_idx < length(t)
            cumulative_pos(last_idx+1:end) = total_displacement;
        end
    end
    
    % 繪製累積位移
    plot(t, -cumulative_pos, 'b', 'LineWidth', 2); hold on;
    yline(0, 'k--', 'LineWidth', 1);
    
    % 標記各段終點
    for m = 1:numel(results)
        if ~isempty(results(m).pos)
            idx2 = segments(m, 2);
            plot(t(idx2), -cumulative_pos(idx2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            
            % 標註累積距離
            text(t(idx2), -cumulative_pos(idx2), sprintf('  %.2fm', abs(cumulative_pos(idx2))), ...
                 'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold');
        end
    end
    
    % 標記運動段
    for m = 1:size(segments, 1)
        idx1 = segments(m, 1);
        idx2 = segments(m, 2);
        patch([t(idx1) t(idx2) t(idx2) t(idx1)], ...
              [min(-cumulative_pos)*1.1 min(-cumulative_pos)*1.1 max(-cumulative_pos)*1.1 max(-cumulative_pos)*1.1], ...
              'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    end
    
    % 標註總距離
    text(t(end)*0.02, max(-cumulative_pos)*0.95, ...
         sprintf('總位移: %.3f m\n總距離: %.3f m', abs(total_displacement), sum(distances)), ...
         'FontSize', 11, 'Color', 'k', 'FontWeight', 'bold', ...
         'BackgroundColor', [1 1 0.8], 'EdgeColor', 'k');
    
    title('總位移變化', 'FontSize', 12);
    xlabel('時間 (s)'); ylabel('累積位移 (m)');
    grid on;
    xlim([0, t(end)]);
    legend('累積位移', '段落終點', '運動段', 'Location', 'best');
    
    sgtitle(sprintf('總體運動分析 (共 %d 段，總距離 %.3f m)', numel(results), sum(distances)), ...
            'FontSize', 14, 'FontWeight', 'bold');
end

end  % function結束