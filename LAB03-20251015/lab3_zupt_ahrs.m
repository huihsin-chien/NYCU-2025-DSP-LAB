% =====================================================================
% 步態分析程式 - IMU 數據處理與參數提取
% =====================================================================
clear; clc; close all;

%% ==================== 1. 數據載入與初始校正 ====================
file = fullfile("walk", "straight1_2.mat");
data = load(file).data;
data = scale_and_calib(data);

% 參數設定
g = 9.81;
fs = 50;                      % 取樣率 Hz
calib_sec_start = 0.1;        % 靜止校正起始秒數
calib_sec_end = 3;            % 靜止校正結束秒數
n = size(data, 1);
t = (0:n-1)' / fs;

% 靜止校正
n_calib_start = round(calib_sec_start * fs);
n_calib_end = round(calib_sec_end * fs);
pre_segment = data(n_calib_start:n_calib_end, :);
pre_segment_mean = mean(pre_segment, 1);
data = data - pre_segment_mean;

% 繪製未濾波數據
labels = {'a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)', ...
          '\omega_x (rad/s)', '\omega_y (rad/s)', '\omega_z (rad/s)'};

figure('Name', '未濾波加速度數據');
for k = 1:3
    subplot(3, 1, k);
    plot(t, data(:, k));
    grid on;
    ylabel(labels{k});
    if k == 1
        title('Unfiltered Acceleration Data');
    end
end

figure('Name', '未濾波陀螺儀數據');
for k = 4:6
    subplot(3, 1, k-3);
    plot(t, data(:, k));
    grid on;
    ylabel(labels{k});
    if k == 4
        title('Unfiltered Gyroscope Data');
    end
end

%% ==================== 2. 帶通/低通濾波 ====================
ax = data(:, 1); ay = data(:, 2); az = data(:, 3);
gx = data(:, 4); gy = data(:, 5); gz = data(:, 6);

% 加速度：帶通濾波 (0.2-5 Hz)
[bpB, bpA] = butter(2, [0.2 5] / (fs/2), 'bandpass');
ax = filtfilt(bpB, bpA, ax);
ay = filtfilt(bpB, bpA, ay);
az = filtfilt(bpB, bpA, az);

% 陀螺儀：低通濾波 (6 Hz)
[lpB, lpA] = butter(2, 6 / (fs/2), 'low');
gx = filtfilt(lpB, lpA, gx);
gy = filtfilt(lpB, lpA, gy);
gz = filtfilt(lpB, lpA, gz);

% 更新 data 矩陣
data(:, 1:6) = [ax, ay, az, gx, gy, gz];

% 繪製濾波後數據
figure('Name', '濾波後數據 加速度');
for k = 1:3
    subplot(3, 1, k);
    plot(t, data(:, k));
    grid on;
    ylabel(labels{k});
    if k == 1
        title('Filtered IMU Data');
    end
end

figure('Name', '濾波後數據 角速度');
for k = 4:6
    subplot(3, 1, k-3);
    plot(t, data(:, k));
    grid on;
    ylabel(labels{k});
    if k == 1
        title('Filtered IMU Data');
    end
end

%% ==================== 3. FSM 步態階段分析 ====================
% FSM 參數設定
TH_SWING_START = 30;   % Stage 0 -> 1 轉換閾值 (rad/s)
TH_SWING_END = -40;    % Stage 1 -> 2 轉換閾值 (rad/s)
WINDOW_SIZE = 5;       % 局部極值判斷的窗格大小
N = length(gz);
stages_output = zeros(N, 1);
current_stage = 0;

% FSM 狀態機
for i = 1:N
    start_j = max(1, i - WINDOW_SIZE);
    end_j = min(N, i + WINDOW_SIZE);
    
    switch current_stage
        case 0  % Stance Phase - 等待 Toe-Off
            if gz(i) > TH_SWING_START
                current_stage = 1;
            end
            
        case 1  % Swing Phase - 等待擺動減速
            if gz(i) < TH_SWING_END
                current_stage = 2;
            end
            
        case 2  % Swing Phase - 等待 Heel Strike
            if i > WINDOW_SIZE && i < N - WINDOW_SIZE
                is_local_max = all(gz(i) >= gz(start_j:end_j));
                if is_local_max
                    current_stage = 3;
                end
            end
            
        case 3  % Stance Phase - 等待 Mid-Stance
            if i > WINDOW_SIZE && i < N - WINDOW_SIZE
                is_local_min = all(gz(i) <= gz(start_j:end_j));
                if is_local_min
                    current_stage = 0;
                end
            end
    end
    
    stages_output(i) = current_stage;
end

% 繪製 FSM 結果
figure('Name', 'FSM 步態階段分割');
yyaxis left;
plot(t, gz, 'g-', 'LineWidth', 1.5, 'DisplayName', '\omega_z (rad/s)');
hold on;
stage_display = stages_output * 5;
plot(t, stage_display, 'b-', 'LineWidth', 2, 'DisplayName', 'Stage (0,1,2,3)');
ylabel('\omega_z (rad/s)');
ylim([-1.1*max(abs(gz)), 1.1*max(abs(gz))]);
grid on;

yyaxis right;
plot(t, az, 'r-', 'LineWidth', 1.5, 'DisplayName', 'a_z (m/s^2)');
ylabel('a_z (m/s^2)');
ylim([-1.1*max(abs(az)), 1.1*max(abs(az))]);

title('FSM 步態階段分割 (基於 \omega_z)');
xlabel('Time (s)');
legend('show', 'Location', 'SouthEast');
hold off;

% 輸出 Stance Phase 統計
stance_indices = (stages_output == 0 | stages_output == 3);
fprintf('\nFSM 分析結果:\n');
fprintf('Stance Phase (Stage 0 & 3) 總時間百分比: %.2f%%\n', ...
        sum(stance_indices) / N * 100);

%% ==================== 4. 步態週期檢測 ====================
% 找到所有 Heel Strike 點 (Stage 2 -> 3 轉換)
hs_indices = find(stages_output(1:end-1) == 2 & stages_output(2:end) == 3);
hs_times = t(hs_indices);
stride_times = diff(hs_times);

fprintf('\n======================================================\n');
fprintf('步態週期 (Stride Cycle) 分析\n');
fprintf('======================================================\n');
fprintf('偵測到的 Heel Strike 點數量: %d 個\n', length(hs_indices));
fprintf('完整步態週期數量: %d 步\n', length(stride_times));
if ~isempty(stride_times)
    fprintf('平均 Stride Time: %.4f s\n', mean(stride_times));
    fprintf('標準差: %.4f s\n', std(stride_times));
end

%% ==================== 5. AHRS 姿態融合 ====================
fusion_filter = ahrsfilter('SampleRate', fs);
gyroData = [gx, gy, gz];
accData = [ax, ay, az];
magData = zeros(size(accData));
quaternion_q = fusion_filter(accData, gyroData, magData);

% 轉換到 Earth Frame 並移除重力
Acc_Earth = zeros(N, 3);
% gravity_vector = [0, 0, g];
gravity_vector = [-g, 0, 0];
for i = 1:N
    Acc_Rotated = rotateframe(quaternion_q(i), accData(i, :));
    Acc_Earth(i, :) = Acc_Rotated - gravity_vector;
end

%% ==================== 6. ZUPT 積分與步態參數計算 ====================
dt = 1 / fs;
zupt_indices = (stages_output == 0 | stages_output == 3);

% 初始化速度和位移
Velocity = zeros(N, 3);
Position = zeros(N, 3);

% 記錄上一次 ZUPT 校正時的位置（用於位移校正）
last_zupt_position = [0, 0, 0];

% ZUPT 積分（改進版）
for i = 2:N
    % 積分計算速度和位移
    Velocity(i, :) = Velocity(i-1, :) + (Acc_Earth(i, :) + Acc_Earth(i-1, :)) * dt / 2;
    Position(i, :) = Position(i-1, :) + (Velocity(i, :) + Velocity(i-1, :)) * dt / 2;
    
    % ZUPT 校正（強化版）
    if zupt_indices(i) == 1
        % 速度歸零
        Velocity(i, :) = [0, 0, 0];
        
        % 垂直位移校正：在 Stance Phase 期間，垂直位移應該保持在地面高度
        % 將垂直位移（X軸）逐漸拉回到上次 ZUPT 的高度
        Position(i, 1) = last_zupt_position(1);
        
        % 更新水平位置記錄
        last_zupt_position = Position(i, :);
    end
end

%% ==================== 7. 步態參數提取 ====================
fprintf('\n======================================================\n');
fprintf('步態參數分析 (逐步)\n');
fprintf('======================================================\n');
fprintf(' 步數 | 起始時間 (s) | 結束時間 (s) | 步輻 (m) | 步高 (m)\n');
fprintf('------------------------------------------------------\n');

Stride_Data = struct('Length', {}, 'Height', {});
total_distance = 0;

% 注意：這裡遍歷所有的 HS 點
for k = 1:length(hs_indices)
    start_idx = hs_indices(k);
    
    % 最後一步到數據結尾
    if k == length(hs_indices)
        end_idx = N;
    else
        end_idx = hs_indices(k+1);
    end
    
    % 計算步輻（水平位移 Y-Z 平面）
    step_vec = Position(end_idx, 2:3) - Position(start_idx, 2:3);
    step_length = sqrt(step_vec(1)^2 + step_vec(2)^2);
    
    % 計算步高（垂直位移 X 軸最大值）
    swing_x_pos = Position(start_idx:end_idx, 1);
    max_height = max(swing_x_pos) - Position(start_idx, 1);
    
    total_distance = total_distance + step_length;
    
    fprintf(' %4d | %12.4f | %12.4f | %8.4f | %8.4f\n', ...
            k, t(start_idx), t(end_idx), step_length, max_height);
    
    Stride_Data(k).Length = step_length;
    Stride_Data(k).Height = max_height;
end

% 總結輸出
fprintf('\n======================================================\n');
fprintf('總結:\n');
fprintf('======================================================\n');
fprintf('偵測到的 Heel Strike 點數: %d 個\n', length(hs_indices));
fprintf('總步數: %d 步\n', length(Stride_Data));
fprintf('總步輻: %.4f 公尺\n', total_distance);
fprintf('平均步輻: %.4f 公尺\n', mean([Stride_Data.Length]));
fprintf('平均步高: %.4f 公尺\n', mean([Stride_Data.Height]));
fprintf('======================================================\n');

%% ==================== 8. 軌跡繪圖 ====================
figure('Name', 'ZUPT 步態軌跡與參數');

% 水平軌跡 (Y-Z 平面)
subplot(2, 1, 1);
plot(Position(:, 3), Position(:, 2), 'b-', 'LineWidth', 1.5, ...
     'DisplayName', '總軌跡 (Y-Z)');
hold on;
plot(Position(hs_indices, 3), Position(hs_indices, 2), 'ro', ...
     'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', '腳跟接觸點 (HS)');
xlabel('Earth Frame Z (m)');
ylabel('Earth Frame Y (m)');
title('ZUPT 估計水平位移軌跡 (Y-Z 平面)');
axis equal;
grid on;
legend('show');

% 垂直位移 (X 軸)
subplot(2, 1, 2);
plot(t, Position(:, 1), 'm-', 'LineWidth', 1.5, 'DisplayName', '垂直位移 X');
hold on;
plot(t(hs_indices), Position(hs_indices, 1), 'ro', ...
     'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'HS 點 X');
xlabel('時間 (s)');
ylabel('垂直位移 X (m)');
title('垂直位移 (X 軸) - 用於檢查步高');
grid on;
legend('show');