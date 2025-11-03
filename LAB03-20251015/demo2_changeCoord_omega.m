% Input 是已經經過 scale and calib 的訊號
clear;       % 清除 workspace 中的變數
clc;         % 清除 Command Window
close all;   % 關閉所有 figure 視窗
data = load("scale_and_calibed/straight2.mat").calib_data;
% plot_imu(data); %figure1-4
 g = 9.81;
fs  = 50;                     % 取樣率 Hz
calib_sec_start = 0.1;        % 靜止校正秒數
calib_sec_end = 3;            % 靜止校正秒數
n  = size(data,1);
t  = (0:n-1)'/fs;

%% ===================== (1) 校正：扣除傾斜面(重力/偏移)平均 =====================
% 取前 calib_sec 秒做平均，代表在傾斜平面上靜止時的量測
n_calib_start = round(calib_sec_start*fs);
n_calib_end = round(calib_sec_end*fs);
pre_segment = data(n_calib_start:n_calib_end,:);
pre_segment_mean = mean(pre_segment, 1);
disp(pre_segment_mean);

% 減掉平均
data = data - pre_segment_mean;
%check original data
% 假設 data 為 N×6（[ax ay az gx gy gz]）
N = size(data,1);

% 用現有 t，沒有就用樣本編號；若有 fs 也會用秒為單位
if exist('t','var')
    x = t;
    xlab = 'Time (s)';
elseif exist('fs','var')
    x = (0:N-1)'/fs;
    xlab = 'Time (s)';
else
    x = (1:N)';
    xlab = 'Sample';
end

labels = {'a_x (m/s^2)','a_y (m/s^2)','a_z (m/s^2)', ...
          '\omega_x (rad/s)','\omega_y (rad/s)','\omega_z (rad/s)'};

figure('Name','IMU signals (unfiltered)','Color','w');
for k = 1:6
    subplot(6,1,k);
    plot(x, data(:,k), 'LineWidth', 1);
    grid on;
    ylabel(labels{k});
    if k == 1, title('Unfiltered IMU Signals'); end
    if k < 6
        set(gca,'XTickLabel',[]); % 中間子圖不顯示 x 刻度標籤
    else
        xlabel(xlab);
    end
end

% 可選：讓所有子圖同步縮放/捲動
linkaxes(findall(gcf,'Type','axes'),'x');

data_unfilt = data;

%check original data
% plot_imu(data); %figure 5-8

%% ===================== (2) 帶通濾波（Butterworth Bandpass Filter） =====================
% --- 濾波器參數設計 ---
fs = 50;                     % 確認您的取樣率 (Hz)
f_low = 0.2;                 % 低頻截止頻率 (Hz)，用來移除飄移
f_high = 5;                  % 高頻截止頻率 (Hz)，用來移除雜訊
butter_order = 4;            % 濾波器階數。4階 (n=2 for butter func) 是一個穩定且常用的選擇

% --- 執行濾波 ---
% 計算正規化頻率 Wn = [f_low, f_high] / (fs/2)
Wn = [f_low, f_high] / (fs/2);

% 使用 'bandpass' 模式設計 Butterworth 濾波器
% 注意：butter(n, ...) 中 n=2 對應 4 階濾波器，因為 filtfilt 會將階數加倍
[b, a] = butter(butter_order/2, Wn, 'bandpass');

% 使用 filtfilt 進行零相位濾波，避免訊號產生時間延遲
data = filtfilt(b, a, data);

%%check original filtered data
% 假設 data 為 N×6（[ax ay az gx gy gz]）
N = size(data,1);

% 用現有 t，沒有就用樣本編號；若有 fs 也會用秒為單位
if exist('t','var')
    x = t;
    xlab = 'Time (s)';
elseif exist('fs','var')
    x = (0:N-1)'/fs;
    xlab = 'Time (s)';
else
    x = (1:N)';
    xlab = 'Sample';
end

labels = {'a_x (m/s^2)','a_y (m/s^2)','a_z (m/s^2)', ...
          '\omega_x (rad/s)','\omega_y (rad/s)','\omega_z (rad/s)'};

figure('Name','IMU signals (filtered)','Color','w');
for k = 1:6
    subplot(6,1,k);
    plot(t, data(:,k), 'LineWidth', 1);
    grid on;
    ylabel(labels{k});
    if k == 1, title('Filtered IMU Signals'); end
    if k < 6
        set(gca,'XTickLabel',[]); % 中間子圖不顯示 x 刻度標籤
    else
        xlabel(xlab);
    end
end

% 可選：讓所有子圖同步縮放/捲動
linkaxes(findall(gcf,'Type','axes'),'x');

%=====================(3) 積分角速度得到pitch ============================

% 取出 gyro（原本就是 deg/s）
omega_z_deg = data(:,6);          % deg/s
omega_z_rad = deg2rad(omega_z_deg);   % 轉成 rad/s

pitch_angle_rad = cumtrapz(t, omega_z_rad);       % 積分得到 rad
pitch_angle_deg = rad2deg(pitch_angle_rad);   % 需要畫成度數再轉一次即可

%積分角速度得到pitch

% ===================== (4) 繪製結果 ============================
figure('Name', 'Calculated Pitch Angle');
plot(t, pitch_angle_deg, 'LineWidth', 1.5);
title('Pitch Angle from Gyroscope Integration');
xlabel('Time (s)');
ylabel('Pitch Angle (degrees)');
grid on;
legend('Pitch Angle');

%=====================(5) CORRECTED GAIT INTERVAL DETECTION (with Preallocation) ====================
% Assume the following variables are in your workspace from your script:
% t:          time vector (in seconds)
% data:       your N x 6 IMU data (use the filtered data for better results)
% fs:         sampling frequency (e.g., 50 Hz)

% --- Step 1: Isolate the signal and define parameters ---
omega_z_deg_det = data(:,6);             % deg/s

min_peak_prominence = 50; % Increased value to avoid noise
min_peak_distance_sec = 0.5;
min_peak_distance_samples = round(min_peak_distance_sec * fs);


% --- Step 2: Find all the major negative peaks (t_push_off, the red dots) ---
[~, push_off_indices] = findpeaks(-omega_z_deg_det, ...
                           'MinPeakProminence', min_peak_prominence, ...
                           'MinPeakDistance', min_peak_distance_samples);
t_push_off_timestamps = t(push_off_indices);


% --- Step 3: Find the PREVIOUS zero-crossing (the "Purple X") ---

% --- MODIFICATION: Preallocate the array for speed ---
num_peaks = length(push_off_indices);
% Preallocate with NaNs (Not a Number). This is good practice because if a
% crossing is not found for a specific peak, it will remain NaN.
t_start_swing_timestamps = NaN(num_peaks, 1);

% Loop through each negative peak that we found
for i = 1:length(push_off_indices)
    
    % Define a search range: from the end of the previous step up to the current peak
    end_index = push_off_indices(i);
    if i > 1
        % Start searching from where the last push-off occurred
        start_index = push_off_indices(i-1);
    else
        % For the first peak, search from the beginning of the data
        start_index = 2; 
    end
    
    % Search BACKWARDS from the peak
    for j = end_index:-1:start_index
        % Look for the pattern: [positive or zero, negative]
        if omega_z_deg_det(j) < 0 && omega_z_deg_det(j-1) >= 0
            % Found it! The crossing is between index j-1 and j.
            % We'll use linear interpolation for accuracy.
            y1 = omega_z_deg_det(j-1); % Value before (positive)
            y2 = omega_z_deg_det(j);   % Value after (negative)
            x1 = t(j-1);       % Time before
            x2 = t(j);         % Time after
            
            t_cross = x1 - y1 * (x2 - x1) / (y2 - y1);
            
            % --- MODIFICATION: Use direct assignment instead of appending ---
            t_start_swing_timestamps(i) = t_cross;
            
            break; % Found the crossing, stop searching and move to the next peak
        end
    end
    % NOTE: The 'if ~found_crossing' block is no longer needed because the
    % array was pre-filled with NaNs.
end


% --- Step 4: Display and Visualize the Corrected Results ---
fprintf('Found %d swing phase intervals.\n\n', length(t_push_off_timestamps));
fprintf(' # | t_start_swing | t_push_off  | Duration (s)\n');
fprintf('---|---------------|-------------|--------------\n');
for i = 1:length(t_push_off_timestamps)
    if ~isnan(t_start_swing_timestamps(i))
        duration = t_push_off_timestamps(i) - t_start_swing_timestamps(i);
        fprintf('%2d |    %7.3f s   |   %7.3f s  |    %6.3f\n', i, t_start_swing_timestamps(i), t_push_off_timestamps(i), duration);
    else
        fprintf('%2d |      NaN      |   %7.3f s  |      NaN\n', i, t_push_off_timestamps(i));
    end
end

% Create the updated plot
figure('Name', 'Corrected Gait Interval Detection');
plot(t, omega_z_deg_det, 'b-', 'LineWidth', 1, 'DisplayName', 'ω_z');
hold on;
% Plot the push-off points (red dots)
plot(t_push_off_timestamps, omega_z_deg_det(push_off_indices), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 't_push_off');
% Plot the NEW start of swing points (the "Purple X")
plot(t_start_swing_timestamps, zeros(size(t_start_swing_timestamps)), 'mX', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 't_start_swing (Purple X)');
hold off;
grid on;
title('Corrected Swing Phase Intervals on ω_z Signal');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
legend;

%=====================(6) FINAL PITCH ANGLE CALCULATION (Simplified and Robust) ====================

% --- Step 1: 從帶通濾波後的 omega_z 計算角度 ---
% 由於 omega_z 已經經過帶通濾波，其直流偏移已被移除。
% 因此，直接積分後，只會剩下微小的線性飄移。
pitch_angle_rad_drifting = cumtrapz(t, omega_z_rad);


% --- Step 2: 使用 detrend 移除積分後產生的線性飄移 ---
% detrend 函數會計算訊號的整體線性趨勢（一條斜線），並將其從訊號中減去。
% 這是處理積分後殘餘飄移最直接、最穩定的方法。
pitch_angle_rad_corrected = detrend(pitch_angle_rad_drifting);


% --- Step 3: 將弧度轉換為角度以方便繪圖 ---
pitch_angle_deg_drifting = rad2deg(pitch_angle_rad_drifting); % 這是未經 detrend 的
pitch_angle_deg_corrected = rad2deg(pitch_angle_rad_corrected); % 這是最終校正結果


% --- Step 4: 視覺化最終結果 ---
figure('Name', 'Final Corrected Pitch Angle (Simplified Method)');

% 我們將原始的 data(:,6) 積分結果作為對比的紅線
% 注意：這裡的 omega_z 是濾波後的，所以紅線的飄移已經比最開始好很多了
plot(t, pitch_angle_deg_drifting, 'r-', 'DisplayName', 'After Integration (with minor drift)');
hold on;
plot(t, pitch_angle_deg_corrected, 'b-', 'LineWidth', 2, 'DisplayName', 'After Final Correction (detrended)');
hold off;
grid on;
title('Pitch Angle After Integration and Detrending');
xlabel('Time (s)');
ylabel('Pitch Angle (degrees)');
legend;

%% ===================== (7) Rotate -> ZUPT integration -> Step length by A->B =====================
% 1) 旋轉 body(ax,ay) 到地座標（以 pitch 角做 2D 旋轉）
theta_rad = deg2rad(pitch_angle_deg_corrected(:));
ax = data(:,1); ay = data(:,2); az = data(:,3);   % 已帶通
c = cos(theta_rad); s = sin(theta_rad);
a_vert = ax.*c - ay.*s;          % 垂直分量（含重力）
a_fwd  = ax.*s + ay.*c;          % 前進分量（我們要的）

% 初段靜止偏置移除（0.1~3 s）
i0b = max(1, round(0.1*fs)); 
i1b = min(numel(ax), round(3.0*fs));
a_vert = a_vert - mean(a_vert(i0b:i1b));
a_fwd  = a_fwd  - mean(a_fwd(i0b:i1b));

% 輕度高通去低頻漂移，不動到 1~3 Hz 步頻
[bHP,aHP] = butter(2, 0.1/(fs/2), 'high');
a_fwd_hp  = filtfilt(bHP, aHP, a_fwd);

% 2) ZUPT（用未帶通加速度模長 + 角速）→ 建立連續 v_fwd, s_fwd
acc_un    = data_unfilt(:,1:3);                 % 未帶通（只有扣均值）
acc_norm  = sqrt(sum(acc_un.^2,2));
gyro_deg  = data(:,4:6);                         % 帶通後角速（deg/s）
gyro_norm = vecnorm(gyro_deg,2,2);

zupt_gyro_th = 8;                 % deg/s
zupt_acc_th  = 0.25*9.81;         % ||a|-g| < 0.25g
is_zupt = (gyro_norm < zupt_gyro_th) & (abs(acc_norm - 9.81) < zupt_acc_th);
is_zupt = movmean(double(is_zupt), round(0.2*fs)) > 0.5;   % 0.2 s 平滑

% 找擺腿段（非 ZUPT）
swing_mask = ~is_zupt;
d = diff([0; swing_mask; 0]);
seg_st = find(d== 1); 
seg_en = find(d==-1)-1;
N = numel(t);
v_fwd = zeros(N,1);
s_fwd = zeros(N,1);

% 段內 0→0 速度修正；位移承接上一點（確保連續）
for k = 1:numel(seg_st)
    i0 = seg_st(k); i1 = seg_en(k);
    if i1 <= i0, continue; end
    v_seg = cumtrapz(t(i0:i1), a_fwd_hp(i0:i1));
    v_seg = v_seg - linspace(0, v_seg(end), numel(v_seg))';   % 速度尾端回 0
    v_fwd(i0:i1) = v_seg;

    s_seg = cumtrapz(t(i0:i1), v_seg);
    if i0 > 1, s_fwd(i0:i1) = s_fwd(i0-1) + s_seg;
    else,      s_fwd(i0:i1) = s_seg; 
    end
end

% 3) 以 A->B（start_swing -> push_off）取步長：step = s_fwd(B) - s_fwd(A)
%    將時間戳轉為樣本索引
idxA = round(interp1(t, 1:N, t_start_swing_timestamps, 'nearest', 'extrap')); 
idxB = round(interp1(t, 1:N, t_push_off_timestamps,   'nearest', 'extrap')); 
idxA = max(1, min(N, idxA(:)));
idxB = max(1, min(N, idxB(:)));

%    對每個 B 找「之前最近的 A」(A < B；你圖上的 A2→B2)
pairA = nan(size(idxB));
for k = 1:numel(idxB)
    p = find(idxA < idxB(k), 1, 'last');
    if ~isempty(p), pairA(k) = idxA(p); end
end
valid = ~isnan(pairA);
pairA = pairA(valid);
pairB = idxB(valid);

%    時長過濾（太短/太長的擺腿丟掉）
durAB = t(pairB) - t(pairA);
maskDur = (durAB >= 0.25) & (durAB <= 1.60);   % 可按資料調
pairA = pairA(maskDur);
pairB = pairB(maskDur);
durAB = durAB(maskDur);

%    計算步長
step_len_AB = s_fwd(pairB) - s_fwd(pairA);

%    若大多數仍為負，代表前進軸號誌反了 → 整體翻正
if median(step_len_AB) < 0
    s_fwd       = -s_fwd; 
    v_fwd       = -v_fwd; 
    a_fwd_hp    = -a_fwd_hp;
    step_len_AB = -step_len_AB;
end

% 4) 視覺化（時間窗口：以 A/B 範圍 ±1 s）
if ~isempty(pairA) && ~isempty(pairB)
    tmin = max(t(1), min([t(pairA); t(pairB)]) - 1);
    tmax = min(t(end), max([t(pairA); t(pairB)]) + 1);
else
    tmin = t(1); tmax = t(end);
end

figure('Name','Forward axis (Earth frame): a, v, s');
subplot(3,1,1); plot(t, a_fwd_hp, 'LineWidth',1.1); grid on;
ylabel('a_{fwd} (m/s^2)'); title('Forward axis (Earth frame)'); yline(0,'k-'); xlim([tmin tmax]);
subplot(3,1,2); plot(t, v_fwd, 'LineWidth',1.1); grid on;
ylabel('v_{fwd} (m/s)'); xlim([tmin tmax]);
subplot(3,1,3); plot(t, s_fwd, 'LineWidth',1.1); grid on;
ylabel('s_{fwd} (m)'); xlabel('Time (s)'); xlim([tmin tmax]);

% 在 s_fwd 上標出 A(×) 與 B(●) 並畫出每一步的綠線段
if ~isempty(pairA)
    figure('Name','Step length by A->B on s_{fwd}');
    plot(t, s_fwd, 'LineWidth',1.2); grid on; hold on;
    tA = t(pairA); tB = t(pairB);
    plot(tA, s_fwd(pairA), 'mx', 'MarkerSize',10, 'LineWidth',2, 'DisplayName','A=start\_swing');
    plot(tB, s_fwd(pairB), 'ro', 'MarkerFaceColor','r', 'DisplayName','B=push\_off');
    for i = 1:numel(pairA)
        plot([tA(i), tB(i)], [s_fwd(pairA(i)), s_fwd(pairB(i))], 'g-', 'LineWidth',2);
    end
    xlabel('Time (s)'); ylabel('s_{fwd} (m)');
    muAB = mean(step_len_AB,'omitnan'); sdAB = std(step_len_AB,'omitnan');
    title(sprintf('A->B step lengths (mean %.3f m, SD %.3f m)', muAB, sdAB));
    legend('s_{fwd}','A=start\_swing','B=push\_off','step segment'); xlim([tmin tmax]); hold off;

    % 步長時間序列（用段中點當時間）
    figure('Name','Step length A->B (series)');
    stem( (tA+tB)/2, step_len_AB, 'filled'); grid on; yline(0,'k-');
    xlabel('Time (s)'); ylabel('Step length (m)');
    title(sprintf('Step length A->B: Mean=%.3f m, SD=%.3f m', muAB, sdAB));
    xlim([tmin tmax]);

    % 列印清單
    fprintf('Step length by A->B (start_swing -> push_off):\n');
    fprintf(' # |    A(s)   |    B(s)   |  dur(s) | step_len(m)\n');
    fprintf('---|-----------|-----------|---------|------------\n');
    for i = 1:numel(step_len_AB)
        fprintf('%2d | %9.3f | %9.3f | %7.3f | %10.3f\n', ...
            i, tA(i), tB(i), durAB(i), step_len_AB(i));
    end
end




















