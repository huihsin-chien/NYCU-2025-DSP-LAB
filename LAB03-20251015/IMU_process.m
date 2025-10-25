%% IMU pipeline: 校正 → 頻域/時域 → 濾波 → 積分與漂移修正
% data: n×6, columns = [acc_x acc_y acc_z gyro_x gyro_y gyro_z] in raw LSB

data = load("car1.mat");
data = data.data;

%% ===================== 使用者設定 =====================
fs  = 50;             % 取樣率 Hz
calib_sec = 3;        % 靜止校正秒數（移動前的時間長度）
ACC_SCALE  = 16384;   % LSB/g
GYRO_SCALE = 131;     % LSB/(deg/s)
g = 9.81;             % m/s^2
fc = 5;               % 低通截止頻率 (Hz) — 依你的動作頻帶調整，常見 3–10 Hz
butter_order = 4;     % 濾波器階數 (even)
% 若資料在檔案: data = r  round(calib_sec*fs));
acc_bias  = mean(acc_ms2(1:n_calib,:), 1);     % 3×1
gyro_bias = mean(gyro_dps(1:n_calib,:), 1);

% 扣除偏移（得到動態成分）
acc_dyn   = double(acc_ms2)  - acc_bias;               % m/s^2
gyro_corr = double(gyro_dps) - gyro_bias;              % deg/s

%% ===================== (2) 畫出 6 個訊號的時域/頻域 =====================
% 頻譜：顯示 0~Nyquist 的單邊幅度頻譜
L = n;
NFFT = 2^nextpow2(L);
f = fs*(0:(NFFT/2))/NFFT;

sig_all = [acc_dyn gyro_corr];
sig_names = {'ax (m/s^2)','ay (m/s^2)','az (m/s^2)', ...
             'gx (deg/s)','gy (deg/s)','gz (deg/s)'};

figure('Name','Time Domain','Color','w'); 
for k = 1:6
    subplot(6,1,k);
    plot(t, sig_all(:,k), 'LineWidth',1);
    grid on; xlabel('Time (s)'); ylabel(sig_names{k});
end

figure('Name','Frequency Domain','Color','w');
for k = 1:6
    X = fft(sig_all(:,k), NFFT)/L;
    P1 = 2*abs(X(1:NFFT/2+1));           % 單邊幅度
    subplot(6,1,k);
    plot(f, P1, 'LineWidth',1);
    xlim([0 fs/2]); grid on;
    xlabel('Frequency (Hz)'); ylabel(['|',sig_names{k},'|']);
end

%% ===================== (3) 低通濾波（Butterworth + 零相位） =====================
% 對加速度/角速度都可濾
Wn = fc/(fs/2); 
[b,a] = butter(butter_order/2, Wn, 'low');  % butter_order 為偶數
acc_f = filtfilt(b,a, acc_dyn);             % 濾後加速度 (m/s^2)
gyro_f = filtfilt(b,a, gyro_corr);          % 濾後角速度 (deg/s)

%% ===================== (4) 積分 → 漂移修正 → 再積分 =====================
% 4-1 速度 (梯形積分)
v_raw = zeros(n,3);
for i = 2:n
    v_raw(i,:) = v_raw(i-1,:) + 0.5*(acc_f(i,:)+acc_f(i-1,:))*(1/fs);
end

% 4-2 漂移修正（假設從 t_ss = calib_sec 到 t_se = 最後一點）
t_ss = calib_sec;
t_se = t(end);
idx_ss = find(t>=t_ss, 1, 'first');
idx_se = n;

% 令速度在 t_se 回到 0：線性趨勢扣除 v_d(t) = (v_e/(t_se-t_ss))*(t-t_ss)
v_e = v_raw(idx_se,:);                       % 結束時的三軸速度誤差
slope = v_e / (t_se - t_ss);                 % 1×3
td = t - t_ss;
td(td<0) = 0;                                % t<t_ss 之前不校正
v_drift = td .* slope;                       % n×3 線性漂移
v_corr  = v_raw - v_drift;                   % 校正後速度

% 4-3 位置 (再用梯形積分)
p = zeros(n,3);
for i = 2:n
    p(i,:) = p(i-1,:) + 0.5*(v_corr(i,:)+v_corr(i-1,:))*(1/fs);
end

%% ============== 視覺化：速度/位置（含與未含漂移修正） =================
figure('Name','Velocity (with/without drift correction)','Color','w');
axlbl = {'vx','vy','vz'};
for k = 1:3
    subplot(3,1,k);
    plot(t, v_raw(:,k), 'b-', 'LineWidth',1); hold on;
    plot(t, v_corr(:,k),'r-', 'LineWidth',1);
    yline(0,'k:'); grid on;
    xlabel('Time (s)'); ylabel([axlbl{k},' (m/s)']);
    legend('with drift','drift-corrected','Location','best');
end

figure('Name','Position (integrated from corrected velocity)','Color','w');
plbl = {'px','py','pz'};
for k = 1:3
    subplot(3,1,k);
    plot(t, p(:,k), 'LineWidth',1);
    grid on; xlabel('Time (s)'); ylabel([plbl{k},' (m)']);
end
sgtitle('Trajectory components');

%% ============== 小提醒 =================
% 1) fc 要依動作頻帶調：步行/手持多在 <~5 Hz；機械振動可再高。
% 2) 若運動只在中段，且最後仍在靜止狀態，上述線性漂移修正能有效讓 v(t_se)→0。
% 3) 若需要更穩健的 t_ss/t_se，可用閾值自動偵測：例如 |acc_dyn| 的能量超過門檻的起訖。
% 4) 若要地球座標位移，需做姿態解算(方向餘弦/四元數)把加速度旋回 ENU/NED 再積分。
