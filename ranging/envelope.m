sample_rate = 160000;

temperature = 23;
c = 331.3+0.6*temperature;                 % 聲速 (m/s)
dead_zone = 200e-6;      % 忽略發射直耦訊號 (200 us)

data = load("60.mat");
received_data = data.received_data(2:8192); % 清掉第一個異常值
received_data = received_data - mean(received_data); % 濾除DC直流偏壓

N = length(received_data);
t_rc = (1:N)/sample_rate; % 每個採樣點的時間

fc = 40000; % 載波 40 kHz

% Step 1: 下變頻
baseband = received_data .* exp(-1j*2*pi*fc*t_rc);

% Step 2: 設計低通濾波器 (截止頻率要大於訊號帶寬，但小於fs/2)
bw = 5000; % 假設訊號帶寬 5 kHz，可依實際情況調
[b,a] = butter(6, bw/(sample_rate/2)); 
baseband_lpf = filtfilt(b,a,baseband);

% Step 3: 取 envelope
env = abs(baseband_lpf);

% Plot
figure;
plot(t_rc, received_data); hold on;
plot(t_rc, env, 'r', 'LineWidth', 2);
legend('原始訊號','Envelope');
title('去除40k載波後的包絡');



% fft
figure;hold on;
fft_origin = fft(received_data);
fft_baseband = fft(baseband);
fft_basebandlpf = fft(baseband_lpf);
f = (0:N-1)*(sample_rate/N); % Frequency vector
plot(f, fft_origin, 'r');figure;
plot(f, fft_baseband, 'b');figure;
plot(f, fft_basebandlpf,'g');


env_norm = env / max(env); % 正規化

% Step 4: 找 peaks (時間軸上)
[pk, loc] = findpeaks(env_norm, t_rc, ...
                      'MinPeakHeight', 0.2, ...      % 過濾掉雜訊
                      'MinPeakDistance', 1e-4);      % 至少相隔 100 us

% Step 5: 找最大 peak
[~, max_idx] = max(pk);  
tof = loc(max_idx);      % 最大 peak 的時間
distance = (tof * c) / 2; % 單程距離 (除以2)

fprintf("Max peak at t = %.6f s → Distance = %.3f m\n", tof, distance);

% Plot
figure;
plot(t_rc, env_norm, 'r'); hold on;
plot(loc, pk, 'bo', 'MarkerFaceColor','b');       % 標記所有 peaks
plot(loc(max_idx), pk(max_idx), 'gs', 'MarkerFaceColor','g'); % 標記最大 peak
legend('Envelope','All peaks','Max peak');
xlabel('Time (s)');
ylabel('Normalized amplitude');
title('Envelope with max peak');