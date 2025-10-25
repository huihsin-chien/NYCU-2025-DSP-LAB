% ecg = load("MATLAB/hand_move_up_down.mat");
% ecg = load("MATLAB/hand_grasp.mat");
% ecg = load("MATLAB/foot_move.mat");
% ecg = load("MATLAB/hsecg_1.mat");
ecg = load("MATLAB/yhecg_5.mat");
% ecg = load("MATLAB/demo_1.mat");
ecg = ecg.received_data-mean(ecg.received_data);
Fs = 1000;              % 取樣率
ecg = ecg(:);           % 確保是 column vector
N   = length(ecg);
t   = (0:N-1)/Fs;

%% FIGURE 1: Raw signal - Time Domain
figure(1);
plot(t, ecg); grid on
xlabel('Time (s)'); ylabel('Amplitude');
title('FIGURE 1: Raw ECG - Time Domain');

%% FIGURE 2: Raw signal - Frequency Domain
f = (-N/2:N/2-1)*(Fs/N);
ECG_fft = abs(fftshift(fft(ecg)))/N;
figure(2);
plot(f, 20*log10(ECG_fft)); grid on
ylabel('Magnitude (dB)');
xlabel('Frequency (Hz)'); 
title('FIGURE 2: Raw ECG - Frequency Domain');

%% FIR Low-pass Filter Design (cutoff = 45 Hz)
% 參數

order_lp = 200;                 % 低通階數
order_bs = 200;                 % 帶阻階數（stop/high 建議偶數）
cutoff = 25;                    % 低通截止(Hz)
f1 = 26; f2 = 30;               % 帶阻範圍(Hz) —— 這段其實已在低通之外

% 可選視窗（不給就預設 Hamming）
win_lp = hamming(order_lp+1);
win_bs = hamming(order_bs+1);

% 設計濾波器
b_lp = fir1(order_lp, cutoff/(Fs/2), 'low',  win_lp, 'scale');   % 低通
b_bs = fir1(order_bs, [f1 f2]/(Fs/2), 'stop', win_bs, 'scale');  % 帶阻

% 串接：先低通 → 再帶阻（LTI 串接順序其實等價）
y1 = filtfilt(b_lp, 1, ecg);     % 零相位
ecg_filt = filtfilt(b_bs, 1, y1);





%% FIGURE 3: Filtered signal - Time Domain
figure(3);
plot(t, ecg_filt); grid on
xlabel('Time (s)'); ylabel('Amplitude');
title('FIGURE 3: Filtered ECG (FIR Low-pass) - Time Domain');

%% FIGURE 4: Filtered signal - Frequency Domain
ECGf_fft = abs(fftshift(fft(ecg_filt)))/N;
figure(4);
plot(f, 20*log10(ECGf_fft)); grid on
xlabel('Frequency (Hz)'); ylabel('Magnitude(dB)');
title('FIGURE 4: Filtered ECG (FIR Low-pass) - Frequency Domain');

%% FIGURE 5: Filter Frequency Response
[H,ff] = freqz(b_lp,1,1024,Fs);
figure(5);
plot(ff, 20*log10(abs(H)),'LineWidth',1.2); grid on
xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
title('FIGURE 5: FIR Low-pass Filter Frequency Response');
ylim([-80 5]);



%% 指標計算
% --- Inputs: ecg_filt (column), Fs ---

% 1) Bandpass to focus on QRS (steep, ~5–15 Hz works well across Fs)
[b,a] = butter(2, [5 15]/(Fs/2), 'bandpass');
y = filtfilt(b,a, ecg_filt);

% 2) Differentiate -> Square -> Moving window integrate (Pan–Tompkins style)
dy  = [y(1); diff(y)];          % keep length
sq  = dy.^2;
w   = max(1, round(0.12*Fs));   % ~120 ms integration window
mwi = movmean(sq, w);

% 3) Adaptive threshold on the integrated signal
%    Use a robust baseline + safety factor; adjust k if needed.
baseline = median(mwi);
noise_sd = 1.4826*median(abs(mwi - baseline));   % robust sigma (MAD)
k = 3;                                           % sensitivity factor
th = baseline + k*noise_sd;

% 4) Peak finding on the QRS-energized signal with physiological constraints
%    Refractory period 300 ms (avoid counting T after R);
%    QRS width ~50–150 ms at half height.
minDist   = round(0.30*Fs);          % refractory period
minWidth  = round(0.05*Fs);          % 50 ms
maxWidth  = round(0.15*Fs);          % 150 ms
[~, qrs_locs, qrs_w, qrs_prom] = findpeaks( ...
    mwi, ...
    'MinPeakHeight', th, ...
    'MinPeakDistance', minDist, ...
    'MinPeakWidth', minWidth, ...
    'MaxPeakWidth', maxWidth, ...
    'WidthReference','halfheight');

% (Optional) discard weak residuals if needed
keep = qrs_prom > 0.5*median(qrs_prom);
qrs_locs = qrs_locs(keep);

% 5) Snap each detection to the true R apex on the original ECG
%    Search a small window around each detection for the max amplitude.
snap_win = round(0.08*Fs);  % ±80 ms
R_locs = zeros(size(qrs_locs));
for i = 1:numel(qrs_locs)
    i0 = max(1, qrs_locs(i)-snap_win);
    i1 = min(length(ecg_filt), qrs_locs(i)+snap_win);
    [~, rel] = max(ecg_filt(i0:i1));
    R_locs(i) = i0 + rel - 1;
end

% 6) RR & HR
RR     = diff(R_locs)/Fs;         % seconds
HR_bpm = 60./RR;                  
HR_mean = mean(HR_bpm);

% 7) Peak-to-peak voltage (optional: on raw or filtered)
Vpp = max(ecg_filt) - min(ecg_filt);

% Print results
fprintf('\n===== ECG Results =====\n');
fprintf('Peak-to-peak voltage: %.3f\n', Vpp);
if ~isempty(RR)
    fprintf('Mean RR interval    : %.1f s\n', mean(RR));
    fprintf('Mean HR             : %.1f bpm\n', HR_mean);
else
    fprintf('Not enough R peaks detected.\n');
end
fprintf('=======================\n');
