ecg = load()

Fs = 1000;              % 取樣率
ecg = ecg(:);           % 確保是 column vector
N   = length(ecg);
t   = (0:N-1)/Fs;
ecg = ecg - mean(ecg)   % 去除 DC 雜訊

%% FIGURE 1: Raw signal - Time Domain
figure(1);
plot(t, ecg); grid on
xlabel('Time (s)'); ylabel('Amplitude');
title('FIGURE 1: Raw ECG - Time Domain');

%% FIGURE 2: Raw signal - Frequency Domain
f = (-N/2:N/2-1)*(Fs/N);
ECG_fft = abs(fftshift(fft(ecg)))/N;
figure(2);
plot(f, ECG_fft); grid on
xlabel('Frequency (Hz)'); ylabel('Magnitude');
title('FIGURE 2: Raw ECG - Frequency Domain');

%% FIR Low-pass Filter Design (cutoff = 45 Hz)
cutoff = 45;                % Hz
order  = 200;               % FIR filter order (可依需要調整)
b = fir1(order, cutoff/(Fs/2), 'low');
a = 1;                      % FIR denominator

% Apply filter
ecg_filt = filtfilt(b,a,ecg);

%% FIGURE 3: Filtered signal - Time Domain
figure(3);
plot(t, ecg_filt); grid on
xlabel('Time (s)'); ylabel('Amplitude');
title('FIGURE 3: Filtered ECG (FIR Low-pass) - Time Domain');

%% FIGURE 4: Filtered signal - Frequency Domain
ECGf_fft = abs(fftshift(fft(ecg_filt)))/N;
figure(4);
plot(f, ECGf_fft); grid on
xlabel('Frequency (Hz)'); ylabel('Magnitude');
title('FIGURE 4: Filtered ECG (FIR Low-pass) - Frequency Domain');

%% FIGURE 5: Filter Frequency Response
[H,ff] = freqz(b,1,1024,Fs);
figure(5);
plot(ff, 20*log10(abs(H)),'LineWidth',1.2); grid on
xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
title('FIGURE 5: FIR Low-pass Filter Frequency Response');
ylim([-80 5]);

%% 指標計算
% Peak-to-peak voltage
Vpp = max(ecg_filt) - min(ecg_filt);

% R peak detection
[~,R_locs] = findpeaks(ecg_filt,'MinPeakProminence',0.5*std(ecg_filt), ...
                                   'MinPeakDistance',round(0.25*Fs));
RR = diff(R_locs)/Fs;            % 秒
RR_ms = RR*1000;                 % 毫秒
HR_bpm = 60./RR;                 % 即時心率
HR_mean = mean(HR_bpm);

% Print results
fprintf('\n===== ECG Results =====\n');
fprintf('Peak-to-peak voltage: %.3f\n', Vpp);
if ~isempty(RR)
    fprintf('Mean RR interval    : %.1f ms\n', mean(RR_ms));
    fprintf('Mean HR             : %.1f bpm\n', HR_mean);
else
    fprintf('Not enough R peaks detected.\n');
end
fprintf('=======================\n');
