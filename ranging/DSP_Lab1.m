%% ================= Local Functions =================
function y_demod = demodulation(x, Fs, fc)
%DEMUDOLATION 將實數輸入與 e^{-j 2π f_c t} 相乘做下變頻
% Input
%   x  : time seires
%   Fs : sampling rate (Hz)
%   fc : 載波頻率 (Hz)
% Output
%   y_demod : demodulated sequence

    N = numel(x);
    t = (0:N-1)/Fs;
    y_demod = x(:).' .* exp(-1j*2*pi*fc*t);  
    y_demod = y_demod(:);                   
end

function env = envelope_lowpass(y_demod, Fs, fc_lp)
% 先low pass 再取 abs 得到 envelop
% Input
%   y_demod : demulated signal
%   Fs      : sampling rate
%   fc_lp   : low pass 截止頻率
% Output
%   env     : envelop (abs(lp))
    y_lpf = lowpass(y_demod, fc_lp, Fs);
    env   = abs(y_lpf);
end

function [idx_onset, idx_peak] = find_peak_onset(env)
% 1. 找 envelope 中的所有局部峰值 (可能有多個peak)
% 2. 從局部峰值當中找到最大峰值的位置
% 3. 從最大峰值往回找 onset (斜率從負轉正的地方)
% Input
%   env : envelope time series (N×1)
%
% Output
%   idx_onset index of onset
%   idx_peak: index of max peak

    % --- 用 findpeaks 找所有峰值 ---
    [pks, locs] = findpeaks(env);

    % 如果訊號內有至少一個峰值
    if ~isempty(pks)
        [~, i_max] = max(pks);
        idx_peak   = locs(i_max);
    else
        % 如果沒有偵測到峰值，就退而求其次取最大值
        [~, idx_peak] = max(env);
    end

    % --- 往回找 onset ---
    dy = diff(env);
    last_nonpos = find(dy(1:idx_peak-1) <= 0, 1, 'last');
    if isempty(last_nonpos)
        idx_onset = 1;
    else
        idx_onset = last_nonpos + 1;
    end
end

function dist = distance(idx, Fs, start_idx, offset_cm)
% Input
%   idx : either onset or peak
%   start_idx the start of transmitted signal
%   offset_cm: the offset netween the board and the ultrasound module
% Output
%   dist : distance betweeb the board and the measured object
    dist = ( (idx-start_idx)*1/Fs*(331+0.6*25)*100 + offset_cm )/2; % in cm
end

close all;
% ---------- load data ----------
file_name = "demo20.mat";
data = load(file_name);

% --------- remove 1st sample and DC component ---------
received_data = data.received_data(2:8192);
received_data = received_data-mean(received_data);
transmitted_data = data.tx_received_data(2:8192);
transmitted_data = transmitted_data-mean(transmitted_data);

% --------- params ---------
Fs = 160000;
L = length(received_data);
t_rc = (0:L-1)/Fs;
fc = 40503; % carrier frequency 載波頻率
f_axis = Fs/L*(-L/2:L/2-1);
fc_lp = 10000; % low pass filter 截止頻率

% Figure 1. Plot T and X 
figure(1);
plot(t_rc,transmitted_data, 'r'); hold on;
plot(t_rc,received_data, 'b'); hold off;
legend('Transmitted', 'Received');
title('Time domain raw signal')
subtitle(file_name);

% --------- demodulation (frequency shift for fc: carrier frequency) ----------
demod_r = demodulation(received_data, Fs, fc);

% Figure 2. Recieved data: raw vs demodulated (Frequency domain)
figure(2);
fft_received_data = abs(fftshift(fft(received_data))); 
fft_demod = abs(fftshift(fft(demod_r)));
%plot(f_axis, fft_received_data, 'b'); %hold on;
plot(f_axis, fft_demod, 'r'); hold off;
%legend('raw recieved', 'demodulated');
%title('Recieved data: raw vs demodulated (Frequency domain)')
title('Demodulated data (Frequency domain)')
subtitle(file_name);

% Figure 3. Reveived data Traw vs demodulated (Time domain)
figure(3);
plot(t_rc, received_data, 'b'); hold on;
plot(t_rc, demod_r, 'r'); hold off;
legend('raw recieved', 'demodulated');
title('Recieved data: raw vs demodulated (Time domain)')
subtitle(file_name);

% ------------ Low pass filter (取envelop) -------------
env = envelope_lowpass(demod_r, Fs, fc_lp);

% Figure 4. Demodulated data vs. Envelop (after Low pass filter) (Frequency Domain)
figure(4);
plot(f_axis, abs(fftshift(fft(demod_r))), 'r'); hold on;
plot(f_axis, abs(fftshift(fft(env))), 'k', 'LineWidth',2); hold off;
legend('demodulated', 'env');
title('Demodulated data vs. Envelop (after Low pass filter) (Frequency Domain)')
subtitle(file_name);

% Figure 5. Demodulated data vs. Envelop (after Low pass filter) (Time Domain)
figure(5);
plot(t_rc, demod_r, 'g'); hold on;
plot(t_rc, env, 'k'); hold off;
legend('demodulated', 'env');
title('Demodulated data vs. Envelop (after Low pass filter) (Time Domain)')
subtitle(file_name);

% ---------- Find peak -------------
[idx_onset, idx_peak] = find_peak_onset(env);

% Figure 6. Finding onset and peak points
figure(6);
plot(env,'k'); hold on;
plot(idx_peak, env(idx_peak),'go','MarkerSize',8,'LineWidth',2);
plot(idx_onset, env(idx_onset),'ro','MarkerSize',8,'LineWidth',2);
text(idx_peak, env(idx_peak), sprintf('x=%.2d', idx_peak), 'VerticalAlignment','bottom','HorizontalAlignment','right','Color','k','FontSize',10);
text(idx_onset, env(idx_onset), sprintf('x=%.2d', idx_onset), 'VerticalAlignment','top','HorizontalAlignment','left','Color','r','FontSize',10);
legend('envelope','peak','onset');
subtitle(file_name);

% ---------- Calculate Distance -------
distance_by_peak = distance(idx_peak, Fs, 3, 1);
distance_by_onset = distance(idx_onset, Fs, 3, 1);
fprintf('distance_by_peak %.2f\n', distance_by_peak);
fprintf('distance_by_onset %.2f\n', distance_by_onset);