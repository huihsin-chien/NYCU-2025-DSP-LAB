% ecg = load("MATLAB/hand_move_up_down.mat");
% ecg = load("MATLAB/hand_grasp.mat");
% ecg = load("MATLAB/foot_move.mat");
% ecg = load("MATLAB/hsecg_1.mat");
% ecg = load("MATLAB/with_rc_2.mat");
% ecg = load("MATLAB/without_rc.mat");
ecg = load("MATLAB/demo_test.mat");
ecg = double(ecg.received_data);                 % ensure double
ecg = ecg - mean(ecg);                           % de-mean
Fs  = 1000;
ecg = ecg(:);
N   = length(ecg);
t   = (0:N-1)/Fs;
f   = (-N/2:N/2-1)*(Fs/N);

%% FIR Low-pass Filter Design (cutoff = 40 Hz)
cutoff = 40;  order = 200;
b = fir1(order, cutoff/(Fs/2), 'low');
a = 1;
ecg_lp = filtfilt(b,a,ecg);                      % keep a copy (LP result)

%% Optional: high-pass to remove baseline wander (<0.5 Hz)
[b_hp,a_hp] = butter(2, 0.5/(Fs/2), 'high');
ecg_bp = filtfilt(b_hp, a_hp, ecg_lp);           % LP + HP (band-limited, baseline removed)

%% --- Wavelet Transform on ECG ---
% A) CWT (limit y-axis to useful band after LP)
[cfs,frq] = cwt(ecg_bp, 'amor', Fs);
figure(2);
imagesc(t, frq, abs(cfs)); axis xy; grid on
ylim([0 80]);                                    % after 40 Hz LP, most energy <= ~40
xlabel('Time (s)'); ylabel('Frequency (Hz)');
title('FIGURE 2: ECG CWT Scalogram (|coefficients|)'); colorbar

% B) Wavelet denoising with stronger settings
wname = 'sym4';
levMax = wmaxlev(N, wname);
lev = min(6, levMax);                            % bound to data length

ecg_wden = wdenoise(ecg_bp, lev, ...
    'Wavelet',         wname, ...
    'DenoisingMethod', 'SURE', ...               % stronger & adaptive than Universal
    'ThresholdRule',   'Soft', ...
    'NoiseEstimate',   'LevelDependent');        % per-level sigma

%% FIGURE 3: Time Domain (compare LP vs LP+HP+Wavelet)
figure(3);
%plot(t, ecg_lp, 'LineWidth', 1.2, 'Color', [0.6, 0.6, 0.9], 'DisplayName','LP only'); hold on
plot(t, ecg_wden, 'r-', 'LineWidth', 1.2, 'DisplayName','LP + Wavelet');
grid on; xlabel('Time (s)'); ylabel('Amplitude');
title('FIGURE 3: ECG (LP+Wavelet)'); legend; hold off

%% FIGURE 4: Frequency Domain (compare)
ECG_lp_fft  = abs(fftshift(fft(ecg_lp)))/N;
ECG_wd_fft  = abs(fftshift(fft(ecg_wden)))/N;
figure(4);
plot(f, ECG_lp_fft,  'DisplayName','LP only'); hold on
plot(f, ECG_wd_fft,  'DisplayName','LP + Wavelet');
xlim([0 80]); grid on
xlabel('Frequency (Hz)'); ylabel('Magnitude');
title('FIGURE 4: Spectrum'); legend; hold off

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
