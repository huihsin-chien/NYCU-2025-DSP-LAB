function plot_imu(data, fs)
% PLOT_ACC_GYRO  畫出六軸訊號的時域與頻域圖
%
%   plot_acc_gyro(data, fs)
%
% 輸入:
%   data - n×6 array, 每列為 [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
%   fs   - 取樣頻率 (Hz)，預設 50
%
% 輸出:
%   畫出兩張圖：一張加速度，一張陀螺儀，皆包含時域與頻域子圖

    if nargin < 2
        fs = 50;  % 預設取樣率
    end

    n = size(data, 1);
    t = (0:n-1)' / fs;

    acc = data(:, 1:3);
    gyro = data(:, 4:6);

    acc_names  = {'a_x (m/s^2)','a_y (m/s^2)','a_z (m/s^2)'};
    gyro_names = {'g_x (deg/s)','g_y (deg/s)','g_z (deg/s)'};

    % ---------- 時域 ----------
    figure('Name','Accelerometer - Time Domain','Color','w');
    for k = 1:3
        subplot(3,1,k);
        plot(t, acc(:,k), 'LineWidth',1);
        grid on; xlabel('Time (s)'); ylabel(acc_names{k});
    end

    figure('Name','Gyroscope - Time Domain','Color','w');
    for k = 1:3
        subplot(3,1,k);
        plot(t, gyro(:,k), 'LineWidth',1);
        grid on; xlabel('Time (s)'); ylabel(gyro_names{k});
    end

    % ---------- 頻域 ----------
    L = n;
    NFFT = 2^nextpow2(L);
    f = fs*(0:(NFFT/2))/NFFT;

    figure('Name','Accelerometer - Frequency Domain','Color','w');
    for k = 1:3
        X = fft(acc(:,k), NFFT)/L;
        P1 = 2*abs(X(1:NFFT/2+1));
        subplot(3,1,k);
        plot(f, P1, 'LineWidth',1);
        xlim([0 fs/2]); grid on;
        xlabel('Frequency (Hz)'); ylabel(['|',acc_names{k},'|']);
    end

    figure('Name','Gyroscope - Frequency Domain','Color','w');
    for k = 1:3
        X = fft(gyro(:,k), NFFT)/L;
        P1 = 2*abs(X(1:NFFT/2+1));
        subplot(3,1,k);
        plot(f, P1, 'LineWidth',1);
        xlim([0 fs/2]); grid on;
        xlabel('Frequency (Hz)'); ylabel(['|',gyro_names{k},'|']);
    end

end
