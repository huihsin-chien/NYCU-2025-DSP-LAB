clear; clc; close all;
subfolder = './car_90cm';
filename = 'car2.mat';
fullFilePath = fullfile(subfolder, filename);

data = load(fullFilePath);
data = data.data
calibrated_data = scale_and_calib(data);
calibrated_data = calibrated_data - mean(calibrated_data);


%% 參數設定
    ACC_SCALE  = 16384;   % LSB/g
    GYRO_SCALE = 131;     % LSB/(deg/s)
    g = 9.8;

    % ---- Scale ----
    acc_x  = ( data(:,1) / ACC_SCALE ) * g;
    acc_y  = ( data(:,2) / ACC_SCALE ) * g;
    acc_z  = ( data(:,3) / ACC_SCALE ) * g;
    gyro_x = data(:,4) / GYRO_SCALE;
    gyro_y = data(:,5) / GYRO_SCALE;
    gyro_z = data(:,6) / GYRO_SCALE;
data=[acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z];

plot_imu(calibrated_data);

figure();
plot(data(:, 4), 'r-','DisplayName','raw');
hold on; % Retain current plot
plot(calibrated_data(:, 4), 'b-','DisplayName','calibrated');
title('Gryo_x');
legend show; % Display legend