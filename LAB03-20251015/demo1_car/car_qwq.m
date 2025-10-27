% clear; clc; close all;
subfolder = 'car_90cm';
filename = 'car1.mat';
fullFilePath = fullfile(subfolder, filename);

data = load(fullFilePath);
data = data.data
data = scale_and_calib(data);
plot_imu(data); 
imu_distance_estimation(data, 50);

