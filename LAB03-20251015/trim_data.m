clear;       % 清除 workspace 中的變數
clc;         % 清除 Command Window
close all;   % 關閉所有 figure 視窗
file = fullfile("walk","square3.mat");
data = load(file).data;

figure();
subplot(2,1,1);
plot(data(:, 3)); %　acc_z
grid on;
xlabel("sample point"); ylabel("acc_z");
subplot(2,1,2);
plot(data(:, 6)); % omega_z
grid on;
xlabel("sample point"); ylabel("omega_z");


%{

7350:8584
8872:10027



subwalk_2 = data(7350:8584, :);     
subwalk_3 = data(8872:10027, :);
figure();
subplot(2,1,1);
plot(subwalk_2(:, 3)); % acc_z for subwalk_1
grid on;
xlabel("sample point"); ylabel("acc_z (subwalk 1)");
subplot(2,1,2);
plot(subwalk_3(:, 3)); % acc_z for subwalk_2
grid on;
xlabel("sample point"); ylabel("acc_z (subwalk 2)");
%}


% data = data(5773:6966, :);
data = data(8872:10027, :);    
figure();
plot(data(:, 3)); % acc_z for subwalk_1
grid on;
xlabel("sample point"); ylabel("acc_z (subwalk 1)");