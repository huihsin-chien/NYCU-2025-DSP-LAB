clear;       % 清除 workspace 中的變數
clc;         % 清除 Command Window
close all;   % 關閉所有 figure 視窗
file = fullfile("walk","straight2.mat");
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

subwalk_1 = data(790:1709, :);
subwalk_2 = data(1910:2895, :);
figure();
subplot(2,1,1);
plot(subwalk_1(:, 3)); % acc_z for subwalk_1
grid on;
xlabel("sample point"); ylabel("acc_z (subwalk 1)");
subplot(2,1,2);
plot(subwalk_2(:, 3)); % acc_z for subwalk_2
grid on;
xlabel("sample point"); ylabel("acc_z (subwalk 2)");
