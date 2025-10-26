function data_calib = scale_and_calib(data)
% SCALE_AND_CALIB 將六軸感測資料進行單位轉換與校正
%
% 輸入：
%   data (n×6) - 每列為 [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
%
% 輸出：
%   data_calib (n×6) - 校正後的 [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
%
% 範例：
%   load('data/y_up.mat'); 
%   result_calib = scale_and_calib(result);

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

    %% 校正參數（依表格）
    S_accx = 0.9996;  b_accx = 0.0820;
    S_accy = 0.9980;  b_accy = 0.0883;
    S_accz = 0.9924;  b_accz = 0.1106;

    S_gyrox = 1.0;    b_gyrox = 0.0528;
    S_gyroy = 1.0;    b_gyroy = 0.0333;
    S_gyroz = 1.0;    b_gyroz = 0.0145;

    %% 依公式 x_calib = x*S + b
    acc_x_calib  = acc_x  * S_accx  + b_accx;
    acc_y_calib  = acc_y  * S_accy  + b_accy;
    acc_z_calib  = acc_z  * S_accz  + b_accz;

    gyro_x_calib = gyro_x * S_gyrox + b_gyrox;
    gyro_y_calib = gyro_y * S_gyroy + b_gyroy;
    gyro_z_calib = gyro_z * S_gyroz + b_gyroz;

    %% 組合校正後的資料
    data_calib = [acc_x_calib, acc_y_calib, acc_z_calib, ...
                  gyro_x_calib, gyro_y_calib, gyro_z_calib];

end