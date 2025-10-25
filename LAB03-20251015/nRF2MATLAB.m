filename = './strait/ble_processed_square_3.txt';

% 讀取檔案所有行
lines = readlines(filename);
lines = strtrim(lines);        
lines(lines=="") = [];        
num_lines = numel(lines);

data = zeros(num_lines,6);  % 每行存 [acc_x acc_y acc_z gyro_x gyro_y gyro_z]

for k = 1:num_lines
    % 改成以空白分割，而不是 '-'
    hex_vals = strsplit(strtrim(lines(k)));  
    % 移除可能存在的 "0x" 前綴（保險用）
    hex_vals = erase(hex_vals, "0x");  

    for i = 1:6
        low_byte  = hex2dec(hex_vals{2*i-1});
        high_byte = hex2dec(hex_vals{2*i});
        val = bitor(low_byte, bitshift(high_byte, 8));  % 組成 16-bit
        
        % 二補數轉換
        if val >= 2^15
            val = val - 2^16;
        end
        data(k,i) = val;
    end
end

disp(data);
