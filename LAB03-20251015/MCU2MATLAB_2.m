portName   = "COM4";
baudRate   = 115200;
timeoutSec = 2;

s = serialport(portName, baudRate, "Timeout", timeoutSec);
flush(s);

write(s, uint16(hex2dec('5A')), "uint16");
fprintf("→ Sent command: 0x%X\n", hex2dec('5A'));

WORDS_PER_FRAME = 10;                 
BYTES_PER_FRAME = WORDS_PER_FRAME*2;  
PREFIX = uint16(hex2dec('0FFF'));
POSTFIX = uint16(hex2dec('0FFF'));

buf = uint8([]);
result = [];

fprintf("→ 開始接收資料... 按 Ctrl+C 可中止接收\n");

try
    while true
        if s.NumBytesAvailable > 0
            newb = read(s, s.NumBytesAvailable, "uint8");
            buf = [buf, newb];
        else
            pause(0.001);
        end


        while numel(buf) >= BYTES_PER_FRAME
            idx = strfind(buf, uint8([255 15]));  
            if isempty(idx)
                if numel(buf) > 1
                    buf = buf(end-1:end);
                end
                break;
            end

            startPos = idx(1);  

            if (numel(buf) - startPos + 1) < BYTES_PER_FRAME

                buf = buf(startPos:end);
                break;
            end

            frame_bytes = buf(startPos : startPos + BYTES_PER_FRAME - 1);

            buf = buf(startPos + BYTES_PER_FRAME : end);
            words_u16 = typecast(uint8(frame_bytes), 'uint16'); 

            words_i16 = typecast(words_u16, 'int16');
            if ~(words_u16(1) == PREFIX && words_u16(10) == POSTFIX)
                continue;
            end
            counter = words_u16(2);
            acc_x   = words_i16(3);
            acc_y   = words_i16(4);
            acc_z   = words_i16(5);
            gyro_x  = words_i16(6);
            gyro_y  = words_i16(7);
            gyro_z  = words_i16(8);
            % words_i16(9) 保留不用

            result = [result; double([acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])]; %#ok<AGROW>

            fprintf("← #%5d | ACC=[%6d %6d %6d]  GYRO=[%6d %6d %6d] (cnt=%d)\n", ...
                 size(result,1), acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, counter);
        end
    end
catch
    fprintf("→ 中止接收，總筆數: %d。\n", size(result,1));
end

clear s;
