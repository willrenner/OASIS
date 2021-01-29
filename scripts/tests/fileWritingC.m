clc;clear;
hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
fileID = fopen("../logs/hatTest.txt", 'a'); %appends to end of file, or creates file and writes to it
format long; %smart formating
pause on

x = 0;
y = 5;
a = 0;
time = hat;
time2 = hat;
while (a < 1500000)
    x = x + 1;
    y = y -.5 + 1 * rand();
    diff = hat - time;
    diff2 = hat - time2;
    fprintf(fileID,'%f %.2f\r\n',diff, y); % Write to file
    if (diff2 >= 1)
        disp("fps: " + x);
        x = 0;
        time2 = hat;
    end
    a = a + 1;
%     pause(0.05) % Sets fps essentially
    if (~ishandle(hWaitbar)) % Stop if cancel button was pressed
        disp('Stopped by user');
        break;
    end
end
fclose(fileID);