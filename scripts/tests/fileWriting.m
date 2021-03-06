clc;clear;
hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
fileID = fopen("../../logs/posix3.txt", 'a'); %appends to end of file, or creates file and writes to it
format shortg; %smart formating
pause on

x = 0;
y = 5;
a = 0;
tic;
while (a < 1500000)
    x = x + 1;
    y = y -.5 + 1 * rand();
    c = clock;
    t = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss.SSS Z');
    p = posixtime(t)
    fprintf(fileID,'%15f %4f\r\n',p, y); % Write to file
%     fprintf(fileID,'%.2f %.2f\r\n',c, y); % Write to file  
%     fprintf(fileID,'%i-%i-%i-%i-%i-%2.3f %.2f\r\n',c, y); % Write to file
    if (toc >= 1)
        disp("fps: " + x);
        x = 0;
        tic;
    end
    a = a + 1;
    pause(0.01) % Sets fps essentially
    if (~ishandle(hWaitbar)) % Stop if cancel button was pressed
        disp('Stopped by user');
        break;
    end
end
fclose(fileID);