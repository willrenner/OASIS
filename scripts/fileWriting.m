clc;clear;
pause on
x = 0;
y = 5;
fileID = fopen("../logs/abc.txt", 'a'); %appends to end of file, or creates file and writes to it
format shortg; %smart formating
a = 0;
tic;
while (a < 1500000)
%     x = x + 1;
    y = y -.5 + 1 * rand();
    c = clock;
    fprintf(fileID,'%i-%i-%i-%i-%i-%2.3f %.2f\r\n',c, y);
    if (toc >= 1)
        disp("fps: " + x);
        x = 0;
        tic;
    end
    a = a + 1;
    pause(0.05)
end
fclose(fileID);
%fclose('all')