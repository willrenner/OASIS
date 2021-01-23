clc;clear;
pause on
x= 0;
y = 5;
fileID = fopen("../logs/abc.txt", 'A'); %appends to end of file, or creates file and writes to it
format shortg; %smart formating
a = 0;
tic;
while (a < 1500000)
    x = x + 1;
    y = y + 1;
    c = clock;
    fprintf(fileID,'%i-%i-%i-%i-%i-%2.3f %.2f\r\n',c, y);
    if (toc >= 1)
        disp("fps: " + x);
        x = 0;
        tic;
    end
    a = a + 1;
end
fclose(fileID);
%fclose('all')