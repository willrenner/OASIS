clc;clear;
pause on
x= 0;
y = 5;
fileID = fopen("../logs/abc.txt", 'a'); %appends to end of file, or creates file and writes to it
format shortg; %smart formating
a = 0;
start = tic;
while (a < 15000)
    x = x + 1;
    y = y + 1;
%     c = clock
    currTime = toc(start);
%     fprintf(fileID,'%i-%i-%i-%i-%i-%2.3f %.2f\r\n',c, y);
    fprintf(fileID, '%2.2f, %i\r\n', currTime, y);
    pause(0.5)
    a = a + 1;
end
fclose(fileID);
%fclose('all')