clc;clear;
pause on
x= 0;
y = 5;
fileID = fopen("abcc.txt", 'a'); %appends to end of file, or creates file and writes to it
format shortg; %smart formating
a = 0;
while (a < 15)
    x = x + 1;
    y = y+1
    c = clock

    fprintf(fileID,'%i-%i-%i-%i-%i-%2.3f %.2f\r\n',c, y);
    pause(0.5)
    a = a + 1;
end
fclose(fileID);
