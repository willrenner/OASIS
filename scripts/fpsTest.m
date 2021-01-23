clc;clear;
pause on
x = 0;
y = 5;
a = 0;
tic;
while (a < 15000000)
    x = x + 1;
    y = y + 1;
    if (toc >= 1)
        disp("fps: " + x);
        x = 0;
        tic;
    end
    a = a + 1;
end
