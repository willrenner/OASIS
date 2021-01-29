clear;
pause on
x = 0;
y = 5;
a = 0;
tic;

time1 = hat;
while (a < 15000000)
    x = x + 1;
    y = y + 1;
    if (toc >= 0.00001)
        disp("fps: " + x);
        x = 0;
        tic;
    end
%     diff = hat - time1;
%     if (diff >= 0.0001)
%         disp("fps: " + x);
%         x = 0;
%         time1 = hat;
%     end
    a = a + 1;
end
