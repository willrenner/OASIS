% bool getCurrentSensorValue(void*) { //analog read is slow
%     float val = -0.04757 * analogRead(currentSensorPin) + 24.36; //eqn to get amperage, y = mx + b by testing
%     val = sqrt(pow(val, 2) + pow(drillCurrent, 2)); //kinda RMS not really, may not even work who knows
%     drillCurrent = 0.1*val + 0.9*drillCurrent; //complementary filter
%     return true;
% }

% timer.every(1/currentSensorRate * 1000, getCurrentSensorValue);

% rate = 120
clc;clear;
%%Time specifications:
Fs = 10000;                   % samples per second
dt = 1/Fs;                   % seconds per sample
StopTime = 1;                % seconds
t = (0:dt:StopTime-dt)';     % seconds
%%Sine wave:
Fc = 5;                     % hertz
x = 169.7*cos(2*pi*Fc*t);
count = 1;
cutoff = 60;
for (i = 1:size(x))
    if ((x(count) > cutoff) | (x(count) < -cutoff))
%         disp(x(count))
        x(count) = 0;
    end
    count = count + 1;
end
sampleTime = 155;
dtsample = 1/sampleTime;
counter = 0;
size = 0;
% tsample = (0:dtsample:StopTime-dtsample)';
for (i = 1:Fs/sampleTime:length(t))
    val = x(round(i));
    counter = counter + val^2;
%     disp(round(i) + "--" + val)
    size = size + 1;
end
disp(sqrt(counter / size));
% Plot the signal versus time:
figure;
plot(t,x);
xlabel('time (in seconds)');
title('Signal versus Time');
