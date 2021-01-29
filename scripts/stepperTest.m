clear;clc;
a = arduino('COM3', 'Mega2560');
hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');

pulsePin = "D2";
directionPin = "D3";
dt = 0.1;

writeDigitalPin(a,directionPin,1);
% configurePin(a, pulsePin,
pause on;
while (1)
    if (~ishandle(hWaitbar)) % Stop if cancel button was pressed
        disp('Stopped by user');
        break;
    end
%   5 steps per second?
    writeDigitalPin(a,pulsePin,1);
    disp('HIGH');
    pause(dt);
    writeDigitalPin(a,pulsePin,0);
    disp('LOW');
    pause(dt);
end

