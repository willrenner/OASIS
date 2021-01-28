clear;clc;
a = arduino('COM3', 'Mega2560');
hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');

pulsePin = "D1";
directionPin = "D2";
highPin = "D3";
writeDigitalPin(a,highPin,1);
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
    pause(0.1);
    writeDigitalPin(a,pulsePin,0);
    pause(0.1);
end

