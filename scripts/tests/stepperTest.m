clear;clc;
a = arduino('COM3', 'Mega2560');
hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');

pulsePin = "D2";
directionPin = "D3";
dt = 0.1;
loopCount = 0;

writeDigitalPin(a,directionPin,1);
% configurePin(a, pulsePin,
pause on;
time1 = tic;
while (1)
    writeDigitalPin(a,pulsePin,0);
%     disp('LOW');
    loopCount = loopCount + 1;
    if (toc(time1) >= 1)
%         disp("fps: " + loopCount);
        loopCount = 0;
        time1 = tic;
    end
%     if (~ishandle(hWaitbar)) % Stop if cancel button was pressed
%         disp('Stopped by user');
%         break;
%     end
%   5 steps per second?
    writeDigitalPin(a,pulsePin,1);
%     disp('HIGH');
%     pause(dt);
    
    
%     pause(dt);
end

