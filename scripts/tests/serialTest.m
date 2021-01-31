clc;clear;
% a = arduino('COM3', 'Mega2560');
serialPort = serialport("COM3", 9600);
% configureTerminator(serialPort,"LF");
hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');

while(1)
    if (~ishandle(hWaitbar)) % Stop if cancel button was pressed
        disp('Stopped by user');
        break;
    end
%     data = readline(serialPort); %for reading asci
    data = read(serialPort,1,"int32")
%     disp(data);
    pause(0.01);
end
clear serialPort;

