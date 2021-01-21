clc;
clear;

%read and send serial
%numberOfVars = 5;
%serialPort = serialport("COM1", 9600)
running = true;

rop_speed_cmd = '0';
rop_direction_cmd = '0';
drilling_mode = '0';

%configureCallback(serialPort,"terminator", callbackFcn); %calls callbackFcn when the serial port contains a terminator; the arudino has sent all its data over serial
%figure("Name", "Recieved Data");
%tic; %start timer

a = arduino('COM3', 'Mega2560');
s = servo(a, 'D12');


pause on
while (running)
    disp("Dir: " + rop_direction_cmd + ", Speed: " + rop_speed_cmd + ", Mode: " + drilling_mode);
    
    if (drilling_mode == '0') %manual mode
        
        
    elseif (drilling_mode == '1') %automatic mode
        writePosition(s, 0);
        pause(500/1000);
        writePosition(s, 1);
        pause(500/1000);
    end
    
    
    
    pause(100/1000);
end
clear a;

% function callbackFcn()
%     %get data from serial
%     data = read(serialPort, numberOfVars, "single"); % reads from serial port numberOfVars (5) 32bit values; data is a row vector of 32bit values
%     current = data(0);
%     wob = data(1);
%     rop = data(2);
%     temp = data(3);
%     elapsedTime = toc;
%     
%     %plot data
%     plot(elapsedTime, wob);
%     xlim('auto');
%     ylim('auto');
%     drawnow;
% end