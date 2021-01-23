clc;
clear;

%read and send serial
numberOfVars = 1;
% serialPort = serialport("COM3", 9600);
% configureTerminator(serialPort, "CR/LF"); %google this (means \r\n)
% a = arduino('COM3', 'Mega2560');
% s = servo(a, 'D12');
% configureCallback(serialPort, "terminator", @tester); %calls callbackFcn when the serial port contains a terminator; the arudino has sent all its data over serial
% configureCallback(serialPort,"off");

running = true;
rop_speed_cmd = '0';
rop_direction_cmd = '0';
drilling_mode = '1';
data = [];
tCurrent = [];

t = tiledlayout(2,1);
ax1 = nexttile;
ax2 = nexttile;
hold([ax1 ax2],'on');
grid;

tStart = tic; %start timer
while (running)
    disp("Waiting on data...");
    if (drilling_mode == '0') %manual mode
            
    elseif (drilling_mode == '1') %automatic mode
%         data = read(serialPort, 5, "string")
        data(end + 1) = str2double(readline(serialPort)); %reads 1 32bit float
    end
    
    tCurrent(end + 1) = toc(tStart);
    plot(ax1, tCurrent(end), data(end), '.k');
    plot(ax2, tCurrent(end), data(end), '.k');
    
  
end


