clc;clear;clear global;
global data;
global dataRecieved;
data = "";
dataRecieved = false;

rop_direction_cmd = 0;
drilling_mode = 0;
rop_speed_cmd = 0;


addpath('../../app');
appHandle = arduinoApp;

serialPort = serialport("COM4", 115200);
configureTerminator(serialPort,"LF"); %sets newline as line ending
hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
changed = false;
changedTimer = tic;
% count = 0;
flush(serialPort); %so that data doesnt get clogged/backed up
configureCallback(serialPort,"terminator",@readSerialData)
while(1)
    if (~ishandle(hWaitbar)) % Stop if cancel button was pressed
        break;
    end
    if (toc(changedTimer) > 0.25) %sets a send rate
        changed = true;
        changedTimer = tic;
    end
    if (changed)
        linetowrite = getValuesFromApp(appHandle);
        writeline(serialPort, linetowrite);
        changed = false;
    end
    if (dataRecieved)
        %write this data to file and be done with it
        disp(data);
        dataRecieved = false;
    end
    pause(0.01);
end

disp('Stopped by user');
configureCallback(serialPort,"off");
delete(appHandle);
clear serialPort;

function returnVal = getValuesFromApp(appHand)
    try %if app is closed accidently warning appears
        dir = appHand.ROP_Direction_Cmd;
        speed = appHand.ROP_Speed_Cmd;
        returnVal = dir + "," + speed;
    catch
        warning("App handle lost!");
    end
    
end

function readSerialData(src,~)
    global data;
    global dataRecieved;
    data = readline(src);
    dataRecieved = true;
end


 
 
