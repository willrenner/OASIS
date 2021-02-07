clc;clear;clear global;
global data;
global dataRecieved;
data = "";
dataRecieved = false;
global running; 
running = true;
% addpath('../../app'); %to be able to run next line to open app
appHandle = arduinoApp;
% serialPort = serialport("COM4", 115200);
% configureTerminator(serialPort,"LF"); %sets newline as line ending
% flush(serialPort); %so that data doesnt get clogged/backed up
% configureCallback(serialPort,"terminator",@readSerialData)

% hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
changed = false;
changedTimer = tic;
count = 0;
while(running)
    if (toc(changedTimer) > 0.25) %sets a send rate
        changed = true;
        changedTimer = tic;
    end
    if (changed)
        linetowrite = getValuesFromApp(appHandle);
%         writeline(serialPort, linetowrite);
        changed = false;
    end
    if (dataRecieved)
        %write this data to file and be done with it
        disp(data);
        dataRecieved = false;
    end
    pause(0.01);
end
clearVars();


function returnVal = getValuesFromApp(appHand)
    global running;
    try %if app is closed accidently warning appears
        cmdMode = appHand.Drilling_Mode; %1 for manual ROP control, 0 for (automatic) pid control
        dir = appHand.ROP_Direction_Cmd; %drill dir
        speed = appHand.ROP_Speed_Cmd; %drill speed
        miragePosition = appHand.Mirage_Position_Cmd; %mirage position
        returnVal = cmdMode +  "," + dir + "," + speed + "," + miragePosition;
    catch
        warning("App handle lost!");
        returnVal = "";
        running = false; %add send a reset command to arduino
    end
    
end

function readSerialData(src,~)
    global data;
    global dataRecieved;
    data = readline(src);
    dataRecieved = true;
end

function clearVars()
    disp('Stopped by user');
%     configureCallback(serialPort,"off");
    clear serialPort;
end

 
 
