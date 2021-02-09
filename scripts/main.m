clc;clear all;
global data;
global dataRecieved;
data = "";
dataRecieved = false;
global running; 
running = true;
% addpath('../../app'); %to be able to run next line to open app
appHandle = arduinoApp;
serialPort = serialport("COM3", 115200);
configureTerminator(serialPort,"LF"); %sets newline as line ending
flush(serialPort); %so that data doesnt get clogged/backed up
configureCallback(serialPort,"terminator",@readSerialData)

% hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
changed = false;
changedTimer = tic;
count = 0;
while(running)
    try
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
            dataArray = strsplit(data, ',');
            WOBValToApp = str2double(dataArray(3));
            WOBValToApp = round(WOBValToApp,2);
            appHandle.WOBNEditField.Value = WOBValToApp;
            dataRecieved = false;
        end
        pause(0.01);
    catch
        disp("ERROR!");
        break;
    end
end
disp('Stopped... final info:');
disp('data: ' + data);
disp('linetowrite: ' + linetowrite);
configureCallback(serialPort,"off");
% clear all;



function returnVal = getValuesFromApp(appHand)
    %if app is closed accidently warning appears
    cmdMode = appHand.Drilling_Mode; %1 for manual ROP control, 0 for (automatic) pid control
    dir = appHand.ROP_Direction_Cmd; %drill dir
    speed = appHand.ROP_Speed_Cmd; %drill speed
    miragePosition = appHand.Mirage_Position_Cmd; %mirage position
    returnVal = cmdMode +  "," + dir + "," + speed + "," + miragePosition;
end

function readSerialData(src,~)
    global data;
    global dataRecieved;
    data = readline(src);
    dataRecieved = true;
end

function clearVars()

end

 
 
