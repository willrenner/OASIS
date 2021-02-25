clc;clear all;
format long;
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
fileID = fopen("../logs/abcdef.txt", 'a'); %appends to end of file, or creates file and writes to it
% hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
changed = false;
changedTimer = tic;
count = 0;
while(running)
%     try
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
            %indecies =====> [WOB, drillRPM, drillCurrent, drillPos, limitSwitchReached] ... update as needed
            dataArray = strsplit(data, ',');
            sizeOfArr = size(dataArray);
            if (sizeOfArr(2) > 1) %new
                disp(data);
                WOB = round(str2double(dataArray(1)), 2);
                drillRPM = str2double(dataArray(2)); 
                drillCurrent = str2double(dataArray(3)); 
                drillPos = str2double(dataArray(4)); 
                limitSwitchReached = str2double(dataArray(5));
                %-----send to app-----
                appHandle.WOBNEditField.Value = WOB;
                if (limitSwitchReached == 1) 
                    appHandle.LimitSwitchReachedLamp.Color = [0,1,0]; %rgb
                else
                    appHandle.LimitSwitchReachedLamp.Color = [1,1,1];
                end
                %--------------------

                %-----log to file----
                %log all values
%                 t = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss.SSS Z')
%                 p = posixtime(t)
%                 fprintf(fileID,'%i-%i-%i-%i-%i-%2.3f %.2f\r\n',c, y); % Write to file
%                 fprintf(fileID,'%.2f %.2f\r\n',c, y); % Write to file
                
                %--------------------
            else
                disp("Message from arduino: ");
                disp(data);
            end
            dataRecieved = false;
        end
        pause(0.01);
%     catch
%         disp("ERROR!");
%         break;
%     end
end
disp('Stopped... final info:');
disp('data: ' + data);
% disp('linetowrite: ' + linetowrite);
configureCallback(serialPort,"off");
clear all;



function returnVal = getValuesFromApp(appHand) %mode, dir, speed, miragePos
    %if app is closed accidently warning appears
    cmdMode = appHand.Drilling_Mode; %1 for manual ROP control, 0 for (automatic) pid control
    dir = appHand.ROP_Direction_Cmd; %drill dir
    speed = appHand.ROP_Speed_Cmd; %drill speed
    miragePosition = appHand.Mirage_Position_Cmd; %mirage position
    DrillRPM = appHand.RPMSpeed_Cmd;
    returnVal = cmdMode +  "," + dir + "," + speed + "," + miragePosition;
end

function readSerialData(src,~)
    global data;
    global dataRecieved;
    data = readline(src);
    dataRecieved = true;
end


 
 
