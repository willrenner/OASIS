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
fileID = fopen("../logs/secondLog.txt", 'a'); %appends to end of file, or creates file and writes to it
% hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
changed = false;
changedTimer = tic;
count = 0;

while(running)
    try
        if (toc(changedTimer) > 0.1) %sets a send rate
            changed = true;
            changedTimer = tic;
        end
        if (changed)
            linetowrite = getValuesFromApp(appHandle);
            writeline(serialPort, linetowrite);
            changed = false;
        end
        if (dataRecieved)
            %indecies =====> [WOB, drillRPM, drillCurrent, drillPos, mirageAngle, drillLimitSwitchActive] ... update as needed
            dataArray = strsplit(data, ',');
            sizeOfArr = size(dataArray);
            if (sizeOfArr(2) > 1) %if array contains a comma, meaning not a Serial debug statement
                fprintf("WOB: %3.2f, RPM: %7.2f, Current: %2.2f, Z-Pos: %6.2f, Mir-Ang: %7.2f, Lim-Sw: %2.0f --- ",dataArray(1),dataArray(2),dataArray(3),dataArray(4),dataArray(5), dataArray(6));
%               [drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare]
                fprintf("CmdMode: %2.0f, DirectionCmd: %2.0f, SpeedCmd: %7.2f, MirageAngleCmd: %7.2f, RPM_Cmd: %7.2f, HeaterCmd: %2.0f, PumpCmd: %2.0f, TareCmd: %2.0f\n",dataArray(7),dataArray(8),dataArray(9),dataArray(10),dataArray(11), dataArray(12), dataArray(13), dataArray(14));
                setAppData(appHandle, dataArray); %change data in app
                writeDataToFile(dataArray, fileID) %log to file
            else %meaning Serial debug statment, and not data array
                disp("Message from arduino: ");
                disp(data);
            end
            dataRecieved = false;
        end
        pause(0.01);
    catch e
        disp("ERROR!");
        fprintf(1,'The identifier was: %s\n',e.identifier);
        fprintf(1,'The message was: %s\n',e.message);
        break;
    end
end
configureCallback(serialPort,"off");
fclose(fileID);
clear all;



function writeDataToFile(da, fid)
    t = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss.SSS Z');
    p = posixtime(t);
%   [WOB, drillRPM, drillCurrent, drillPos, mirageAngle, drillLimitSwitchActive]
    fprintf(fid,'%.3f %.2f %.2f %.2f %.2f %.2f %.2f\r\n', p,da(1),da(2),da(3),da(4),da(5),da(6)); % Write to file  
end

function returnVal = getValuesFromApp(appHand) %[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare, zeroCmd, fakeZeroAcitve]
    drillCmdMode = appHand.Drilling_Mode; %1 for manual ROP control, 0 for (automatic) pid control
    dir = appHand.ROP_Direction_Cmd; %drill dir
    speed = appHand.ROP_Speed_Cmd; %drill speed
    miragePosition = appHand.Mirage_Position_Cmd * appHand.Mirage_Direction_Cmd; %mirage position
    rpm = appHand.RPMSpeed_Cmd;
    heater = appHand.Heater_Cmd;
    pump = appHand.Pump_Cmd;
    tare = appHand.Tare_Cmd;
    zeroCmd = appHand.DrillZero_Cmd;
    fakeZero = appHand.FakeZero_Cmd;
    if (tare == 1)
        appHand.Tare_Cmd = 0; %reset tare to 0 in app
    end
    returnVal = drillCmdMode + "," + dir + "," + speed + "," + miragePosition + "," + rpm + "," + heater + "," + pump + "," + tare + "," + zeroCmd + "," + fakeZero;
end

function readSerialData(src,~)
    global data;
    global dataRecieved;
    data = readline(src);
    dataRecieved = true;
end

function setAppData(appHandle, dataArray)
    WOB = round(str2double(dataArray(1)), 2);
    drillRPM = round(str2double(dataArray(2)), 2);
    drillCurrent = round(str2double(dataArray(3)), 2); 
    drillPos = round(str2double(dataArray(4)), 2);
    mirageAngle = round(str2double(dataArray(5)), 2);
    limitSwitchReached = str2double(dataArray(6));

    %-----send to app-----
    appHandle.WOBNEditField.Value = WOB;
    appHandle.DrillRPMEditField.Value = drillRPM;
    appHandle.DrillPosmmEditField.Value = drillPos;
    appHandle.MiragePosdegEditField.Value = mirageAngle;
    appHandle.DrillCurrentAEditField.Value = drillCurrent;
    if (limitSwitchReached == 1) 
        appHandle.LimitSwitchReachedLamp.Color = [0,1,0]; %rgb
    else
        appHandle.LimitSwitchReachedLamp.Color = [1,1,1];
    end
end


 
 
