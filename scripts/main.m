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
fileID = fopen("../logs/run1.txt", 'a'); %appends to end of file, or creates file and writes to it
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
            %indecies =====> LoadCellLeftValue, LoadCellRightValue, DrillCurrent, HeaterPower, HeaterTemp ... update as needed
            dataArray = strsplit(data, ',');
            sizeOfArr = size(dataArray);
            if (sizeOfArr(2) > 1) %if array contains a comma, meaning not a Serial debug statement
                fprintf("LoadCellLeftValue: %4.2f, LoadCellRightValue: %4.2f, DrillCurrent: %4.2f, HeaterPower: %4.2f, HeaterTemp: %4.2f, DrillPos: %4.2f, Extr. Pos: %4.2f\n ",dataArray(1),dataArray(2),dataArray(3),dataArray(4),dataArray(5),dataArray(6),dataArray(7));
                incomingData = sprintf("LoadCellLeftValue: %4.2f, LoadCellRightValue: %4.2f, DrillCurrent: %4.2f, HeaterPower: %4.2f, HeaterTemp: %4.2f, DrillPos: %4.2f, Extr.Pos: %4.2f\n ",dataArray(1),dataArray(2),dataArray(3),dataArray(4),dataArray(5),dataArray(6),dataArray(7));
                appHandlele.IncomingDataLabel.Text = incomingData; 
                %[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare]
                %fprintf("CmdMode: %2.0f, DirectionCmd: %2.0f, SpeedCmd: %7.2f, MirageAngleCmd: %7.2f, RPM_Cmd: %7.2f, HeaterCmd: %2.0f, PumpCmd: %2.0f, TareCmd: %2.0f, DrillCmd: %2.0f\n",dataArray(7),dataArray(8),dataArray(9),dataArray(10),dataArray(11), dataArray(12), dataArray(13), dataArray(14), dataArray(15));
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
%     t = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss.SSS Z');
%     p = posixtime(t);
% %   coming in from arduino: [WOB, drillRPM, drillCurrent, drillPos, mirageAngle, drillLimitSwitchActive, MSE, heaterTemp,heaterPower]
%     fprintf(fid,'%.3f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n', p,da(1),da(2),da(3),da(4),da(5),da(6),da(7),da(8),da(9)); % Write to file  
end

function returnVal = getValuesFromApp(appHand) 
    %[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare,
    %DrillZeroCmd, fakeZeroAcitve, drillCmd, WOBsetpoint, Kp_Drill, Ki_Drill,
    %Kd_Drill, Kp_Heater, Ki_Heater,Kd_Heater, TemperatureSetpoint,
    %HeaterPowerSetpoint, Extraction_ROP_Speed_Cmd, Pump_ROP_Speed_Cmd,
    %Extraction_ROP_Direction_Cmd, Pump_ROP_Direction_Cmd, Mirage_Speed_Cmd, Mirage_Direction_Cmd, ExtractionZeroCmd]
    drillCmdMode = appHand.Drilling_Mode; %1 for manual ROP control, 0 for (automatic) pid control
    dir = appHand.ROP_Direction_Cmd; %drill dir
    speed = appHand.ROP_Speed_Cmd; %drill speed
    miragePosition = 99; %mirage position
    rpm = appHand.RPMSpeed_Cmd;
    heater = appHand.Heater_Cmd;
    pump = appHand.Pump_Cmd; %not used
    tare = appHand.Tare_Cmd;
    DrillZeroCmd = appHand.DrillZero_Cmd;
    fakeZero = appHand.FakeZero_Cmd;
    drillCmd = appHand.Drill_Cmd;
    WOBsetpoint = appHand.WOBsetpoint;
    Kp_Drill = appHand.Kp_Drill;
    Ki_Drill = appHand.Ki_Drill;
    Kd_Drill = appHand.Kd_Drill;
    Kp_Heater = appHand.Kp_Heater;
    Ki_Heater = appHand.Ki_Heater;
    Kd_Heater = appHand.Kd_Heater;
    TemperatureSetpoint = appHand.TemperatureSetpoint;
    HeaterPowerSetpoint = appHand.HeaterPowerSetpoint;
    Extraction_ROP_Speed_Cmd = appHand.Extraction_ROP_Speed_Cmd;
    Pump_ROP_Speed_Cmd = appHand.Pump_ROP_Speed_Cmd;
    Extraction_ROP_Direction_Cmd = appHand.Extraction_ROP_Direction_Cmd;
    Pump_ROP_Direction_Cmd = appHand.Pump_ROP_Direction_Cmd;
    Mirage_Speed_Cmd = appHand.Mirage_Speed_Cmd;
    Mirage_Direction_Cmd = appHand.Mirage_Direction_Cmd;
    ExtractionZeroCmd = appHand.ExtractionZero_Cmd;
    if (tare == 1)
        appHand.Tare_Cmd = 0; %reset to 0 in app
    end
    if (ExtractionZeroCmd == 1)
        appHand.ExtractionZero_Cmd = 0; %reset to 0 in app
    end
     if (DrillZeroCmd == 1)
        appHand.DrillZero_Cmd = 0; %reset to 0 in app
    end
    
    returnVal = drillCmdMode + "," + dir + "," + speed + "," + miragePosition + "," ...
        + rpm + "," + heater + "," + pump + "," + tare + "," + DrillZeroCmd + "," + fakeZero + "," ...
        + drillCmd + "," + WOBsetpoint + "," + Kp_Drill + "," + Ki_Drill + "," + Kd_Drill+ "," ...
        + Kp_Heater + "," + Ki_Heater + "," + Kd_Heater + "," + TemperatureSetpoint + "," ...
        + HeaterPowerSetpoint + "," + Extraction_ROP_Speed_Cmd + "," + Pump_ROP_Speed_Cmd + "," ...
        + Extraction_ROP_Direction_Cmd + "," + Pump_ROP_Direction_Cmd + "," ...
        + Mirage_Speed_Cmd + "," + Mirage_Direction_Cmd + "," + ExtractionZeroCmd;
end

function readSerialData(src,~)
    global data;
    global dataRecieved;
    data = readline(src);
    dataRecieved = true;
end

function setAppData(appHandle, dataArray) %LoadCellLeftValue, LoadCellRightValue, DrillCurrent, HeaterPower, HeaterTemp
%     WOB = round(str2double(dataArray(1)), 1);
%     drillRPM = round(str2double(dataArray(2)), 1);
%     drillCurrent = round(str2double(dataArray(3)), 1); 
%     drillPos = round(str2double(dataArray(4)), 1);
%     mirageAngle = round(str2double(dataArray(5)), 1);
%     limitSwitchReached = str2double(dataArray(6));
%     MSE = round(str2double(dataArray(7)), 1);
%     heaterTemp = round(str2double(dataArray(8)), 1);
%     heaterPower = round(str2double(dataArray(9)), 1);
    %-----send to app-----
%     appHandle.WOBNEditField.Value = WOB;
%     appHandle.DrillRPMEditField.Value = drillRPM;
%     appHandle.DrillPosmmEditField.Value = drillPos;
%     appHandle.MiragePosdegEditField.Value = mirageAngle;
%     appHandle.DrillCurrentAEditField.Value = drillCurrent;
%     appHandle.HeaterPowerEditField.Value = heaterPower;
%     appHandle.HeaterTempEditField.Value = heaterTemp;
%     appHandle.MSEEditField.Value = MSE;
    
%     if (limitSwitchReached == 1) 
%         appHandle.LimitSwitchReachedLamp.Color = [0,1,0]; %rgb
%     else
%         appHandle.LimitSwitchReachedLamp.Color = [1,1,1];
%     end
end


 
 
