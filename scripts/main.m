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
fileID = fopen("../logs/logTest1.txt", 'a'); %appends to end of file, or creates file and writes to it
changed = false;
changedTimer = tic;
count = 0;
radius = 0.01905;
augerArea = pi * radius^2;
n = 0.1; %drill efficiency
rpm = 1; %rpm cancels out anyways
global drillTorque; drillTorque = 0;
global mse_wob; mse_wob = 0;
global mse_torque; mse_torque = 0;
global MSE; MSE = 0;
global ROP; ROP = 0; %m per sec

while(running)
    try
        if (toc(changedTimer) > 0.1) %sets a send rate
            changed = true;
            changedTimer = tic;
        end
        if (changed)
            linetowrite = getValuesFromApp(appHandle);
            writeline(serialPort, linetowrite);
%             disp("Sent data");
            changed = false;
        end
        if (dataRecieved)
            %indecies =====>     //LoadCellLeftValue, LoadCellRightValue, totalSystemCurrent, HeaterPower, HeaterTemp, DrillPos, ExtractionPos, MiragePos, LoadCellCombined ----- ACTIVE

            dataArray = strsplit(data, ',');
            sizeOfArr = size(dataArray);
            if (sizeOfArr(2) > 1) %if array contains a comma, meaning not a Serial debug statement
                %fprintf("LoadCellLeftValue: %4.2f, LoadCellRightValue: %4.2f, DrillCurrent: %4.2f, HeaterPower: %4.2f, HeaterTemp: %4.2f, DrillPos: %4.2f, Extr. Pos: %4.2f\n ",dataArray(1),dataArray(2),dataArray(3),dataArray(4),dataArray(5),dataArray(6),dataArray(7));
                LoadCellLeftValue = str2double(dataArray(1));
                LoadCellRightValue = str2double(dataArray(2));
                TotalCurrent = str2double(dataArray(3));
                HeaterPower = str2double(dataArray(4));
                HeaterTemp = str2double(dataArray(5));
                DrillPos = str2double(dataArray(6));
                ExtrPos = str2double(dataArray(7));
                MiragePos = str2double(dataArray(8));
                LoadCellCombined = str2double(dataArray(9));
                drillTorque = (TotalCurrent * 120 * 60 * n) / (rpm * 2 * pi);
                mse_wob = LoadCellCombined / augerArea;
                if (ROP < 0.0001) %0.1 mm per sec
                    ROP = 9999999999;
                end
                mse_torque = drillTorque * rpm/(augerArea * ROP);
                MSE = LoadCellCombined / augerArea + drillTorque * rpm/(augerArea * ROP);
                incomingData = sprintf("LoadCellLeftValue: %4.2f \nLoadCellRightValue: %4.2f \nTotalCurrent: %4.2f \nHeaterPower: %4.f \nHeaterTemp: %4.2f \nDrillPos: %4.2f \nExtr.Pos: %4.2f \nMirage.Pos: %4.2f \nLoad Cell Combined: %4.2f \nDrill Torque: %4.2f \nmse_wob: %4.2f \nmse_torque: %4.2f \nMSE: %4.2f\n ", ...
                    LoadCellLeftValue,LoadCellRightValue,TotalCurrent,HeaterPower,HeaterTemp,DrillPos,ExtrPos,MiragePos,LoadCellCombined, drillTorque, mse_wob, mse_torque, MSE);
                
                appHandle.IncomingDataLabel.Text = incomingData; 
                setAppData(appHandle, dataArray); %change data in app
                writeDataToFile(dataArray, fileID) %log to file
            else %meaning Serial debug statment, and not data array
                %disp("Message from arduino: ");
                %disp(data);
                appHandle.IncomingDebugDataLabel.Text = data; 
                appHandle.IncomingDebugDataLabel_2.Text = data; 
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


%LoadCellLeftValue,LoadCellRightValue,TotalCurrent,HeaterPower,HeaterTemp,
%DrillPos,ExtrPos,MiragePos,LoadCellCombined, drillTorque, mse_wob, mse_torque, MSE
function writeDataToFile(da, fid)
    global drillTorque;global mse_wob; global mse_torque; global MSE;
    t = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss.SSS Z');
    p = posixtime(t);
    %coming in from arduino:     //LoadCellLeftValue, LoadCellRightValue, totalSystemCurrent, HeaterPower, HeaterTemp, DrillPos, ExtractionPos, MiragePos, LoadCellCombined ----- ACTIVE
    fprintf(fid,'%.3f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n', p,da(1),da(2),da(3),da(4),da(5),da(6),da(7),da(8),da(9), drillTorque, mse_wob, mse_torque, MSE); % Write to file  
end

function returnVal = getValuesFromApp(appHand) 
    global ROP; %m per sec
    drillCmdMode = appHand.Drilling_Mode; %1 for manual ROP control, 0 for (automatic) pid control
    dir = appHand.ROP_Direction_Cmd; %drill dir
    speed = appHand.ROP_Speed_Cmd; %drill speed
    ROP = speed / 1000;
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
    DrillPower = appHand.DrillPower;
    if (tare == 1)
        appHand.Tare_Cmd = 0; %reset to 0 in app
    end
    if (ExtractionZeroCmd == 1)
        appHand.ExtractionZero_Cmd = 0; %reset to 0 in app
    end
     if (DrillZeroCmd == 1)
        appHand.DrillZero_Cmd = 0; %reset to 0 in app
    end
%     display("pumpdir " + Pump_ROP_Direction_Cmd);
    returnVal = drillCmdMode + "," + dir + "," + speed + "," + miragePosition + "," ...
        + rpm + "," + heater + "," + pump + "," + tare + "," + DrillZeroCmd + "," + fakeZero + "," ...
        + drillCmd + "," + WOBsetpoint + "," + Kp_Drill + "," + Ki_Drill + "," + Kd_Drill+ "," ...
        + Kp_Heater + "," + Ki_Heater + "," + Kd_Heater + "," + TemperatureSetpoint + "," ...
        + HeaterPowerSetpoint + "," + Extraction_ROP_Speed_Cmd + "," + Pump_ROP_Speed_Cmd + "," ...
        + Extraction_ROP_Direction_Cmd + "," + Pump_ROP_Direction_Cmd + "," ...
        + Mirage_Speed_Cmd + "," + Mirage_Direction_Cmd + "," + ExtractionZeroCmd + "," + DrillPower;
end
%[drillCmdMode, dir, speed, miragePosition, rpm, heater, pump, tare,
    %DrillZeroCmd, fakeZeroAcitve, drillCmd, WOBsetpoint, Kp_Drill, Ki_Drill,
    %Kd_Drill, Kp_Heater, Ki_Heater,Kd_Heater, TemperatureSetpoint,
    %HeaterPowerSetpoint, Extraction_ROP_Speed_Cmd, Pump_ROP_Speed_Cmd,
    %Extraction_ROP_Direction_Cmd, Pump_ROP_Direction_Cmd, Mirage_Speed_Cmd, Mirage_Direction_Cmd, ExtractionZeroCmd, DrillPower]

function readSerialData(src,~)
    global data;
    global dataRecieved;
    data = readline(src);
    dataRecieved = true;
end

function setAppData(appHandle, dataArray) %    //LoadCellLeftValue, LoadCellRightValue, totalSystemCurrent, HeaterPower, HeaterTemp, DrillPos, ExtractionPos, MiragePos, LoadCellCombined ----- ACTIVE

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


 
 
