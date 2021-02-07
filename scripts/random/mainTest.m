clc;clear;
hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
fileID = fopen("../logs/abcd.txt", 'a'); %appends to end of file, or creates file and writes to it
format shortg; %smart formating

global leadScrewLead;
global numSteps;
global currentStep;
global prevTime
global directionCmd;
global WOB;
global ROP;
global augerArea;
global drillTorque;
global drillCurrent;
global drillRPM;


augerArea = pi * (0.02)^2;
WOB = 0;
currentStep = 0;
leadScrewLead = 8; %mm/rev
numSteps = 200; %steps/rev, 1.8deg/step
directionCmd = "down";

pulsePin = "D1";
directionPin = "D2";
limitSwitchPin = "D3";
pumpPin = "D4";
heaterPin = "D5";
loadCellPin = "A1";
currentYpos = 0; %once limit switch is reached


ROP = 0; %mm/min
timePerStep = getTimeFromROP(ROP); %calculate timePerStep
run = true;
homing = true;

prevTime = tic;


while(run)
    while(homing)
        if (readDigitalPin(a, limitSwitchPin) == 0)% not at switch yet
            timeSincePrev = toc(prevTime);
            checkIfTimeToStep(timeSincePrev);
            timePerStep_ = getTimeFromROP(ROP);
        elseif (readDigitalPin(a, limitSwitchPin) == 1) % reached the switch
            homing = false;
            break;
        end
    end
    timeSincePrev = toc(prevTime);
    checkIfTimeToStep(timeSincePrev);
    timePerStep = getTimeFromROP(ROP); %calculate timePerStep
    currentYpos = currentStep / numSteps * leadScrewLead; %mm from limit switch
    
    %update all values
    %get MSE value
    %write to file
end





function timePerStep_ = getTimeFromROP(ropIn) %ROP in mm/min
    global leadScrewLead;
    global numSteps;
    if (ropIn <= 0.5) %less than 0.5 mm per minute, just return max time per step so it won't move
        timePerStep_ = 9999; %sec/step
    else
        stepperRevSpeedCmd = ropIn / leadScrewLead / 60; %rev/sec
        stepperRevSpeedCmd = stepperRevSpeedCmd * numSteps; %steps/sec
        timePerStep_ = 1 / stepperRevSpeedCmd; %sec/step
    end
end

function checkIfTimeToStep(timeSincePrev_) %****** wont work if  timeSincePrev >= 2*time per step, means will skip steps
    global directionCmd;
    if (timeSincePrev_ >= timePerStep)
        if (timeSincePrev >= timePerStep * 2)
            disp("Skipped at least one step"); %error check
        end
        stepOnce(directionCmd); %step stepper motor
    end
    
end

function stepOnce(direction) 
    global currentStep;
    global prevTime;
%     if (direction == "down")
%         writeDigitalPin(a, directionPin, 1) %1 for down?
%         writeDigitalPin(a, pulsePin, 1); %each step happens per pulse
%         %sleep for a tiny bit??
%         writeDigitalPin(a, pulsePin, 0);
%     elseif (direction == "up")
%         writeDigitalPin(a, directionPin, 0) %0 for up?
%         writeDigitalPin(a, pulsePin, 1); %each step happens per pulse
%         %sleep for a tiny bit??
%         writeDigitalPin(a, pulsePin, 0);
%     end
        currentStep = currentStep + 1; %increment stepper motor step
        disp("Step count: " +  currentStep);
        prevTime = tic;
end

function activatePump(command)
    if (command == "on")
        writeDigitalPin(a, pumpPin, 1) %will be connected to relay or transistor
    elseif (command == "off")
        writeDigitalPin(a, pumpPin, 0) %will be connected to relay or transistor
    end
end

function activateHeater(command)
    if (command == "on")
        writeDigitalPin(a, heaterPin, 1) %will be connected to relay or transistor
    elseif (command == "off")
        writeDigitalPin(a, heaterPin, 0) %will be connected to relay or transistor
    end
end

function getWOB()
    voltage = readVoltage(a, loadCellPin);
    %do calcs to convert to a WOB

    
end




function MSE_ = calcMSE()
    global drillTorque;
    global drillCurrent;
    global drillRPM;
    global augerArea;
    global WOB;
    global ROP;
    
    MSE_ = WOB/augerArea + drillTorque*drillRPM/(augerArea*ROP); %MSE equation
end