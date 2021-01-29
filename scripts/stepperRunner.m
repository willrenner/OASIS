clc;clear;
clear global;
% hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
global a;
a = arduino('COM3', 'Mega2560');

global leadScrewLead;
global numSteps;
global currentStep;
global prevTime;
global timePerStep;
global prevLogTime;
global currentYpos;
global stepperRevSpeedCmd;
global directionCmd;
global pulsePin;
global directionPin;
global dt;
global loopCount;

loopCount = 0;


dt = 0.001;

pulsePin = "D2";
directionPin = "D3";
leadScrewLead = 8; %mm/rev
numSteps = 100; %steps/rev, 1.8deg/step
currentYpos = 0; %once limit switch is reached
directionCmd = "down";


currentStep = 0;
ROP = 200; %mm/min
run = true;
prevTime = tic;
prevLogTime = tic;
pause on;
time1 = tic;
%****** wont work if  timeSincePrev >= 2*time per step, means will skip steps
while(run)
    loopCount = loopCount + 1;
    if (toc(time1) >= 1)
        disp("fps: " + loopCount);
        loopCount = 0;
        time1 = tic;
    end
%     if (~ishandle(hWaitbar)) % Stop if cancel button was pressed
%         disp('Stopped by user');
%         break;
%     end
    timeSincePrev = toc(prevTime);
    writeDigitalPin(a, pulsePin, 0); %set pulse to low before anything
    checkIfTimeToStep(timeSincePrev);
    timePerStep = getTimeFromROP(ROP); %calculate timePerStep
    currentYpos = currentStep / numSteps * leadScrewLead; %mm from limit switch
    
    if (toc(prevLogTime) > 0.5)
        logData();
        prevLogTime = tic;
    end
end

function timePerStep_ = getTimeFromROP(ropIn) %ROP in mm/min
    global leadScrewLead;
    global numSteps;
    global stepperRevSpeedCmd;
    if (ropIn <= 0.5) %less than 0.5 mm per minute, just return max time per step so it won't move
        timePerStep_ = 9999; %sec/step
        stepperRevSpeedCmd = 0;
    else
        stepperRevSpeedCmd = ropIn / leadScrewLead / 60; %rev/sec
        stepperRevSpeedCmd = stepperRevSpeedCmd * numSteps; %steps/sec
        timePerStep_ = 1 / stepperRevSpeedCmd; %sec/step
    end
end

function checkIfTimeToStep(timeSincePrev_) %****** wont work if  timeSincePrev >= 2*time per step, means will skip steps
    global directionCmd;
    global timePerStep;
    if (timeSincePrev_ >= timePerStep)
        if (timeSincePrev_ >= timePerStep * 2)
            disp("Skipped at least one step"); %error check
        end
        stepOnce(directionCmd); %step stepper motor
    end   
end

function stepOnce(direction) 
    global currentStep;
    global prevTime;
    global directionPin;
    global pulsePin;
    global dt;
    global a;
    if (direction == "down")
        writeDigitalPin(a, directionPin, 1) %1 for down?
        writeDigitalPin(a, pulsePin, 1); %each step happens per pulse
        %sleep for a tiny bit??
%         pause(dt);
%         writeDigitalPin(a, pulsePin, 0);
    elseif (direction == "up")
        writeDigitalPin(a, directionPin, 0) %0 for up?
        writeDigitalPin(a, pulsePin, 1); %each step happens per pulse
        %sleep for a tiny bit??
%         pause(dt);
%         writeDigitalPin(a, pulsePin, 0);
    end
        prevTime = tic;
        currentStep = currentStep + 1; %increment stepper motor step        
end

function logData() 
    global currentStep;
    global currentYpos;
    global timePerStep;
    global stepperRevSpeedCmd;
    disp("Step count: " +  currentStep);
    disp("Y pos (mm): " +  currentYpos);
    disp("Time per step: " +  timePerStep);
    disp("Steps per sec: " +  stepperRevSpeedCmd);
end