clc;clear;
hWaitbar = waitbar(0, 'Running...', 'Name', 'AARC','CreateCancelBtn','delete(gcbf)');
% a = arduino('COM3', 'Mega2560');

global leadScrewLead;
global numSteps;
global currentStep;
global prevTime;
global timePerStep;
global prevLogTime;
global currentYpos;

pulsePin = "D1";
directionPin = "D2";
leadScrewLead = 8; %mm/rev
numSteps = 200; %steps/rev, 1.8deg/step
currentYpos = 0; %once limit switch is reached
directionCmd = "down";

currentStep = 0;
ROP = 1000; %mm/min
run = true;
prevTime = tic;
prevLogTime = tic;

%****** wont work if  timeSincePrev >= 2*time per step, means will skip steps
while(run)
    if (~ishandle(hWaitbar)) % Stop if cancel button was pressed
        disp('Stopped by user');
        break;
    end
    timeSincePrev = toc(prevTime);
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
        prevTime = tic;
        currentStep = currentStep + 1; %increment stepper motor step        
end

function logData() 
    global currentStep;
    global currentYpos;
    global timePerStep;
    disp("Step count: " +  currentStep);
    disp("Y pos (mm): " +  currentYpos);
    disp("Time per step: " +  timePerStep);
end