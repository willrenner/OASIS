clc;clear;

global leadScrewLead;
global numSteps;

pulsePin = "D1";
directionPin = "D2";
leadScrewLead = 8; %mm/rev
numSteps = 200; %steps/rev, 1.8deg/step
currentYpos = 0; %once limit switch is reached
directionCmd = "down";

currentStep = 0;
rop = 10; %mm/min
run = true;
prevTime = tic;


%****** wont work if  timeSincePrev >= 2*time per step, means will skip steps
while(run)
    timeSincePrev = toc(prevTime);
    timePerStep = getTimeFromROP(rop); %calculate timePerStep
    if (timeSincePrev >= timePerStep)
        if (timeSincePrev >= timePerStep * 2)
            disp("Skipped at least one step"); %error check
        end
        stepOnce(directionCmd); %step stepper motor
        currentStep = currentStep + 1; %increment stepper motor step
        disp("Step count: " +  currentStep);
        prevTime = tic;
    end
    currentYpos = currentStep / numSteps * leadScrewLead; %mm from limit switch
end

function timePerStep_ = getTimeFromROP(ropIn) %ROP in mm/min
    global leadScrewLead;
    global numSteps;
    stepperRevSpeedCmd = ropIn / leadScrewLead / 60; %rev/sec
    stepperRevSpeedCmd = stepperRevSpeedCmd * numSteps; %steps/sec
    timePerStep_ = 1 / stepperRevSpeedCmd; %sec/step
end

function stepOnce(direction) %down if direction = 1
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
end