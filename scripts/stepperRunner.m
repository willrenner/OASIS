pulsePin = "D1";
directionPin = "D2";
leadScrewLead = 8; %mm/rev
numSteps = 200; %steps/rev, 1.8deg/step
currentYpos = 0; %once limit switch is reached
dt = 1; %sec
t1 = tick;
t2 = toc;
%pwm freq = 100, so if on 1% 10 times per sec
timePerStep = 
currentStep = 0;

function setROP(rop) %rop in cm/min
    rop = rop / 10;  %now mm/min
    stepperRevSpeedCmd = rop / leadScrewLead / 60; %rev/sec
    stepperRevSpeedCmd = stepperRevSpeedCmd * numSteps; %steps/sec
    timePerStep = 1 / stepperRevSpeedCmd;
end

function stepOnce() 
    
end