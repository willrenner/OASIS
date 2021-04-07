% bool getCurrentSensorValue(void*) { //analog read is slow
%     float val = -0.04757 * analogRead(currentSensorPin) + 24.36; //eqn to get amperage, y = mx + b by testing
%     val = sqrt(pow(val, 2) + pow(drillCurrent, 2)); //kinda RMS not really, may not even work who knows
%     drillCurrent = 0.1*val + 0.9*drillCurrent; //complementary filter
%     return true;
% }

% timer.every(1/currentSensorRate * 1000, getCurrentSensorValue);

% rate = 120

rate = 120;