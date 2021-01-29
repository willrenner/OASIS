format long
x= 0;
%HAT High Accuracy Timer
%  [TIME] = HAT uses the high-resolution performance counter to get the 
%  time which exceeds one microsecond accuracy.
time = hat;
% do some calculations

while (1)
    time = hat;
    x = x + 1;
    diff = hat - time
%     print(diff)
end