clc;clear;
rop_speed_cmd = '0';
drilling_mode = '1';
a = arduino('COM3', 'Mega2560');

pause on
while (drilling_mode == '1')
   disp("Speed: " + rop_speed_cmd);
   disp("1");
   writeDigitalPin(a, 'D13', 1);
   pause(str2double(rop_speed_cmd) + 0.5);
   disp("0");
   writeDigitalPin(a, 'D13', 0);
   pause(str2double(rop_speed_cmd) + 0.5);
end