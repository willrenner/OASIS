clear;clc;
a = arduino('COM3', 'Mega2560');
s = servo(a, 'D2');

for angle = 0:0.2:1
    writePosition(s, angle);
    current_pos = readPosition(s);
    current_pos = current_pos*180;
    fprintf('Current motor position is %d degrees\n', current_pos);
    pause(2);
end

