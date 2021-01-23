clc;clear;
running = true;
rop_speed_cmd = 0; %between 1 and x
rop_direction_cmd = 0; %down is 1, up is -1, stop is 0
drilling_mode = 0; %0 is manual, 1 is automatic



% figure;
t = tiledlayout(2,1);
t.TileSpacing = 'compact';
t.Padding = 'compact';
t.Title.String = "My title";
t.Title.FontWeight = 'bold';
t.XLabel.String = 'Time(My x-Axis Label)';
t.XLabel.FontSize = 14;

ax1 = nexttile;
ax2 = nexttile;
h1 = animatedline(ax1);
h2 = animatedline(ax2);
hold([ax1 ax2],'on');
grid([ax1, ax2]);

data = 5;
tCurrent = 0;
tStart = tic; %start timer
pause on;
while (running)

%     if (drilling_mode == '0') %manual mode
%             
%     elseif (drilling_mode == '1') %automatic mode
% 
%     end
    data = data + 0.01;
    tCurrent = toc(tStart);
%     plot(ax1, tCurrent, data, '.k');
%     plot(ax2, tCurrent, data, '.k');
    addpoints(h1,tCurrent, data);
    addpoints(h2,tCurrent, data);
    drawnow limitrate; %20 fps graphics updates
%     pause(0.01);
    xlim([ax1, ax2], [tCurrent - 20, tCurrent + 20]);
    ylim([ax1, ax2], [data - 20, data + 20]);
end