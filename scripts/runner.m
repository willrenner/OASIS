running = true;
rop_speed_cmd = 0; %between 1 and x
rop_direction_cmd = 0; %down is 1, up is -1, stop is 0
drilling_mode = 0; %0 is manual, 1 is automatic

t = tiledlayout(2,1);
ax1 = nexttile;
ax2 = nexttile;
hold([ax1 ax2],'on');
grid;

tStart = tic; %start timer
while (running)

    if (drilling_mode == '0') %manual mode
            
    elseif (drilling_mode == '1') %automatic mode

    end
    
    tCurrent(end + 1) = toc(tStart);
    plot(ax1, tCurrent(end), data(end), '.k');
    plot(ax2, tCurrent(end), data(end), '.k');
end