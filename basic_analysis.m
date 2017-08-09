M = csvread('log.csv',1,0);
%M = csvread('log-example.csv',1,0);
[n_frames, n_cars] = size(M);
n_cars = (n_cars - 1)/6;
times = M(:,1);

for ii = 0:(n_cars - 1)
    % Plot trajectory
    current_fig = figure(ii + 1);
    set(current_fig,'Position',[100,100,700,600]);
    clf;
    subplot(2,2,1:2);
    x_position = M(:,(3 + ii*6));
    y_position = M(:,(4 + ii*6));
    plot(x_position, y_position, '-or',...
        'MarkerSize', 4);
    xlim([-50,1250]);
    ylim([-50,950]);
    xlabel('x-coordinate (mm)');
    ylabel('y-coordinate (mm)');
    title('Car position over time');
    
    % Graph speed
    subplot(2,2,3);
    speeds = sqrt(M(:,(5 + ii*6)).^2 + M(:,(6 + ii*6)).^2);
    plot(times, speeds, '-ob',...
        'MarkerSize', 4);
    xlabel('time (s)');
    ylabel('speed (mm/s)');
    title('Car speed');
    
    % Graph orientation
    subplot(2,2,4);
    orientations = M(:,(7 + ii*6));
    plot(times, orientations, '-ob',...
        'MarkerSize', 4);
    ylim([-10,370]);
    xlabel('time (s)');
    ylabel('orientation (degreess)');
    title('Car orientation');
    
    % Calculate statistics
    mean_area = mean(M(:,(2 + 6*ii)));
    mean_x = mean(M(:,(3 + 6*ii)));
    mean_y = mean(M(:,(4 + 6*ii)));
    mean_vx = mean(M(:,(5 + 6*ii)));
    mean_vy = mean(M(:,(6 + 6*ii)));
    mean_speed = mean(speeds);
    mean_th = mean(M(:,(7 + 6*ii)));
    
    sd_area = std(M(:,(2 + 6*ii)));
    sd_x = std(M(:,(3 + 6*ii)));
    sd_y = std(M(:,(4 + 6*ii)));
    sd_vx = std(M(:,(5 + 6*ii)));
    sd_vy = std(M(:,(6 + 6*ii)));
    sd_speed = std(speeds);
    sd_th = std(M(:,(7 + 6*ii)));
    
    % Print output
    fprintf("\n");
    fprintf("Car %i statistics:\n", ii + 1);
    fprintf("  Area:     %5.1f  %5.1f\n", mean_area, sd_area);
    fprintf("  x:        %5.1f  %5.1f\n", mean_x, sd_x);
    fprintf("  y:        %5.1f  %5.1f\n", mean_y, sd_y);
    fprintf("  v_x:      %5.1f  %5.1f\n", mean_vx, sd_vx);
    fprintf("  v_y:      %5.1f  %5.1f\n", mean_vy, sd_vy);
    fprintf("  speed:    %5.1f  %5.1f\n", mean_speed, sd_speed);
    fprintf("  Orient.:  %5.1f  %5.1f\n", mean_th, sd_th);
end

% Calculate speed
frame_times = diff(times);
mean_fps = mean(frame_times.^(-1));
sd_fps = std(frame_times.^(-1));
fprintf("\n");
fprintf("Frames per second:  %4.2f  %4.2f\n\n", mean_fps, sd_fps);

  