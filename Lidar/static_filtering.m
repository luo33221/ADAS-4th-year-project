% Load the .bag file
bag = rosbag('s_curve2.bag');
bagInfo = rosbag('info',"10m 5 stop 2.bag"); % obtain bag information;


lidar_msgs = readMessages(select(bag, 'Topic', '/scan')); % select scan topic in the bag

% Initialize parameters to 0
cumulative_time = zeros(length(lidar_msgs), 1);
previous_time = 0;
ranges_mean = zeros(length(lidar_msgs), 1);
centroids = zeros(length(lidar_msgs), 2); 

for i = 1:length(lidar_msgs)
    % Read ranges and angles for angles 340 to 380, narrow down the range
    % of angles
    ranges = lidar_msgs{i}.Ranges(300:420);
    
    % Set a threshold to remove the points representing the wall;
    valid_indices = isfinite(ranges) & (ranges < 9.2);
   
    ranges = ranges(valid_indices);
   
    angles = lidar_msgs{i}.AngleMin + (300:420) * lidar_msgs{i}.AngleIncrement;
    angles = angles(valid_indices); 
    
    % Calculate x and y coordinates
    % note that the angles' is a transpose matrix, so that x and y
    % calculated are a column matrix
    x = ranges .* cos(angles');
    y = ranges .* sin(angles');
    
    % Apply median filter to filter out the noises
    window_size = 2;
    x_filtered = medfilt2(x, [window_size, window_size]);
    y_filtered = medfilt2(y, [window_size, window_size]);
    
    % taking means of x and y to find the centroid of the points
    centroids(i, 1) = mean(x_filtered);
    centroids(i, 2) = mean(y_filtered);
    ranges_mean(i) = mean(ranges);
    
    % Using scan time for each individual scan to find the cumulative time;
    scan_time = lidar_msgs{i}.ScanTime;
    cumulative_time(i) = previous_time + scan_time;
    previous_time = cumulative_time(i);
end

range = zeros(length(centroids), 1);
velocity = zeros(length(centroids), 1);
angle = zeros(length(centroids), 1);

for i = 2:length(centroids)
    % Calculate displacements from centroids
    displacement(i) = norm(centroids(i, :) - centroids(i-1, :));
   
    % Calculate delta time between consecutive scans
    time_diff = cumulative_time(i) - cumulative_time(i-1);
    if ranges_mean(i) < ranges_mean(i-1)
        velocity(i) = -1 * displacement(i) / time_diff;
    else 
        velocity(i) = displacement(i) / time_diff;
    end
    
    % using inverse tangent to find the angle of the object
    angle(i) = atan2(centroids(i, 2) - centroids(i-1, 2), centroids(i, 1) - centroids(i-1, 1));
end

figure;

% plot(cumulative_time, ranges_mean);
% xlabel('Time');
% ylabel('Distance');
% title('Distance over Time for 10m 5 stops for LIDAR');


% Plot range, velocity, and angle over time

subplot(1,3, 1);
plot(cumulative_time, ranges_mean);
xlabel('Time');
ylabel('Range');
title('Distance over Time');

subplot(1,3, 3);
plot(cumulative_time, velocity);
xlabel('Time');
ylabel('Velocity');
title('Velocity over Time');

subplot(1,3, 2);
plot(cumulative_time, angle);
xlabel('Time');
ylabel('Angle');
title('Angle over Time');

filename = '10m 5 stop 2.csv';

% Create a matrix to store the data
data = [angle, ranges_mean, velocity];

% Write the data matrix to a CSV file
writematrix(data, filename);



% % Plot ranges_mean versus cumulative_time
% plot(cumulative_time, ranges_mean);
% xlabel('Cumulative Time');
% ylabel('Mean Ranges');
% title('Mean Ranges versus Cumulative Time - 10m back and forth');

