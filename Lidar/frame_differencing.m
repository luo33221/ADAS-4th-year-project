frame differencing

% Processing the data
% Load the .bag file
bag = rosbag('s_curve2.bag');

% Read LiDAR messages
lidar_msgs = readMessages(select(bag, 'Topic', '/scan'));

% Initialize arrays to store data
x = zeros(length(lidar_msgs), 720); 
y = zeros(length(lidar_msgs), 720); 
ranges_mean = zeros(length(lidar_msgs), 1);

% Obtain discrete angle values using minimum angle and angle increments,
% and convert the angles from radians form to degrees
min_angle = lidar_msgs{1}.AngleMin;
angle_increment = lidar_msgs{1}.AngleIncrement;
angles = rad2deg(min_angle + (0:length(lidar_msgs{1}.Ranges)-1) * angle_increment);

% Initialize the time values by setting default to 0
cumulative_time = zeros(length(lidar_msgs), 1);
previous_time = 0;

% Read from each lidar scan 
for i = 1:length(lidar_msgs)
    % Read range values from lidar message
    ranges_current_scan = lidar_msgs{i}.Ranges;    
    
    valid_indices = isfinite(ranges_current_scan) & (ranges_current_scan < 9.2);
   
    ranges_current_scan = ranges_current_scan(valid_indices);
    ranges_mean(i) = mean(valid_indices);
    

    % Calculate the cumulative time
    scan_time = lidar_msgs{i}.ScanTime;
    cumulative_time(i) = previous_time + scan_time;
    previous_time = cumulative_time(i);
    
    % Calculate Cartesian coordinates from polar readings, 
    for j = 1:length(ranges_current_scan)
        angle_rad = deg2rad(angles(j));
        x(i, j) = ranges_current_scan(j) * cosd(angles(j));
        y(i, j) = ranges_current_scan(j) * sind(angles(j));
    end

    % Calculate the mean range for the current scan
    ranges_mean(i) = mean(ranges_current_scan);
end

% Create a structure array to store x, y coordinates, and time
lidar_data = struct('x', x, 'y', y, 'time', cumulative_time);
% Save the structure array to a MAT file
save('processed_struct2.mat', 'lidar_data');

filtering (Median Filter)
% Load processed data
load('processed_struct2.mat');

% Apply median filter to x and y coordinates separately to reduce noise;
window_size = 3;
x_filtered = medfilt2(lidar_data.x, [window_size, window_size]);
y_filtered = medfilt2(lidar_data.y, [window_size, window_size]);

% Save the filtered data into a new mat file;
filtered_mat_filename = 'filtered2.mat';
save(filtered_mat_filename, 'x_filtered', 'y_filtered', 'cumulative_time');

Frame differencing
data = load('filtered2.mat');

% Extract x and y coordinates
x_coords = data.x_filtered; 
y_coords = data.y_filtered; 

distance = zeros(size(x_coords));
centroids = zeros(size(x_coords, 1), 2); 

moved_points_data.moved_x = {};
moved_points_data.moved_y = {};

% Iterating over scans
for num_scan = 2:size(x_coords, 1)
    for coordinates = 1:size(x_coords, 2)
        % Calculate the distance between corresponding pairs of coordinates
        dx = x_coords(num_scan, coordinates) - x_coords(num_scan-1, coordinates);
        dy = y_coords(num_scan, coordinates) - y_coords(num_scan-1, coordinates);
        distance(num_scan, coordinates) = sqrt(dx^2 + dy^2);
    end

    % Setting threshold to filter movements
    threshold1 = 0.2; % based on pedestrian moving speed
    threshold2 = 1.5;
    moved_points = (distance(num_scan, :) > threshold1) & (distance(num_scan, :) < threshold2);


    if any(moved_points)
        moved_x = x_coords(num_scan, moved_points);
        moved_y = y_coords(num_scan, moved_points);
        centroids(num_scan, 1) = mean(moved_x, 'omitnan');
        centroids(num_scan, 2) = mean(moved_y, 'omitnan');
        
        % Store the moved points for each scan
        moved_points_data.moved_x{end+1} = moved_x;
        moved_points_data.moved_y{end+1} = moved_y;
    end
end

save('moved_points_data.mat', 'moved_points_data');


% Clustering
load('moved_points_data.mat');

min_speed = 0.15; % Minimum speed in meters per second
max_speed = 3; % Maximum speed in meters per second

loaded_moved_x = moved_points_data.moved_x;
loaded_moved_y = moved_points_data.moved_y;

epsilon = 0.05;  
minpts = 3; 

% Each scan processing
for i = 1:num_scan-1
    % Check if the inputs are cells and extract numeric data if necessary
    if iscell(loaded_moved_x)
        x = loaded_moved_x{i}; % Assuming each cell contains data for a scan
    else
        x = loaded_moved_x; % Use directly if not a cell
    end
    
    if iscell(loaded_moved_y)
        y = loaded_moved_y{i}; % Assuming each cell contains data for a scan
    else
        y = loaded_moved_y; % Use directly if not a cell
    end

    % Concatenate x and y into a single matrix
    XY = [x, y];

    % DBSCAN clustering using MATLAB packages
    [labels, corepts] = dbscan(XY, epsilon, minpts);

    % Calculate centroids for clustered points
    if any(labels > 0) 
        centroids(i, 1) = mean(x(labels > 0), 'omitnan'); 
        centroids(i, 2) = mean(y(labels > 0), 'omitnan');
    else 
        centroids(i, :) = [NaN, NaN];
    end

    % Storing clustering result per scan
    all_labels{i} = labels;
end


% Filtering clusters & calculating distance, angle and velocity
valid_centroids = zeros(num_scan, 2);  % Store centroids that meet speed requirements
valid_diplacement = zeros(num_scan, 1);
valid_speeds = zeros(num_scan, 1);
valid_angles = zeros(num_scan, 1);
distance = zeros(num_scan,1);

for i = 1:num_scan
    if all(~isnan(centroids(i, :))) % Check if centroids are valid (not NaN)
        x = centroids(i, 1);
        y = centroids(i, 2);
        distance(i) = sqrt(x^2 + y^2);
    else
        distance(i) = NaN; % Assign NaN if centroid data is invalid
    end
end


% Processing each scan for speed and centroid tracking

for i = 2:num_scan
    if all(~isnan(centroids(i-1, :))) && all(~isnan(centroids(i, :)))
        % Calculate distance between centroids
        dx = centroids(i, 1) - centroids(i-1, 1);
        dy = centroids(i, 2) - centroids(i-1, 2);
        displacement = sqrt(dx^2 + dy^2);
        
        % Fetch scan time from current lidar message
        if isfield(lidar_msgs{i}, 'ScanTime') && lidar_msgs{i}.ScanTime > 0
            scan_time = lidar_msgs{i}.ScanTime;
        else
            scan_time = 1;
        end

        % Calculate speed
        speed = displacement / scan_time;
        speeds(i-1) = speed; 

        % Check if the speed is within the threshold
        if speed >= min_speed && speed <= max_speed
            valid_centroids(i-1, :) = centroids(i-1, :); % Store centroid if speed is valid
            valid_centroids(i, :) = centroids(i, :);

            valid_diplacement(i-1) = displacement;
            valid_speeds(i-1) = speed;

            % Calculate angle of movement from radians to degrees
            angle = atan2(dy, dx) * (180 / pi); 
            valid_angles(i-1) = angle;
        end
    end
end


% Plotting Result
figure;

% Plot range, velocity, and angle over time

subplot(1,3, 1);
plot(cumulative_time, distance(i));
xlabel('Time');
ylabel('Range');
title('Distance over Time');

subplot(1,3, 3);
plot(cumulative_time, valid_speeds(i));
xlabel('Time');
ylabel('Velocity');
title('Velocity over Time');

subplot(1,3, 2);
plot(cumulative_time, valid_angles(i));
xlabel('Time');
ylabel('Angle');
title('Angle over Time');
