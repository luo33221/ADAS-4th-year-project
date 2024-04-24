%
load('filtered4.mat')


[M, N] = size(x_filtered);
[M_time, ~] = size(cumulative_time);
num_points_per_scan = N;
num_scans = M;

% check if the nuumber of scan and cumumlative time match in amount
% if M_time ~= num_scans
%     error('Mismatch in the number of scans and cumulative time.');
% end

ptClouds = cell(num_scans, 1);
transforms = cell(num_scans - 1, 1);
for i = 1:num_scans - 1
    
    x_coordinates_current = x_filtered(i, :)';
    y_coordinates_current = y_filtered(i, :)';
    x_coordinates_next = x_filtered(i+1, :)';
    y_coordinates_next = y_filtered(i+1, :)';
    
    current_points = [x_coordinates_current, y_coordinates_current, zeros(size(x_coordinates_current))];
    next_points = [x_coordinates_next, y_coordinates_next, zeros(size(x_coordinates_next))];
    
    current_ptCloud = pointCloud(current_points);
    next_ptCloud = pointCloud(next_points);
    
    [tform, ~] = pcregistericp(current_ptCloud, next_ptCloud, 'Metric', 'pointToPoint');
    transforms{i} = tform;
end

save('transforms2.mat', 'transforms');


figure;

% Plot the transformed point clouds
for i = 2:num_scans
    transformed_points = [x_filtered(i, :)', y_filtered(i, :)', zeros(numel(x_filtered(i, :)), 1)];
    transformed_ptCloud = pointCloud(transformed_points);
    pcshow(transformed_ptCloud, 'MarkerSize', 50);
end

hold off;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Registered Point Clouds');

