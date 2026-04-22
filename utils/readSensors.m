function sensors = readSensors(sonar_subs, laser_sub)
%READSENSORS Aggregate all sensor data into a unified struct.
%
%  sensors = readSensors(sonar_subs, laser_sub)
%
%  Inputs:
%    sonar_subs  - 1×8 cell array of rossubscriber handles {sonar0…sonar7}
%    laser_sub   - rossubscriber handle for the LiDAR
%
%  Output struct fields:
%    sensors.sonar         - [1×8]  Sonar distances [m], NaN = invalid
%    sensors.sonar_angles  - [1×8]  Sonar mounting angles [rad] (ROS convention)
%    sensors.laser_ranges  - [1×N]  LiDAR range readings [m], NaN = invalid
%    sensors.laser_angles  - [1×N]  LiDAR angle for each ray [rad]
%    sensors.laser_min_front - Minimum LiDAR range in ±45° frontal sector [m]
%
%  Sonar mounting (ROS convention: 0 = fwd, positive = left/CCW):
%    Index: 1    2    3    4    5    6    7    8
%    Sonar: 0    1    2    3    4    5    6    7
%    Angle:+90° +54° +18° -18° -54° -90° +135°-135°

% --- Sonar mounting angles [rad] --------------------------------------
% Sonars 0-5: arc from left (+90°) to right (-90°) through front
% Sonars 6-7: rear diagonals (+135° left-rear, -135° right-rear)
sensors.sonar_angles = deg2rad([90, 54, 18, -18, -54, -90, 135, -135]);

MAX_VALID = 3.5;  % [m] hard cap for reasonable robot environments

% --- Read each sonar --------------------------------------------------
sensors.sonar = nan(1, 8);
for i = 1:8
    msg = sonar_subs{i}.LatestMessage;
    if isempty(msg)
        continue;
    end
    r = double(msg.Range_);
    if r >= msg.MinRange && r <= msg.MaxRange && r <= MAX_VALID
        sensors.sonar(i) = r;
    end
end

% --- Read LiDAR -------------------------------------------------------
laser_msg = laser_sub.LatestMessage;
if ~isempty(laser_msg)
    ranges    = double(laser_msg.Ranges);
    angle_min = double(laser_msg.AngleMin);
    angle_inc = double(laser_msg.AngleIncrement);
    N         = numel(ranges);

    sensors.laser_angles = angle_min + (0:N-1) * angle_inc;

    % Invalidate out-of-range readings
    valid = ranges >= double(laser_msg.RangeMin) & ...
            ranges <= double(laser_msg.RangeMax);
    ranges(~valid) = NaN;
    sensors.laser_ranges = ranges;

    % Minimum range in the ±45° front sector
    front = abs(sensors.laser_angles) <= pi/4;
    front_ranges = ranges(front);
    if any(~isnan(front_ranges))
        sensors.laser_min_front = min(front_ranges(~isnan(front_ranges)));
    else
        sensors.laser_min_front = Inf;
    end
else
    sensors.laser_ranges    = [];
    sensors.laser_angles    = [];
    sensors.laser_min_front = Inf;
end
end
