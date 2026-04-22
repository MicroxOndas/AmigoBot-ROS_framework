function [v, w, is_clear] = obstacleAvoidance(sensors, config)
%OBSTACLEAVOIDANCE Reactive obstacle avoidance using a Virtual Force Field.
%
%  Each sonar and LiDAR ray within the influence radius generates a
%  repulsive force vector.  The resultant vector is used to compute a
%  heading correction and scaled forward speed.
%
%  SIGNATURE
%    [v, w, is_clear] = obstacleAvoidance(sensors, config)
%
%  INPUTS
%    sensors  - Sensor struct from readSensors()
%    config   - Parameter struct:
%                 .influence_radius  [m]    Sensor influence radius
%                 .emergency_dist    [m]    Stop / hard-turn threshold
%                 .clear_dist        [m]    "Path clear" threshold
%                 .max_v             [m/s]  Maximum allowed speed
%                 .max_w             [rad/s] Maximum turn speed
%
%  OUTPUTS
%    v        - Forward velocity command  [m/s]
%    w        - Angular velocity command  [rad/s]
%    is_clear - true when no obstacle is within clear_dist in front

R = config.influence_radius;

% ======================================================================
% 1. ACCUMULATE REPULSIVE FORCES FROM SONARS
% ======================================================================
% Coordinate frame: x = robot forward, y = robot left (ROS standard)
% Force pushes robot *opposite* to each obstacle direction.
fx = 0;
fy = 0;

for i = 1:8
    d     = sensors.sonar(i);
    angle = sensors.sonar_angles(i);

    if isnan(d) || d >= R
        continue;
    end
    d = max(d, 0.05);  % Clamp to avoid singularity

    % Linear potential gradient: weight grows from 0 at R to 1 at ~0
    weight = (R - d) / R;

    % Repulsive force: away from the obstacle (oppose the sonar direction)
    fx = fx - weight * cos(angle);
    fy = fy - weight * sin(angle);
end

% ======================================================================
% 2. ACCUMULATE REPULSIVE FORCES FROM LIDAR (FRONT SECTOR ±60°)
% ======================================================================
if ~isempty(sensors.laser_ranges)
    front_mask = abs(sensors.laser_angles) <= pi/3;
    angles_f   = sensors.laser_angles(front_mask);
    ranges_f   = sensors.laser_ranges(front_mask);

    for j = 1:numel(ranges_f)
        d     = ranges_f(j);
        angle = angles_f(j);

        if isnan(d) || d >= R
            continue;
        end
        d = max(d, 0.05);

        % LiDAR has many rays – down-weight relative to sparse sonars
        weight = 0.4 * (R - d) / R;
        fx = fx - weight * cos(angle);
        fy = fy - weight * sin(angle);
    end
end

% ======================================================================
% 3. COMPUTE VELOCITY COMMANDS FROM RESULTANT FORCE
% ======================================================================
force_mag = hypot(fx, fy);

% ---- Emergency stop / hard turn (obstacle very close ahead) ----------
if sensors.laser_min_front < config.emergency_dist
    v = 0;
    % Turn toward the side with more space (sign of fy indicates which)
    % If fy > 0 more obstacles on left → turn right; fy < 0 → turn left
    if abs(fy) > 0.1
        w = -sign(fy) * config.max_w;
    else
        % Symmetric: choose based on slight sonar imbalance
        left_sum  = sum(sensors.sonar(1:3), 'omitnan');
        right_sum = sum(sensors.sonar(4:6), 'omitnan');
        w = sign(right_sum - left_sum) * config.max_w;
    end
    is_clear = false;
    return;
end

% ---- No significant obstacles ----------------------------------------
if force_mag < 0.05
    v        = config.max_v;
    w        = 0;
    is_clear = true;
    return;
end

% ---- General avoidance -----------------------------------------------
% Desired heading: direction of the net repulsive vector
desired_heading = atan2(fy, fx);   % In robot frame

% Angular output: steer toward the repulsive escape direction
w = max(-config.max_w, min(config.max_w, 2.5 * desired_heading));

% Forward speed: full speed when aligned, reduced when highly misaligned
forward_component = max(0, cos(desired_heading));
v = config.max_v * forward_component * max(0, 1 - force_mag);
v = max(0, min(config.max_v, v));

% ---- Report clearance status -----------------------------------------
front_sonars = sensors.sonar(2:5);   % Indices 2–5: front-arc sonars
front_ok     = all(isnan(front_sonars) | front_sonars > config.clear_dist);
lidar_ok     = sensors.laser_min_front > config.clear_dist;
is_clear     = front_ok && lidar_ok;
end
