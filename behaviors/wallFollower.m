function [v, w] = wallFollower(pose, sensors, pid_wall, config)
%WALLFOLLOWER PID-based wall-following behavior.
%
%  Maintains a fixed lateral distance to a wall using side sonars.
%  When the front becomes obstructed the robot turns away from the wall
%  to navigate around a corner.
%
%  SIGNATURE
%    [v, w] = wallFollower(pose, sensors, pid_wall, config)
%
%  INPUTS
%    pose      - Pose struct from readOdometry()  (reserved for extensions)
%    sensors   - Sensor struct from readSensors()
%    pid_wall  - PIDController for lateral wall distance
%    config    - Parameter struct:
%                  .side          'left' | 'right'
%                  .desired_dist  [m]   Target distance to wall
%                  .dt            [s]   Loop period
%                  .max_v         [m/s] Maximum forward speed
%                  .max_w         [rad/s] Maximum turn speed
%                  .front_clear   [m]   Front clearance threshold for turns
%
%  OUTPUTS
%    v  - Forward velocity command  [m/s]
%    w  - Angular velocity command  [rad/s]
%
%  SONAR INDEX MAP (1-based MATLAB, ROS convention: + = left/CCW)
%    Idx: 1    2    3    4    5    6    7    8
%         +90° +54° +18° -18° -54° -90° +135°-135°
%    Side:  L  FL   FF   FF   FR    R  RL   RR

FALLBACK_DIST = 3.0;  % [m] assume no wall when sonar returns NaN

% ---- Select sonars based on tracked wall side ------------------------
if strcmpi(config.side, 'left')
    side_sonar  = safeRead(sensors.sonar(1));  % sonar0: directly left (+90°)
    flank_sonar = safeRead(sensors.sonar(2));  % sonar1: front-left  (+54°)
    sign_w      = +1;   % +w = CCW = turn left = steer away from left wall
else
    side_sonar  = safeRead(sensors.sonar(6));  % sonar5: directly right (-90°)
    flank_sonar = safeRead(sensors.sonar(5));  % sonar4: front-right  (-54°)
    sign_w      = -1;   % -w = CW  = turn right = steer away from right wall
end

% ---- Determine front clearance from LiDAR (preferred) or front sonars
if isfinite(sensors.laser_min_front)
    front_dist = sensors.laser_min_front;
else
    % Fallback: average of the two forward-facing sonars
    front_dist = mean([safeRead(sensors.sonar(3)), safeRead(sensors.sonar(4))]);
end

% ======================================================================
% FRONT OBSTACLE – turn away from wall to negotiate corner
% ======================================================================
if front_dist < config.front_clear
    % Severity of the obstruction (1 = just reached threshold, → 0 = clear)
    severity = 1 - front_dist / config.front_clear;

    % Turn away from the wall with a speed proportional to how blocked we are
    w = -sign_w * severity * config.max_w;
    v = config.max_v * (1 - severity) * 0.3;  % Creep while turning

% ======================================================================
% NORMAL WALL FOLLOWING – PID on lateral distance error
% ======================================================================
else
    % CORREGIDO: Error = Lectura real - Distancia deseada
    dist_error = side_sonar - config.desired_dist;
    % Derivative-like correction: if flank sonar sees wall approaching,
    % pre-emptively steer out (enhances corner exit smoothness)
    flank_error  = flank_sonar - config.desired_dist;
    blend_error  = 0.7 * dist_error + 0.3 * flank_error;

    w_pid = pid_wall.compute(blend_error, config.dt);
    w     = sign_w * w_pid;
    w     = max(-config.max_w, min(config.max_w, w));

    % Reduce forward speed when the distance error is large
    error_ratio = abs(dist_error) / max(config.desired_dist, 0.1);
    v_scale     = max(0.3, 1 - 0.6 * min(1, error_ratio));
    v           = config.max_v * v_scale;
end

% ---- Final clipping --------------------------------------------------
v = max(0, min(config.max_v, v));
w = max(-config.max_w, min(config.max_w, w));

end

% ======================================================================
% LOCAL HELPER
% ======================================================================
function d = safeRead(raw)
    FALLBACK = 3.0;
    if isnan(raw) || raw <= 0
        d = FALLBACK;
    else
        d = raw;
    end
end
