function [v, w, done] = gotoPoint(pose, target, pid_angle, pid_dist, config)
%GOTOPOINT PID-based go-to-point behavior for a differential-drive robot.
%
%  Implements a two-phase controller:
%    Phase 1 – Navigate to the target (x,y) while correcting heading online.
%    Phase 2 – (Optional) Rotate to a desired final heading.
%
%  SIGNATURE
%    [v, w, done] = gotoPoint(pose, target, pid_angle, pid_dist, config)
%
%  INPUTS
%    pose      - Pose struct from readOdometry()
%                  Fields: .x  .y  .theta
%    target    - [x, y]        → navigate to position, no final angle
%                [x, y, theta] → navigate, then align to theta
%    pid_angle - PIDController instance for heading control
%    pid_dist  - PIDController instance for forward speed control
%    config    - Parameter struct (see MissionControl.defaultConfig_):
%                  .dt          [s]   Loop period
%                  .dist_tol    [m]   Position acceptance radius
%                  .angle_tol   [rad] Heading acceptance tolerance (Phase 2)
%                  .max_v       [m/s] Maximum forward speed
%                  .max_w       [rad/s] Maximum angular speed
%                  .slow_dist   [m]   Distance to begin speed ramp-down
%
%  OUTPUTS
%    v    - Forward velocity command  [m/s]
%    w    - Angular velocity command  [rad/s]
%    done - true when the goal (position + optional heading) is reached

has_final_angle = (numel(target) >= 3) && ~isnan(target(3));

dx   = target(1) - pose.x;
dy   = target(2) - pose.y;
dist = hypot(dx, dy);

% ======================================================================
% PHASE 1 – Drive to position
% ======================================================================
if dist > config.dist_tol

    % Desired heading toward target
    target_heading = atan2(dy, dx);
    angle_error    = angleDiff(target_heading, pose.theta);

    % Angular correction
    w = pid_angle.compute(angle_error, config.dt);
    w = max(-config.max_w, min(config.max_w, w));

    % Forward speed:
    %   - Reduce when significantly misaligned (cos penalty)
    %   - Ramp down linearly inside slow_dist
    alignment   = max(0, cos(angle_error));
    dist_factor = min(1, dist / max(config.slow_dist, 1e-3));
    v_desired   = pid_dist.compute(dist, config.dt);
    v           = v_desired * alignment * dist_factor;
    v           = max(0, min(config.max_v, v));  % Forward only

    done = false;

% ======================================================================
% PHASE 2 – Align to final heading (if requested)
% ======================================================================
elseif has_final_angle
    pid_dist.reset();  % Position achieved; freeze distance integrator

    angle_error = angleDiff(target(3), pose.theta);
    

    if abs(angle_error) > config.angle_tol
        w    = pid_angle.compute(angle_error, config.dt);
        w    = max(-config.max_w, min(config.max_w, w));
        v    = 0;
        done = false;
    else
        v = 0; w = 0;
        done = true;
    end

% ======================================================================
% GOAL REACHED
% ======================================================================
else
    v = 0; w = 0;
    done = true;
end
end
