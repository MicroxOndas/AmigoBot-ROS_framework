function [v, w, done] = driveToPoint(pose, target, pid_angle, pid_dist, config)
%DRIVETOPOINT Pure decoupled PID continuous navigation.
%
%  v and w are calculated completely independently. 
%  Warning: May produce wide spirals if target is behind the robot.

    has_final_angle = (numel(target) >= 3) && ~isnan(target(3));
    dx   = target(1) - pose.x;
    dy   = target(2) - pose.y;
    dist = hypot(dx, dy);

    if dist > config.dist_tol
        % --- PID DE ÁNGULO PURO ---
        target_heading = atan2(dy, dx);
        angle_error    = angleDiff(target_heading, pose.theta);
        w = pid_angle.compute(angle_error, config.dt);
        
        % --- PID DE DISTANCIA PURO ---
        v = pid_dist.compute(dist, config.dt);
        
        % Saturación individual de actuadores
        w = max(-config.max_w, min(config.max_w, w));
        v = max(0, min(config.max_v, v)); 
        
        done = false;

    elseif has_final_angle
        pid_dist.reset(); 
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
    else
        v = 0; w = 0;
        done = true;
    end
end