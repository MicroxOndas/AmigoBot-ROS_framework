function [v, w] = approachWall(sensors, config)

    valid_mask = ~isnan(sensors.laser_ranges);
    ranges = sensors.laser_ranges(valid_mask);
    angles = sensors.laser_angles(valid_mask);
    
    if isempty(ranges)
        v = 0.0;
        w = config.max_w * 0.8;
        return;
    end
    
    % Punto más cercano
    [min_dist, idx] = min(ranges);
    target_angle = angles(idx);

    % Definir ángulo lateral deseado
    if strcmpi(config.side, 'right')
        desired_angle = -pi/2;
    else
        desired_angle = pi/2;
    end

    % Mezcla entre "ir de frente" y "alinearse lateralmente"
    % alpha = 0 → ir al frente
    % alpha = 1 → alinearse con la pared
    d0 = config.desired_dist * 2;
    alpha = min(1, max(0, (d0 - min_dist) / d0));

    % Ángulo objetivo interpolado
    blended_angle = (1 - alpha) * target_angle + alpha * desired_angle;

    % ==============================
    % CONTROL ANGULAR
    % ==============================
    Kp_w = 1.5;
    w = Kp_w * blended_angle;
    w = max(-config.max_w, min(config.max_w, w));

    % ==============================
    % CONTROL LINEAL
    % ==============================
    v_scale = 1 - alpha; % cuanto más cerca, menos avanza
    turn_penalty = max(0, 1 - abs(blended_angle)/(pi/3));

    v = config.max_v * v_scale * turn_penalty;
    v = max(0, min(config.max_v, v));
end