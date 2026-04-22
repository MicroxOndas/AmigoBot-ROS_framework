classdef MissionControl < handle
%MISSIONCONTROL High-level autonomous navigation API.
%
%  Orchestrates PID behaviors (go-to-point, wall following, obstacle
%  avoidance) and an optional real-time dashboard.  Designed to be the
%  single entry point for all robot motion commands.
%
%  ────────────────────────────────────────────────────────────────────
%  QUICK START
%  ────────────────────────────────────────────────────────────────────
%    mc = MissionControl(ros_handles);
%
%    mc.gotoPoint([2.0, 1.5]);             % go to (x,y)
%    mc.gotoPoint([2.0, 1.5, pi/2]);      % go to (x,y) then face north
%
%    mc.addWaypoint([1.0, 0.0]);
%    mc.addWaypoint([1.0, 2.0]);
%    mc.addWaypoint([0.0, 0.0, 0]);       % return home, face east
%    mc.runMission();
%
%    mc.followWall('left',  0.4);         % follow left wall indefinitely
%    mc.followWall('right', 0.4, 'Duration', 20);  % 20 s only
%
%    mc.stop();
%    mc.setVelocity(0.2, 0.0);            % manual override
%
%  ────────────────────────────────────────────────────────────────────
%  CONSTRUCTOR
%  ────────────────────────────────────────────────────────────────────
%    ros_handles  struct with fields:
%      .pub     rospublisher for /robot0/cmd_vel
%      .msg     pre-allocated Twist rosmessage
%      .odom    rossubscriber for /robot0/odom
%      .sonars  1×8 cell of rossubscribers {sonar0…sonar7}
%      .laser   rossubscriber for /robot0/laser_1
%
%    Optional name-value pairs:
%      'Rate',          10      Loop rate [Hz]
%      'ShowDashboard', true    Open real-time dashboard window
%
%  ────────────────────────────────────────────────────────────────────
%  GAIN TUNING
%  ────────────────────────────────────────────────────────────────────
%    mc.setGains('gotoPoint_angle', [Kp Ki Kd])
%    mc.setGains('gotoPoint_dist',  [Kp Ki Kd])
%    mc.setGains('wallFollow',      [Kp Ki Kd])
%    mc.Config     % inspect / modify full config struct directly

    % ==================================================================
    properties (Access = public)
        Dashboard   % RobotDashboard handle ([] if ShowDashboard=false)
        Config      % Full parameter struct (edit for fine-tuning)
    end

    properties (Access = private)
        ros_          % ROS handles struct
        rate_         % robotics.Rate object

        pid_angle_    % PIDController – heading (goto point)
        pid_dist_     % PIDController – distance (goto point)
        pid_wall_     % PIDController – wall distance (wall follow)

        waypoints_    % Nx3 matrix  [x, y, theta_or_NaN]
        behavior_     % Current behavior string (displayed in dashboard)

        mission_log_  % Data log dedicated to post-exec reports
    end

    % ==================================================================
    methods (Access = public)

        % --------------------------------------------------------------
        function obj = MissionControl(ros_handles, varargin)
            p = inputParser();
            addRequired(p,  'ros_handles');
            addParameter(p, 'Rate',          10,   @isnumeric);
            addParameter(p, 'ShowDashboard', true, @islogical);
            parse(p, ros_handles, varargin{:});

            obj.ros_       = ros_handles;
            obj.rate_      = robotics.Rate(p.Results.Rate);
            obj.waypoints_ = zeros(0, 3);
            obj.behavior_  = 'IDLE';

            dt = 1 / p.Results.Rate;
            obj.Config = MissionControl.defaultConfig_(dt);

            % ---- Instantiate PID controllers -------------------------
            c = obj.Config;
            obj.pid_angle_ = PIDController( ...
                c.gotoPoint.pid_angle(1), ...
                c.gotoPoint.pid_angle(2), ...
                c.gotoPoint.pid_angle(3), ...
                'MaxOutput',    c.gotoPoint.max_w, ...
                'MinOutput',   -c.gotoPoint.max_w, ...
                'MaxIntegral',  2.0);

            obj.pid_dist_  = PIDController( ...
                c.gotoPoint.pid_dist(1), ...
                c.gotoPoint.pid_dist(2), ...
                c.gotoPoint.pid_dist(3), ...
                'MaxOutput',   c.gotoPoint.max_v, ...
                'MinOutput',   0.0, ...
                'MaxIntegral', 2.0);

            obj.pid_wall_  = PIDController( ...
                c.wallFollow.pid(1), ...
                c.wallFollow.pid(2), ...
                c.wallFollow.pid(3), ...
                'MaxOutput',    c.wallFollow.max_w, ...
                'MinOutput',   -c.wallFollow.max_w, ...
                'MaxIntegral',  1.5);

            % ---- Dashboard -------------------------------------------
            if p.Results.ShowDashboard
                obj.Dashboard = RobotDashboard('MaxHistory', 800);
            else
                obj.Dashboard = [];
            end

            fprintf('[MissionControl] Ready. Rate=%d Hz\n', p.Results.Rate);
            obj.printHelp_();
        end

        % ==============================================================
        % RELATIVE DRIVE (LOCAL COORDINATES)
        % ==============================================================
        function drive(obj, target_local, varargin)
        %DRIVE Move relative to the robot's current pose.
        %
        %  USAGE
        %    mc.drive([x_fwd, y_left])
        %    mc.drive([x_fwd, y_left, theta_local])
        %    mc.drive(..., 'AvoidObstacles', false)
        %    mc.drive(..., 'Mode', 'continuous')   % Arco suave (Por defecto)
        %    mc.drive(..., 'Mode', 'point-shoot')  % Parar y girar
        
            % 1. Parsear explícitamente para documentar y validar las opciones
            p = inputParser();
            addRequired(p,  'target_local');
            addParameter(p, 'AvoidObstacles', true, @islogical);
            
            validModes = {'continuous', 'point-shoot'};
            addParameter(p, 'Mode', 'continuous', @(x) any(validatestring(x, validModes)));
            
            parse(p, target_local, varargin{:});
            
            % 2. Obtener la pose actual del robot
            pose = readOdometry(obj.ros_.odom);
            
            % 3. Extraer coordenadas locales (x = adelante, y = izquierda)
            xl = target_local(1);
            yl = target_local(2);
            
            % 4. Aplicar Matriz de Transformación (Local -> Global)
            xg = pose.x + xl * cos(pose.theta) - yl * sin(pose.theta);
            yg = pose.y + xl * sin(pose.theta) + yl * cos(pose.theta);
            
            target_global = [xg, yg];
            
            % 5. Transformar ángulo local (si existe)
            if numel(target_local) >= 3
                thl = target_local(3);
                target_global(3) = angleDiff(pose.theta + thl, 0);
            end
            
            fprintf('[MissionControl] DRIVE: Local [%.2f, %.2f] → Global [%.2f, %.2f] | Mode: %s\n', ...
                xl, yl, xg, yg, upper(p.Results.Mode));
            
            % 6. Delegar a gotoPoint pasando los parámetros validados
            obj.gotoPoint(target_global, ...
                'AvoidObstacles', p.Results.AvoidObstacles, ...
                'Mode',           p.Results.Mode);
        end

        % ==============================================================
        % GO-TO-POINT / DRIVE-TO-POINT HANDLER
        % ==============================================================
        function gotoPoint(obj, target, varargin)
        %GOTOPOINT Drive to a target position with obstacle avoidance.
        %
        %  USAGE
        %    mc.gotoPoint([x, y])
        %    mc.gotoPoint([x, y, theta], 'AvoidObstacles', false)
        %    mc.gotoPoint([x, y], 'Mode', 'continuous')   % Nuevo: Arco suave (Por defecto)
        %    mc.gotoPoint([x, y], 'Mode', 'point-shoot')  % Clásico: Parar y girar
        
            p = inputParser();
            addRequired(p,  'target');
            addParameter(p, 'AvoidObstacles', false, @islogical);
            % Definimos los modos válidos y establecemos 'continuous' como el estándar
            validModes = {'continuous', 'point-shoot'};
            addParameter(p, 'Mode', 'continuous', @(x) any(validatestring(x, validModes)));
            parse(p, target, varargin{:});
            
            mode = p.Results.Mode;
            
            obj.pid_angle_.reset();
            obj.pid_dist_.reset();
            
            if ~isempty(obj.Dashboard)
                obj.Dashboard.setTarget(target(1:2));
            end
            
            fprintf('[MissionControl] Navigating to (%.2f, %.2f) | Mode: %s\n', ...
                target(1), target(2), upper(mode));
                
            cfg  = obj.Config.gotoPoint;
            done = false;
            
            while ~done
                pose    = readOdometry(obj.ros_.odom);
                sensors = readSensors(obj.ros_.sonars, obj.ros_.laser);
                
                % 1. PRIORIDAD ALTA: Esquiva de obstáculos
                if p.Results.AvoidObstacles && ...
                   sensors.laser_min_front < obj.Config.obstacleAvoid.emergency_dist
                   
                    obj.behavior_ = 'OBSTACLE_AVOID';
                    [v, w, ~]     = obstacleAvoidance(sensors, obj.Config.obstacleAvoid);
                    
                    % Freeze PID integrators while avoiding
                    obj.pid_angle_.reset();
                    obj.pid_dist_.reset();
                    
                % 2. PRIORIDAD BAJA: Navegación hacia el punto
                else
                    if strcmp(mode, 'continuous')
                        obj.behavior_ = 'DRIVE_TO_POINT';
                        % Llama a tu nueva función pura
                        [v, w, done]  = driveToPoint(pose, target, ...
                                                  obj.pid_angle_, obj.pid_dist_, cfg);
                    else
                        obj.behavior_ = 'GOTO_POINT';
                        % Llama a la función clásica "apunta y dispara"
                        [v, w, done]  = gotoPoint(pose, target, ...
                                                  obj.pid_angle_, obj.pid_dist_, cfg);
                    end
                end

                % --- HACK PARA EL BUG DE GAZEBO (Fricción Estática) ---
                % Si el robot intenta ir en línea recta perfecta, inyectamos
                % una micro-rotación invisible para despertar al simulador.
                if v > 0.05 && abs(w) < 1e-6
                    w = 1e-5; 
                end
                
                sendVelocity(obj.ros_.pub, obj.ros_.msg, v, w);
                obj.tickDashboard_(pose, sensors, v, w);
                obj.updateLog_(pose, sensors, v, w);
                waitfor(obj.rate_);
            end
            
            obj.stop();
            fprintf('[MissionControl] Target reached.\n');
        end

        % ==============================================================
        % WAYPOINT MISSION
        % ==============================================================
        function addWaypoint(obj, wp)
        %ADDWAYPOINT Append a waypoint to the mission queue.
        %  wp = [x, y]         – navigate to position
        %  wp = [x, y, theta]  – navigate then align to theta

            if numel(wp) == 2
                wp = [wp(:)', NaN];
            else
                wp = wp(:)';
            end
            obj.waypoints_ = [obj.waypoints_; wp];
            fprintf('[MissionControl] Waypoint %d added: (%.2f, %.2f)\n', ...
                size(obj.waypoints_,1), wp(1), wp(2));
        end

        % --------------------------------------------------------------
        function clearWaypoints(obj)
        %CLEARWAYPOINTS Remove all waypoints from the mission queue.
            obj.waypoints_ = zeros(0, 3);
            fprintf('[MissionControl] Waypoint queue cleared.\n');
        end

        % --------------------------------------------------------------
        function runMission(obj, varargin)
        %RUNMISSION Execute all queued waypoints in order.
        %
        %  mc.runMission()
        %  mc.runMission('AvoidObstacles', true, 'PauseBetween', 0.5)

            p = inputParser();
            addParameter(p, 'AvoidObstacles', true,  @islogical);
            addParameter(p, 'PauseBetween',   0.5,   @isnumeric);
            parse(p, varargin{:});

            n = size(obj.waypoints_, 1);
            if n == 0
                warning('[MissionControl] No waypoints queued.');
                return;
            end

            fprintf('[MissionControl] Mission start: %d waypoints.\n', n);

            for i = 1:n
                wp = obj.waypoints_(i,:);
                if isnan(wp(3))
                    tgt = wp(1:2);
                else
                    tgt = wp;
                end
                fprintf('[MissionControl] Waypoint %d / %d\n', i, n);
                obj.gotoPoint(tgt, 'AvoidObstacles', p.Results.AvoidObstacles);
                pause(p.Results.PauseBetween);
            end

            fprintf('[MissionControl] Mission complete.\n');
            obj.behavior_ = 'IDLE';
        end

        % ==============================================================
        % WALL FOLLOWING
        % ==============================================================
        function followWall(obj, side, desired_dist, varargin)
        %FOLLOWWALL Follow a wall at a set lateral distance.
        %
        %  mc.followWall('left',  0.4)
        %  mc.followWall('right', 0.5, 'Duration', 30)
        %  mc.followWall('left',  0.4, 'UseApproach', true) % Salta la aproximación
        
            p = inputParser();
            addRequired(p,  'side');
            addRequired(p,  'desired_dist');
            addParameter(p, 'Duration', Inf, @isnumeric);
            % NUEVO: Parámetro para hacer el approach opcional
            addParameter(p, 'UseApproach', false, @islogical); 
            parse(p, side, desired_dist, varargin{:});
            
            obj.pid_wall_.reset();
            
            if ~isempty(obj.Dashboard)
                obj.Dashboard.setTarget([]);
            end
            
            cfg              = obj.Config.wallFollow;
            cfg.side         = side;
            cfg.desired_dist = desired_dist;
            fprintf('[MissionControl] followWall: %s side, d=%.2f m\n', side, desired_dist);
            
            % --- LOGICA OPCIONAL DE ESTADO INICIAL ---
            if p.Results.UseApproach
                current_state = 'APPROACH';
                fprintf('[MissionControl] State: APPROACHING closest wall...\n');
            else
                current_state = 'FOLLOW';
                fprintf('[MissionControl] State: FOLLOWING wall directly...\n');
            end
            
            t_start = tic;
            while toc(t_start) < p.Results.Duration
                pose    = readOdometry(obj.ros_.odom);
                sensors = readSensors(obj.ros_.sonars, obj.ros_.laser);
                
                switch current_state
                    case 'APPROACH'
                        obj.behavior_ = 'APPROACH';
                        
                        if ~isempty(sensors.laser_ranges)
                            min_dist = min(sensors.laser_ranges);
                        else
                            min_dist = min(sensors.sonar); 
                        end
                        
                        threshold = max(cfg.front_clear * 1.2, desired_dist * 1.5);
                        
                        if min_dist <= threshold
                            current_state = 'FOLLOW';
                            fprintf('[MissionControl] Wall engaged! Switching to FOLLOW state.\n');
                            continue; 
                        end
                        
                        [v, w] = approachWall(sensors, cfg);
                        
                    case 'FOLLOW'
                        obj.behavior_ = sprintf('WALL_%s', upper(side));
                        [v, w] = wallFollower(pose, sensors, obj.pid_wall_, cfg);
                end
                
                sendVelocity(obj.ros_.pub, obj.ros_.msg, v, w);
                obj.tickDashboard_(pose, sensors, v, w);
                obj.updateLog_(pose, sensors, v, w);
                waitfor(obj.rate_);
            end
            
            obj.stop();
            fprintf('[MissionControl] Wall following ended.\n');
        end
        % ==============================================================
        % MANUAL / UTILITY
        % ==============================================================
        function stop(obj)
        %STOP Send zero velocity command and enter IDLE state.
            sendVelocity(obj.ros_.pub, obj.ros_.msg, 0, 0);
            obj.behavior_ = 'IDLE';
        end

        % --------------------------------------------------------------
        function setVelocity(obj, v, w)
        %SETVELOCITY Send a direct velocity command (manual override).
        %  Bypasses all behaviors.  Use stop() to return to IDLE.
            sendVelocity(obj.ros_.pub, obj.ros_.msg, v, w);
        end

        % --------------------------------------------------------------
        function setGains(obj, name, gains)
        %SETGAINS Update PID gains for a named controller.
        %
        %  mc.setGains('gotoPoint_angle', [Kp Ki Kd])
        %  mc.setGains('gotoPoint_dist',  [Kp Ki Kd])
        %  mc.setGains('wallFollow',      [Kp Ki Kd])

            switch lower(strrep(name, '_', ''))
                case 'gotopointangle'
                    obj.pid_angle_.setGains(gains(1), gains(2), gains(3));
                    obj.Config.gotoPoint.pid_angle = gains;
                case 'gotopointdist'
                    obj.pid_dist_.setGains(gains(1), gains(2), gains(3));
                    obj.Config.gotoPoint.pid_dist = gains;
                case 'wallfollow'
                    obj.pid_wall_.setGains(gains(1), gains(2), gains(3));
                    obj.Config.wallFollow.pid = gains;
                otherwise
                    warning('[MissionControl] Unknown controller: %s', name);
            end
            fprintf('[MissionControl] Gains updated for: %s\n', name);
        end

        function log = getMissionLog(obj)
            % Devuelve los datos recopilados
            log = obj.mission_log_;
        end
        
        function resetMissionLog(obj)
            % Limpia y construye la grabadora garantizando todos los campos
            obj.mission_log_ = struct();
            
            % Variables de tiempo y trayectoria
            obj.mission_log_.t0 = [];
            obj.mission_log_.t  = [];
            obj.mission_log_.x  = [];
            obj.mission_log_.y  = [];
            obj.mission_log_.theta = [];
            
            % Variables de velocidad
            obj.mission_log_.v_cmd  = [];
            obj.mission_log_.w_cmd  = [];
            obj.mission_log_.v_real = [];
            obj.mission_log_.w_real = [];
            
            % Variables de entorno (LiDAR)
            obj.mission_log_.wall_points_x = [];
            obj.mission_log_.wall_points_y = [];
            
            % Guardar estado de los PIDs (si ya existen)
            if ~isempty(obj.pid_angle_)
                obj.mission_log_.pid_angle = obj.pid_angle_;
                obj.mission_log_.pid_dist  = obj.pid_dist_;
            end
            
            fprintf('[MissionControl] Mission Log reset and ready.\n');
        end


    end % public methods

    % ==================================================================
    methods (Access = private)

        % --------------------------------------------------------------
        function tickDashboard_(obj, pose, sensors, v_cmd, w_cmd)
        %TICKDASHBOARD_ Sends the latest data to the UI if it is active.
            
            if ~isempty(obj.Dashboard) && isvalid(obj.Dashboard)
                % Inyectamos los comandos (v_cmd, w_cmd) y las velocidades 
                % reales leídas de la odometría (pose.vx, pose.wz)
                obj.Dashboard.update(pose, ...
                                     sensors, ...
                                     v_cmd, ...
                                     w_cmd, ...
                                     pose.vx, ...
                                     pose.wz, ...
                                     obj.behavior_);
            end
        end
        % --------------------------------------------------------------
        function printHelp_(obj)
            fprintf('\n  ── MissionControl API ────────────────────────────────\n');
            fprintf('  mc.gotoPoint([x y])           Navigate to position\n');
            fprintf('  mc.gotoPoint([x y theta])     … + align to heading\n');
            fprintf('  mc.addWaypoint([x y])         Queue a waypoint\n');
            fprintf('  mc.runMission()               Execute waypoint queue\n');
            fprintf('  mc.followWall(side, dist)     Wall follow\n');
            fprintf('  mc.stop()                     Emergency stop\n');
            fprintf('  mc.setGains(name, [Kp Ki Kd]) Tune PID gains\n');
            fprintf('  mc.Config                     Edit all parameters\n');
            fprintf('  ────────────────────────────────────────────────────\n\n');
        end

        function updateLog_(obj, pose, sensors, v_cmd, w_cmd)
            if isempty(obj.mission_log_)
                obj.resetMissionLog();
                % Guardamos las ganancias actuales al iniciar la grabación
                obj.mission_log_.gains.angle = obj.pid_angle_.Kp; % Ejemplo si quieres guardar solo Kp
                obj.mission_log_.gains.dist = obj.pid_dist_.Kp;
                % O mejor, pasamos los objetos completos para el reporte
                obj.mission_log_.pid_angle = obj.pid_angle_;
                obj.mission_log_.pid_dist = obj.pid_dist_;
            end

            if isempty(obj.mission_log_.t)
                obj.mission_log_.t0 = pose.stamp;
            end
            
            t_rel = pose.stamp - obj.mission_log_.t0;
            
            % Registro estándar
            obj.mission_log_.t(end+1)      = t_rel;
            obj.mission_log_.x(end+1)      = pose.x;
            obj.mission_log_.y(end+1)      = pose.y;
            obj.mission_log_.theta(end+1)  = pose.theta;
            obj.mission_log_.v_cmd(end+1)  = v_cmd;
            obj.mission_log_.w_cmd(end+1)  = w_cmd;
            obj.mission_log_.v_real(end+1) = pose.vx;
            obj.mission_log_.w_real(end+1) = pose.wz;

            % 5. Registro de LiDAR (Paredes limpias)
            % Guardamos datos cada 10 iteraciones temporalmente
            if mod(numel(obj.mission_log_.t), 10) == 0 && ~isempty(sensors.laser_ranges)
                
                % REDUCCIÓN ESPACIAL (EL TRUCO CLAVE): 
                % Cogemos solo 1 de cada 10 rayos del LiDAR. Esto evita el efecto "nube"
                % y crea una línea de puntos separados, igual que en el dashboard.
                decimation = 10;
                ranges = sensors.laser_ranges(1:decimation:end);
                angles = sensors.laser_angles(1:decimation:end);
                
                % Filtramos el espacio vacío (solo guardamos si choca a menos de 3m)
                valid = ~isnan(ranges) & ranges < 3.0; 
                
                if any(valid)
                    world_angles = angles(valid) + pose.theta;
                    px = pose.x + ranges(valid) .* cos(world_angles);
                    py = pose.y + ranges(valid) .* sin(world_angles);
                    
                    obj.mission_log_.wall_points_x = [obj.mission_log_.wall_points_x; px(:)];
                    obj.mission_log_.wall_points_y = [obj.mission_log_.wall_points_y; py(:)];
                end
            end
        end

    end % private methods

    % ==================================================================
    methods (Static, Access = private)

        function cfg = defaultConfig_(dt)
        %DEFAULTCONFIG_ Return the default parameter struct.
        %  All physical limits are conservative to protect the robot.

            % ---- Go-to-point -----------------------------------------
            cfg.gotoPoint.pid_angle   = [2.0, 0.05, 0.30];  % [Kp Ki Kd]
            cfg.gotoPoint.pid_dist    = [0.5, 0.01, 0.10];
            cfg.gotoPoint.dt          = dt;
            cfg.gotoPoint.dist_tol    = 0.10;   % [m]   position tolerance
            cfg.gotoPoint.angle_tol   = 0.06;   % [rad] heading tolerance
            cfg.gotoPoint.max_v       = 0.70;   % [m/s]
            cfg.gotoPoint.max_w       = 0.70;   % [rad/s]
            cfg.gotoPoint.slow_dist   = 0.50;   % [m]   speed ramp-down onset

            % ---- Wall following -------------------------------------
            cfg.wallFollow.pid          = [1.5, 0.02, 0.40];
            cfg.wallFollow.dt           = dt;
            cfg.wallFollow.max_v        = 0.70;   % [m/s]
            cfg.wallFollow.max_w        = 1.00;   % [rad/s]
            cfg.wallFollow.front_clear  = 0.55;   % [m]   turn trigger
            cfg.wallFollow.side         = 'left'; % overridden at runtime
            cfg.wallFollow.desired_dist = 0.40;   % [m]   overridden at runtime

            % ---- Obstacle avoidance ---------------------------------
            cfg.obstacleAvoid.influence_radius = 1.20;  % [m]
            cfg.obstacleAvoid.emergency_dist   = 0.35;  % [m]
            cfg.obstacleAvoid.clear_dist       = 0.60;  % [m]
            cfg.obstacleAvoid.max_v            = 0.25;  % [m/s]
            cfg.obstacleAvoid.max_w            = 1.20;  % [rad/s]
        end

    end % static private methods
end
