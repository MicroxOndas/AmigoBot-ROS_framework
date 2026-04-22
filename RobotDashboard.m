classdef RobotDashboard < handle
%ROBOTDASHBOARD Real-time three-panel visualization dashboard.
%
%  Panels:
%    Left        : Trajectory map (with LiDAR overlay and target marker)
%    Top-right   : Sensor radar   (sonar beams + LiDAR in robot frame)
%    Bottom-right: Combined Velocities (Commanded vs Real)
%    Bottom bar  : Status text    (behavior · pose · velocities)
%
%  CONSTRUCTOR
%    db = RobotDashboard()
%    db = RobotDashboard('MaxHistory', 800)
%
%  METHODS
%    db.update(pose, sensors, v_cmd, w_cmd, v_real, w_real, behavior)
%    db.setTarget(target)                      
%    db.close()                                

    % ------------------------------------------------------------------
    properties (Constant, Access = private)
        % Sonar mounting angles [rad]
        SONAR_ANGLES    = deg2rad([90, 54, 18, -18, -54, -90, 135, -135]);
        SONAR_MAX_RANGE = 3.0;   % [m] display clamp for radar beams
        RADAR_RADIUS    = 3.5;   % [m] radar plot axis half-width
        ROBOT_RADIUS    = 0.22;  % [m] robot body drawn on radar
        VEL_WINDOW      = 30;    % [s] sliding time-window for velocity plots
        
        % Color Palette
        COLOR_BG        = [0.08 0.09 0.11];
        COLOR_PANEL     = [0.12 0.13 0.16];
        COLOR_GRID      = [0.25 0.27 0.33];
        COLOR_TEXT      = [0.85 0.88 0.92];
        COLOR_VLIN      = [0.15 0.85 0.55]; % Green/Cyan (Real V)
        COLOR_WANG      = [1.00 0.55 0.15]; % Orange (Real W)
        COLOR_TRAJ      = [0.20 0.60 1.00]; % Blue
    end

    % ------------------------------------------------------------------
    properties (Access = private)
        fig_
        % Axes
        ax_map_
        ax_radar_
        ax_vel_
        % Map plot handles
        h_lidar_
        h_traj_
        h_target_
        h_robot_
        h_heading_
        % Radar handles
        h_sonar_beams_
        h_sonar_dots_
        h_lidar_radar_
        % Velocity plot handles
        h_vlin_cmd_line_
        h_vlin_real_line_
        h_wang_cmd_line_
        h_wang_real_line_
        % Status
        h_status_
        
        % Pre-allocated Data history (Circular Buffer for Performance)
        hist_x_
        hist_y_
        hist_t_
        hist_v_cmd_
        hist_w_cmd_
        hist_v_real_
        hist_w_real_
        hist_count_   % Total updates received
        max_hist_
        
        t0_           % Start time [s]
        first_update_ % true until first update() call
        last_target_  % cached target for redraw after setTarget()
    end

    % ==================================================================
    methods (Access = public)
        % --------------------------------------------------------------
        function obj = RobotDashboard(varargin)
            p = inputParser();
            addParameter(p, 'MaxHistory', 800, @isnumeric);
            parse(p, varargin{:});
            
            obj.max_hist_    = p.Results.MaxHistory;
            
            % Pre-allocate arrays
            obj.hist_x_      = zeros(1, obj.max_hist_);
            obj.hist_y_      = zeros(1, obj.max_hist_);
            obj.hist_t_      = zeros(1, obj.max_hist_);
            obj.hist_v_cmd_  = zeros(1, obj.max_hist_);
            obj.hist_w_cmd_  = zeros(1, obj.max_hist_);
            obj.hist_v_real_ = zeros(1, obj.max_hist_);
            obj.hist_w_real_ = zeros(1, obj.max_hist_);
            obj.hist_count_  = 0;
            
            obj.first_update_ = true;
            obj.last_target_  = [];
            obj.buildLayout_();
        end

        % --------------------------------------------------------------
        function update(obj, pose, sensors, v_cmd, w_cmd, v_real, w_real, behavior)
            if ~isvalid(obj.fig_)
                return;
            end

            % ---- Initialise time reference -------------
            if obj.first_update_
                obj.t0_ = pose.stamp;
                obj.first_update_ = false;
            end
            t = pose.stamp - obj.t0_;

            % ---- Update Circular Buffer ----------------
            obj.hist_count_ = obj.hist_count_ + 1;
            idx = mod(obj.hist_count_ - 1, obj.max_hist_) + 1;
            
            obj.hist_x_(idx)      = pose.x;
            obj.hist_y_(idx)      = pose.y;
            obj.hist_t_(idx)      = t;
            obj.hist_v_cmd_(idx)  = v_cmd;
            obj.hist_w_cmd_(idx)  = w_cmd;
            obj.hist_v_real_(idx) = v_real;
            obj.hist_w_real_(idx) = w_real;

            % ---- Refresh each panel --------------------
            obj.drawMap_(pose, sensors);
            obj.drawRadar_(sensors);
            obj.drawVelocities_();
            obj.drawStatus_(pose, v_cmd, w_cmd, v_real, w_real, behavior);
            
            drawnow limitrate;
        end

        % --------------------------------------------------------------
        function setTarget(obj, target)
            obj.last_target_ = target;
            if isempty(target)
                set(obj.h_target_, 'Visible', 'off');
            else
                set(obj.h_target_, 'XData', target(1), 'YData', target(2), 'Visible', 'on');
            end
        end

        % --------------------------------------------------------------
        function close(obj)
            if isvalid(obj.fig_)
                close(obj.fig_);
            end
        end
    end

    % ==================================================================
    methods (Access = private)
        
        % --------------------------------------------------------------
        function [x_out, y_out, t_out, vc_out, wc_out, vr_out, wr_out] = getHistory_(obj)
            if obj.hist_count_ == 0
                x_out = []; y_out = []; t_out = []; 
                vc_out = []; wc_out = []; vr_out = []; wr_out = [];
                return;
            end
            
            n = min(obj.hist_count_, obj.max_hist_);
            if obj.hist_count_ <= obj.max_hist_
                idx_range = 1:n;
            else
                idx = mod(obj.hist_count_ - 1, obj.max_hist_) + 1;
                idx_range = [idx+1:obj.max_hist_, 1:idx];
            end
            
            x_out  = obj.hist_x_(idx_range);
            y_out  = obj.hist_y_(idx_range);
            t_out  = obj.hist_t_(idx_range);
            vc_out = obj.hist_v_cmd_(idx_range);
            wc_out = obj.hist_w_cmd_(idx_range);
            vr_out = obj.hist_v_real_(idx_range);
            wr_out = obj.hist_w_real_(idx_range);
        end

        % --------------------------------------------------------------
        function buildLayout_(obj)
            obj.fig_ = figure( ...
                'Name',        'Robot Control Dashboard', ...
                'NumberTitle', 'off', ...
                'Color',       obj.COLOR_BG, ...
                'Position',    [50 50 1400 800], ...
                'MenuBar',     'none', ...
                'ToolBar',     'figure', ...
                'Resize',      'on');

            axStyle = { ...
                'Color',      obj.COLOR_PANEL, ...
                'XColor',     obj.COLOR_GRID * 1.5, ...
                'YColor',     obj.COLOR_GRID * 1.5, ...
                'GridColor',  obj.COLOR_GRID, ...
                'GridAlpha',  0.8, ...
                'FontSize',   10, ...
                'FontName',   'Helvetica'};

            left_w = 0.48;  right_w = 0.42;
            gap = 0.04;     margin_l = 0.03;
            
            % ---- Panel 1: Trajectory map ------------------
            obj.ax_map_ = axes(obj.fig_, axStyle{:}, 'Position', [margin_l 0.12 left_w 0.82]);
            grid(obj.ax_map_, 'on'); hold(obj.ax_map_, 'on'); axis(obj.ax_map_, 'equal');
            title(obj.ax_map_, 'Trajectory & Environment Map', 'Color', obj.COLOR_TEXT, 'FontSize', 12);
            xlabel(obj.ax_map_, 'X [m]'); ylabel(obj.ax_map_, 'Y [m]');
            
            obj.h_lidar_ = scatter(obj.ax_map_, [], [], 6, [0.3 0.8 0.5], 'filled', 'MarkerFaceAlpha', 0.6);
            obj.h_traj_ = plot(obj.ax_map_, NaN, NaN, '-', 'Color', obj.COLOR_TRAJ, 'LineWidth', 2.0);
            obj.h_target_ = plot(obj.ax_map_, NaN, NaN, 'p', 'Color', [1.0 0.3 0.3], 'MarkerSize', 16, 'MarkerFaceColor', [1.0 0.3 0.3], 'Visible', 'off');
            obj.h_robot_ = plot(obj.ax_map_, 0, 0, 'o', 'MarkerFaceColor', [0.9 0.8 0.1], 'MarkerEdgeColor', [0.3 0.3 0.3], 'MarkerSize', 10);
            obj.h_heading_ = quiver(obj.ax_map_, 0, 0, 0.4, 0, 0, 'Color', [1.0 1.0 1.0], 'LineWidth', 2.5, 'MaxHeadSize', 0.8);

            % ---- Panel 2: Sensor radar -------------------
           obj.ax_radar_ = axes(obj.fig_, 'Color', obj.COLOR_PANEL, ...
                'Position', [0.50, 0.52, 0.35, 0.42]);
            axis(obj.ax_radar_, 'equal');
            axis(obj.ax_radar_, [-obj.RADAR_RADIUS obj.RADAR_RADIUS -obj.RADAR_RADIUS obj.RADAR_RADIUS]);
            hold(obj.ax_radar_, 'on');
            title(obj.ax_radar_, 'Local Sensor Radar', 'Color', obj.COLOR_TEXT, 'FontSize', 12);
            obj.ax_radar_.XAxis.Visible = 'off'; obj.ax_radar_.YAxis.Visible = 'off';
            obj.buildRadarBackground_();
            
            obj.h_lidar_radar_ = scatter(obj.ax_radar_, [], [], 8, [0.3 0.8 0.5], 'filled', 'MarkerFaceAlpha', 0.7);
            
            cmap = lines(8);
            obj.h_sonar_beams_ = gobjects(1, 8);
            obj.h_sonar_dots_  = gobjects(1, 8);
            for i = 1:8
                obj.h_sonar_beams_(i) = plot(obj.ax_radar_, NaN, NaN, '-', 'LineWidth', 2.5);
                obj.h_sonar_dots_(i)  = plot(obj.ax_radar_, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerEdgeColor', 'none');
            end
            
            th_c = linspace(0, 2*pi, 60);
            plot(obj.ax_radar_, obj.ROBOT_RADIUS * cos(th_c), obj.ROBOT_RADIUS * sin(th_c), '-', 'Color', [0.9 0.8 0.1], 'LineWidth', 2.5);
            plot(obj.ax_radar_, [0, 0], [0, obj.ROBOT_RADIUS*1.5], '-', 'Color', [1.0 1.0 1.0], 'LineWidth', 2.5);

            % Panel 2.1 Logo y título
            % Creamos unos ejes pequeños en la esquina superior derecha
            ax_logo = axes(obj.fig_, 'Position', [0.80, 0.70, 0.18, 0.18], 'Units', 'normalized'); 

            % Esto es lo que evita que el texto "baile" al maximizar:
            hold(ax_logo, 'on');
            axis(ax_logo, 'equal'); % Mantiene la relación de aspecto 1:1
            
            try
                % Intentamos cargar la imagen
                ruta_logo = fullfile('resources', 'roblox_face.png'); 
                [img, ~, alpha] = imread(ruta_logo);
                h_img = image(ax_logo, img);
                
                set(ax_logo, 'YDir', 'reverse'); % O simplemente usa el comando: axis(ax_logo, 'ij');

                % Aplicamos transparencia si el PNG la tiene
                if ~isempty(alpha)
                    set(h_img, 'AlphaData', alpha);
                end
            catch
                % Si no encuentra el archivo, pone un placeholder visual
                patch(ax_logo, [0 1 1 0], [0 0 1 1], obj.COLOR_GRID, 'EdgeColor', obj.COLOR_TEXT);
                text(ax_logo, ancho/2, alto * 1.3, {'Robo-chito', 'Industries'}, ...
                    'Color', obj.COLOR_TEXT, ...
                    'FontSize', 16, ...
                    'FontWeight', 'bold', ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'top', ...
                    'FontName', 'Impact');
            end
            
            % Hacemos que los ejes del logo sean invisibles
            axis(ax_logo, 'off', 'image'); 
            
            % Añadimos el texto justo debajo del logo
            text(ax_logo, 0.5, -0.25, {'Robo-chito', 'Industries'}, ...
                'Units', 'normalized', ...
                'Color', obj.COLOR_TEXT, ...
                'FontSize', 16, ...
                'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', ...
                'FontName', 'Verdana');

            % ---- Panel 3: Combined Velocities -------------------
            obj.ax_vel_ = axes(obj.fig_, axStyle{:}, 'Position', [margin_l + left_w + gap, 0.12, right_w, 0.32]);
            grid(obj.ax_vel_, 'on'); hold(obj.ax_vel_, 'on');
            title(obj.ax_vel_, 'Velocities: Commanded vs Real', 'Color', obj.COLOR_TEXT, 'FontSize', 12);
            xlabel(obj.ax_vel_, 'Time [s]');
            
            % Linear Velocities (Left Y-Axis)
            yyaxis(obj.ax_vel_, 'left');
            obj.ax_vel_.YColor = obj.COLOR_VLIN;
            ylabel(obj.ax_vel_, 'Linear Velocity (v) [m/s]');
            
            obj.h_vlin_cmd_line_  = plot(obj.ax_vel_, NaN, NaN, '--', 'Color', obj.COLOR_VLIN * 0.7, 'LineWidth', 1.5, 'DisplayName', 'v (cmd)');
            obj.h_vlin_real_line_ = plot(obj.ax_vel_, NaN, NaN, '-',  'Color', obj.COLOR_VLIN,       'LineWidth', 2.0, 'DisplayName', 'v (real)');
            ylim(obj.ax_vel_, [-0.1, 0.6]);
            
            % Angular Velocities (Right Y-Axis)
            yyaxis(obj.ax_vel_, 'right');
            obj.ax_vel_.YColor = obj.COLOR_WANG;
            ylabel(obj.ax_vel_, 'Angular Velocity (\omega) [rad/s]');
            
            obj.h_wang_cmd_line_  = plot(obj.ax_vel_, NaN, NaN, '--', 'Color', obj.COLOR_WANG * 0.7, 'LineWidth', 1.5, 'DisplayName', '\omega (cmd)');
            obj.h_wang_real_line_ = plot(obj.ax_vel_, NaN, NaN, '-',  'Color', obj.COLOR_WANG,       'LineWidth', 2.0, 'DisplayName', '\omega (real)');
            ylim(obj.ax_vel_, [-1.5, 1.5]);

            % Legend setup
            leg = legend(obj.ax_vel_, 'Location', 'northwest', 'Orientation', 'horizontal');
            leg.Color = 'none'; leg.TextColor = obj.COLOR_TEXT; leg.EdgeColor = 'none'; leg.FontSize = 8;

            % ---- Status bar ---------------------------
            ax_status = axes(obj.fig_, 'Position', [0 0 1 0.04], 'Color', [0.05 0.05 0.06], 'XColor', 'none', 'YColor', 'none');
            obj.h_status_ = text(ax_status, 0.5, 0.5, 'Initializing...', ...
                'Color', obj.COLOR_TEXT, 'FontSize', 11, 'FontName', 'Monospaced', ...
                'Units', 'normalized', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
            
            drawnow;
        end

        % --------------------------------------------------------------
        function buildRadarBackground_(obj)
            ax = obj.ax_radar_;
            R = obj.RADAR_RADIUS;
            th = linspace(0, 2*pi, 100);
            col = obj.COLOR_GRID;
            
            for r = 1:floor(R)
                plot(ax, r*sin(th), r*cos(th), '-', 'Color', col, 'LineWidth', 0.8);
                text(ax, 0.1, r, sprintf('%dm', r), 'Color', col*1.5, 'FontSize', 8, 'FontName', 'Helvetica');
            end
            for ang_deg = 0:45:315
                a_rad = deg2rad(ang_deg);
                plot(ax, [0, R*sin(a_rad)], [0, R*cos(a_rad)], ':', 'Color', col, 'LineWidth', 0.8);
            end
        end

        % --------------------------------------------------------------
        function drawMap_(obj, pose, sensors)
            [x_hist, y_hist, ~, ~, ~, ~, ~] = obj.getHistory_();
            
            set(obj.h_traj_, 'XData', x_hist, 'YData', y_hist);
            set(obj.h_robot_, 'XData', pose.x, 'YData', pose.y);
            set(obj.h_heading_, 'XData', pose.x, 'YData', pose.y, 'UData', 0.40 * cos(pose.theta), 'VData', 0.40 * sin(pose.theta));
            
            if ~isempty(sensors.laser_ranges) && ~isempty(sensors.laser_angles)
                n = min(length(sensors.laser_ranges), length(sensors.laser_angles));
                r = sensors.laser_ranges(1:n); a = sensors.laser_angles(1:n);
                r = r(:); a = a(:);
                valid = ~isnan(r);
                ga = a(valid) + pose.theta; r_valid = r(valid);
                set(obj.h_lidar_, 'XData', pose.x + r_valid .* cos(ga), 'YData', pose.y + r_valid .* sin(ga));
            end
            
            if numel(x_hist) > 1
                mg = 2.0; 
                x_min = min(x_hist) - mg; x_max = max(x_hist) + mg;
                y_min = min(y_hist) - mg; y_max = max(y_hist) + mg;
                if (x_max - x_min) < 4, x_mid = (x_max+x_min)/2; x_min = x_mid-2; x_max = x_mid+2; end
                if (y_max - y_min) < 4, y_mid = (y_max+y_min)/2; y_min = y_mid-2; y_max = y_mid+2; end
                axis(obj.ax_map_, [x_min, x_max, y_min, y_max]);
            end
        end

        % --------------------------------------------------------------
        function drawRadar_(obj, sensors)
            % ---- 1. LiDAR (Nube de puntos verde) ----
            x_lidar = []; y_lidar = [];
            if ~isempty(sensors.laser_ranges) && ~isempty(sensors.laser_angles)
                n = min(length(sensors.laser_ranges), length(sensors.laser_angles));
                r = sensors.laser_ranges(1:n); a = sensors.laser_angles(1:n);
                r = r(:); a = a(:);
                mask = ~isnan(r); 
                r_l = min(r(mask), obj.SONAR_MAX_RANGE); a_l = a(mask);
                % Rotación para que 0º sea hacia arriba
                x_lidar = -r_l .* sin(a_l); y_lidar =  r_l .* cos(a_l);
            end
            set(obj.h_lidar_radar_, 'XData', x_lidar, 'YData', y_lidar);
            
            % ---- 2. Sonares (Haces de luz/Líneas) ----
            % Iteramos por los 8 sonares (incluyendo los traseros 7 y 8)
            num_sonars = min(8, length(sensors.sonar));
            for i = 1:num_sonars
                d = sensors.sonar(i);
                angle = obj.SONAR_ANGLES(i);
                
                % Si el sensor devuelve NaN, lo dibujamos al máximo rango (faint)
                is_max = isnan(d) || d >= obj.SONAR_MAX_RANGE;
                d_draw = min(d, obj.SONAR_MAX_RANGE);
                if isnan(d_draw), d_draw = obj.SONAR_MAX_RANGE; end
                
                % Cálculo de coordenadas del haz (Línea desde el robot al obstáculo)
                rx = -d_draw * sin(angle); 
                ry =  d_draw * cos(angle);
                ex = -obj.ROBOT_RADIUS * sin(angle); 
                ey =  obj.ROBOT_RADIUS * cos(angle);
                
                % Actualizar la línea del haz y el punto final
                set(obj.h_sonar_beams_(i), 'XData', [ex, rx], 'YData', [ey, ry]);
                set(obj.h_sonar_dots_(i), 'XData', rx, 'YData', ry);
                
                % Lógica de color (Verde -> Rojo según cercanía)
                t_dist = d_draw / obj.SONAR_MAX_RANGE;
                beam_c = [min(1, 2*(1-t_dist)), min(1, 2*t_dist), 0.2];
                
                % Si el dato es real (no NaN), le damos brillo; si no, queda tenue
                alpha = 0.2 + 0.8 * double(~is_max);
                
                set(obj.h_sonar_beams_(i), 'Color', [beam_c, alpha]);
                set(obj.h_sonar_dots_(i), 'MarkerFaceColor', beam_c, 'Color', beam_c);
            end
        end

        % --------------------------------------------------------------
        function drawVelocities_(obj)
            [~, ~, t_hist, vc_hist, wc_hist, vr_hist, wr_hist] = obj.getHistory_();
            if numel(t_hist) < 2
                return;
            end
            
            t_now = t_hist(end);
            
            % Update all 4 lines
            set(obj.h_vlin_cmd_line_,  'XData', t_hist, 'YData', vc_hist);
            set(obj.h_vlin_real_line_, 'XData', t_hist, 'YData', vr_hist);
            set(obj.h_wang_cmd_line_,  'XData', t_hist, 'YData', wc_hist);
            set(obj.h_wang_real_line_, 'XData', t_hist, 'YData', wr_hist);
            
            t_lo = max(0, t_now - obj.VEL_WINDOW);
            t_hi = max(1e-3, t_now); 
            xlim(obj.ax_vel_, [t_lo, t_hi]);
            
            % Dynamic Y-Limits (checking both cmd and real arrays)
            yyaxis(obj.ax_vel_, 'left');
            max_v = max(max(vc_hist), max(vr_hist));
            if max_v > 0.5, ylim(obj.ax_vel_, [-0.1, max_v*1.2]); end
            
            yyaxis(obj.ax_vel_, 'right');
            max_w = max(max(abs(wc_hist)), max(abs(wr_hist)));
            if max_w > 1.4, ylim(obj.ax_vel_, [-max_w*1.2, max_w*1.2]); end
        end

        % --------------------------------------------------------------
        function drawStatus_(obj, pose, v_cmd, w_cmd, v_real, w_real, behavior)
            txt = sprintf('BEHAVIOR: %s  |  POSE: x:%5.2fm y:%5.2fm θ:%5.1f°  |  CMD: v:%5.2f ω:%5.2f  |  REAL: v:%5.2f ω:%5.2f', ...
                upper(behavior), pose.x, pose.y, rad2deg(pose.theta), v_cmd, w_cmd, v_real, w_real);
            set(obj.h_status_, 'String', txt);
        end
    end
end