classdef ReportFactory
%REPORTFACTORY Generates and saves standard mission execution reports in Dark Mode.
%
%  Usage:
%    ReportFactory.generateReport(logData, target_info, mission_name, save_folder)
%    ReportFactory.generateReport(..., 'ShowLidar', false)
%    ReportFactory.generateReport(..., 'ShowOrientation', false)
%    ReportFactory.generateReport(..., 'ShowGains', false)

    methods (Static)
        function fig = generateReport(logData, target_info, mission_name, save_folder, varargin)
            
            % 1. PARSEADOR DE ARGUMENTOS OPCIONALES
            p = inputParser();
            addRequired(p,  'logData');
            addRequired(p,  'target_info');
            addRequired(p,  'mission_name');
            addRequired(p,  'save_folder');
            
            % Parámetros visuales (Todos activados por defecto)
            addParameter(p, 'ShowLidar',       true, @islogical);
            addParameter(p, 'ShowOrientation', true, @islogical);
            addParameter(p, 'ShowGains',       true, @islogical);
            addParameter(p, 'ShowTarget',      true, @islogical);
            
            parse(p, logData, target_info, mission_name, save_folder, varargin{:});
            opt = p.Results;

            % 2. Crear carpeta si no existe
            if ~exist(save_folder, 'dir')
                mkdir(save_folder);
            end

            % 3. Extraer datos 
            t = logData.t; x = logData.x; y = logData.y; yaw = logData.theta;
            
            if numel(t) < 2
                fprintf('[ReportFactory] Not enough data to generate report.\n');
                fig = []; return;
            end
            
            % Estadísticas
            t_total = t(end) - t(1);
            dist_total = sum(sqrt(diff(x).^2 + diff(y).^2));

            % 4. PALETA DE COLORES (Dark Mode)
            C_BG    = [0.08 0.09 0.11]; C_PANEL = [0.12 0.13 0.16];
            C_TEXT  = [0.85 0.88 0.92]; C_GRID  = [0.25 0.27 0.33];
            C_VLIN  = [0.15 0.85 0.55]; C_WANG  = [1.00 0.55 0.15];

            % 5. Configurar Figura
            fig = figure('Name', ['Reporte: ', mission_name], ...
                         'Color', C_BG, 'Position', [100 100 1300 600], ...
                         'Visible', 'off');
            set(fig, 'InvertHardcopy', 'off'); % Mantiene el Dark Mode al exportar

            ax_style = {'Color', C_PANEL, 'XColor', C_TEXT, 'YColor', C_TEXT, ...
                        'GridColor', C_GRID, 'GridAlpha', 0.8, 'FontSize', 11, ...
                        'FontName', 'Helvetica'};

            % ════════════════════════════════════════════════════════════
            % PANEL IZQUIERDO: TRAYECTORIA XY
            % ════════════════════════════════════════════════════════════
            ax_map = subplot(1, 3, [1 2]);
            set(ax_map, ax_style{:});
            hold on; grid on; axis equal;
            
            % ════════════════════════════════════════════════════════════
            % [OPCIONAL] DIBUJAR PAREDES (FILTRO DE DENSIDAD)
            % ════════════════════════════════════════════════════════════
            if opt.ShowLidar && isfield(logData, 'wall_points_x') && ~isempty(logData.wall_points_x)
                
                px = logData.wall_points_x;
                py = logData.wall_points_y;
                
                % 1. Parámetros del Filtro (Ajustables)
                grid_res = 0.1;      % Tamaño de la cuadrícula: 10 cm
                min_density = 11;     % Nivel de exigencia: mínimo 6 impactos para ser pared
                
                % 2. Crear los bordes de la cuadrícula 2D
                x_edges = min(px)-grid_res : grid_res : max(px)+grid_res;
                y_edges = min(py)-grid_res : grid_res : max(py)+grid_res;
                
                % 3. Calcular la densidad de puntos usando un histograma 2D
                [N, Xedges, Yedges] = histcounts2(px, py, x_edges, y_edges);
                
                % 4. Encontrar las celdas que superan la densidad mínima
                [row, col] = find(N >= min_density);
                
                % 5. Obtener las coordenadas reales (el centro de cada celda densa)
                wall_x = Xedges(row) + grid_res/2;
                wall_y = Yedges(col) + grid_res/2;
                
                % 6. Dibujar solo la pared filtrada
                scatter(wall_x, wall_y, 10, [0.15 0.85 0.55], 'filled', ...
                    'MarkerFaceAlpha', 0.9, 'DisplayName', 'Pared Real (Filtrada)');
            end

            % Línea de trayectoria
            plot(x, y, '-', 'Color', [0.2 0.6 1.0], 'LineWidth', 2.0, 'DisplayName', 'Trayectoria');
            
            % [OPCIONAL] FLECHAS DE ORIENTACIÓN
            if opt.ShowOrientation
                N = max(1, floor(numel(t)/30)); 
                quiver(x(1:N:end), y(1:N:end), 0.15*cos(yaw(1:N:end)), 0.15*sin(yaw(1:N:end)), ...
                    0, 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2, 'MaxHeadSize', 1.5, 'DisplayName', 'Orientación');
            end

            % Marcadores de Inicio y Fin
            scatter(x(1), y(1), 120, 'g', 'filled', 'DisplayName', 'Inicio');
            scatter(x(end), y(end), 120, 'b', 's', 'filled', 'DisplayName', 'Posición Final');

            % [OPCIONAL] Marcador de Destino
            if opt.ShowTarget && ~isempty(target_info)
                scatter(target_info(1), target_info(2), 160, 'r', 'p', 'filled', ...
                    'MarkerEdgeColor', C_TEXT, 'DisplayName', 'Destino');
            end

            xlabel('X [m]'); ylabel('Y [m]'); 
            title('Trayectoria del Robot en el Mundo', 'Color', C_TEXT, 'FontSize', 13);
            leg_map = legend('Location', 'northeast');
            set(leg_map, 'TextColor', C_TEXT, 'Color', C_BG, 'EdgeColor', C_GRID);

            % [OPCIONAL] CAJA DE PARÁMETROS Y GANANCIAS K
            if opt.ShowGains && isfield(logData, 'pid_angle')
                p_ang = logData.pid_angle; p_dist = logData.pid_dist;
                param_str = {
                    sprintf('\\bfDATOS MISIÓN\\rm'),
                    sprintf('Distancia: %.2f m', dist_total),
                    sprintf('Tiempo: %.1f s', t_total),
                    '',
                    sprintf('\\bfGANANCIAS PID\\rm'),
                    sprintf('Angular: [P:%.2f I:%.2f D:%.2f]', p_ang.Kp, p_ang.Ki, p_ang.Kd),
                    sprintf('Lineal:  [P:%.2f I:%.2f D:%.2f]', p_dist.Kp, p_dist.Ki, p_dist.Kd)
                };
                text(0.02, 0.96, param_str, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
                    'Color', C_TEXT, 'BackgroundColor', [C_BG 0.7], 'EdgeColor', C_GRID, 'FontSize', 9);
            end

            % ════════════════════════════════════════════════════════════
            % PANEL DERECHO: VELOCIDADES (CONSIGNAS VS REALES)
            % ════════════════════════════════════════════════════════════
            ax_vel = subplot(1, 3, 3);
            set(ax_vel, ax_style{:});
            grid on; hold on;
            
            % Eje Y Izquierdo (Velocidad Lineal)
            yyaxis left;
            ax_vel.YColor = C_VLIN;
            plot(t, logData.v_cmd, '--', 'Color', C_VLIN * 0.6, 'LineWidth', 1.5, 'DisplayName', 'v (cmd)');
            plot(t, logData.v_real, '-',  'Color', C_VLIN,       'LineWidth', 2.0, 'DisplayName', 'v (real)');
            ylabel('Velocidad Lineal [m/s]'); 
            ylim([-0.1, max(max(logData.v_cmd), max(logData.v_real))*1.2 + 0.1]);
            
            % Eje Y Derecho (Velocidad Angular)
            yyaxis right;
            ax_vel.YColor = C_WANG;
            plot(t, logData.w_cmd, '--', 'Color', C_WANG * 0.6, 'LineWidth', 1.5, 'DisplayName', '\omega (cmd)');
            plot(t, logData.w_real, '-',  'Color', C_WANG,       'LineWidth', 2.0, 'DisplayName', '\omega (real)');
            ylabel('Velocidad Angular [rad/s]'); 
            max_w = max(max(abs(logData.w_cmd)), max(abs(logData.w_real)));
            ylim([-max_w*1.2 - 0.1, max_w*1.2 + 0.1]);

            xlabel('Tiempo [s]'); title('Perfil de Velocidades', 'Color', C_TEXT, 'FontSize', 13);
            leg_vel = legend('Location', 'southoutside', 'Orientation', 'horizontal', 'NumColumns', 2);
            set(leg_vel, 'TextColor', C_TEXT, 'Color', C_BG, 'EdgeColor', C_GRID);

            % Título Global
            clean_name = strrep(mission_name, '_', ' '); 
            sgtitle(sprintf('Resultados de Misión: %s', clean_name), ...
                'FontSize', 16, 'FontWeight', 'bold', 'Color', C_TEXT);

            % ════════════════════════════════════════════════════════════
            % GUARDADO Y VISUALIZACIÓN
            % ════════════════════════════════════════════════════════════
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            safe_name = strrep(mission_name, ' ', '_'); 
            filename_base = fullfile(save_folder, sprintf('%s_%s', safe_name, timestamp));
            
            savefig(fig, [filename_base, '.fig']);
            exportgraphics(fig, [filename_base, '.png'], 'Resolution', 300, 'BackgroundColor', C_BG);
            
            set(fig, 'Visible', 'on'); 
            fprintf('[ReportFactory] Reporte guardado en: %s.png\n', filename_base);
        end
    end
end