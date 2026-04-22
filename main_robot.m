%% MAIN_ROBOT.M  –  Autonomous Robot Navigation System
%  ═══════════════════════════════════════════════════
%
%  Entry point.  Handles ROS initialisation, assembles the ros_handles
%  struct, creates a MissionControl instance, then runs the mission.
%
%  USAGE
%    1. Open MATLAB in the robot_control/ folder (or add it to the path).
%    2. Edit the MISSION section at the bottom.
%    3. Run: >> main_robot
%
%  FILE STRUCTURE
%    robot_control/
%    ├── main_robot.m          ← YOU ARE HERE
%    ├── conect.m              Conection establishment
%    ├── PIDController.m       Generic PID (handle class)
%    ├── MissionControl.m      High-level API (handle class)
%    ├── RobotDashboard.m      Real-time dashboard (handle class)
%    ├── behaviors/
%    │   ├── gotoPoint.m       PID go-to-point
%    │   ├── wallFollower.m    PID wall following
%    │   └── obstacleAvoidance.m  VFF reactive avoidance
%    └── utils/
%        ├── readOdometry.m    Parse odom → pose struct
%        ├── readSensors.m     Aggregate all sensors → sensor struct
%        ├── sendVelocity.m    Publish cmd_vel Twist
%        └── angleDiff.m      Angle subtraction mod 2π

clearvars -except odom sonar0 sonar1 sonar2 sonar3 sonar4 sonar5 sonar6 sonar7 laser pub msg_vel ros_handles mc; 
clc; close all;

% Add subfolders to MATLAB path (only needed once per session)
addpath(fullfile(fileparts(mfilename('fullpath')), 'behaviors'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ═══════════════════════════════════════════════════════════════════
%  SMART ROS INITIALISATION & SUBSCRIBERS
%% ═══════════════════════════════════════════════════════════════════
% Comprobar si ROS ya está inicializado y tenemos las variables clave
if ~ros.internal.Global.isNodeActive || ~exist('odom', 'var') || ~exist('pub', 'var')
    
    conect_irl % conect.m
    
    % Assemble Handles
    ros_handles.pub    = pub;
    ros_handles.msg    = msg_vel;
    ros_handles.odom   = odom;
    ros_handles.sonars = {sonar0, sonar1, sonar2, sonar3, ...
                          sonar4, sonar5, sonar7, sonar6}; % Desordenado a drede
    ros_handles.laser  = laser;
    
    fprintf('Robot connected! Initial pose: x=%.2f y=%.2f\n', ...
        odom.LatestMessage.Pose.Pose.Position.X, ...
        odom.LatestMessage.Pose.Pose.Position.Y);
        
else
    fprintf('ROS is already initialized and subscribers are active. Skipping setup...\n');
    % Si el robot ya está conectado, paramos cualquier movimiento anterior por seguridad
    mc.stop
end

%% ═══════════════════════════════════════════════════════════════════
%  CREATE MISSION CONTROL
%% ═══════════════════════════════════════════════════════════════════
mc = MissionControl(ros_handles, ...
    'Rate',          10, ...      % Hz
    'ShowDashboard', true);       % Open real-time dashboard


%% ═══════════════════════════════════════════════════════════════════
%  MISSION  –  customise this section
%% ═══════════════════════════════════════════════════════════════════

% ── API LIBRARY ─────────────────────────────────
%mc.gotoPoint([2, 1.5]);             % ir a punto
%mc.gotoPoint([2, 1.5, pi/2]);       % ir a punto + orientación final
%mc.gotoPoint([x, y, theta], 'AvoidObstacles', true) % +evitar obstáculos
%mc.gotoPoint([x, y], 'Mode', 'continuous')   % Arco suave (PID orgánico)
%mc.gotoPoint([x, y], 'Mode', 'point-shoot')  % Parar, girar (PID alterado)

%mc.addWaypoint([x, y]);             % encolar waypoint
%mc.runMission();                    % ejecutar cola

%mc.followWall('left',  0.4);        % seguir pared izquierda
%mc.followWall('right', 0.4, 'Duration', 30);
%mc.followWall('left',  0.4, 'UseApproach', true)

%mc.stop();                          % parada de emergencia
%mc.Config                           % acceso a todos los parámetros

%% ═══════════════════════════════════════════════════════════════════
%  OPTIONAL: TUNE PID GAINS  (uncomment to override defaults)
%% ═══════════════════════════════════════════════════════════════════
% mc.setGains('gotoPoint_angle', [2.0, 0.05, 0.30]); //defaults
% mc.setGains('gotoPoint_dist',  [0.5, 0.01, 0.10]);
% mc.setGains('wallFollow',      [1.5, 0.02, 0.40]);

mc.resetMissionLog() % Inicializa el historial.

try
    % ════════════════════════════════════════════════════════════════════
    %                       MAIN MISSION
    % ════════════════════════════════════════════════════════════════════
    %mc.setGains('gotoPoint_dist',[0.5,0,0])
    %mc.setGains('gotoPoint_dist',[0.6,0.1,0])
    
    %mc.drive([0.5,0]);
    %mc.gotoPoint([10,10])
    
    mc.setGains('wallFollow',[0.9,0.008,0.75])
    mc.followWall('left',1,'Duration', 30)

    % ════════════════════════════════════════════════════════════════════
catch exception
    reporte = getReport(exception, 'extended', 'hyperlinks', 'on');
    fprintf(2, 'Error occurred:\n%s\n', reporte);
    
    fprintf(2, 'Emergency breaks engaged!!!\n');
    mc.stop();
end
try
    % ════════════════════════════════════════════════════════════════════
    %                       MISSION REPORT
    % ════════════════════════════════════════════════════════════════════
    datos_mision = mc.getMissionLog();
    %    ReportFactory.generateReport(logData, target_info, mission_name, save_folder)
    %    ReportFactory.generateReport(..., 'ShowLidar', false)
    %    ReportFactory.generateReport(..., 'ShowOrientation', false)
    %    ReportFactory.generateReport(..., 'ShowGains', false)
    ReportFactory.generateReport(mc.getMissionLog(), [2, 2], 'Mision1', 'reportes');
catch exception
     reporte = getReport(exception, 'extended', 'hyperlinks', 'on');
     fprintf(2, 'Error occurred during report gen:\n%s\n', reporte);
end


%% ═══════════════════════════════════════════════════════════════════
%  CLEAN UP
%  Run these lines manually when you are done with the session.
%% ═══════════════════════════════════════════════════════════════════

% if ~isempty(mc.Dashboard), mc.Dashboard.close(); end
% rosshutdown();
