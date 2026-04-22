fprintf('Initializing new ROS connection...\n');

PC_IP  = "172.29.31.105";
IP_SIM = "172.29.30.179";
setenv('ROS_MASTER_URI', char('http://' + IP_SIM + ':11311'));
setenv('ROS_IP', char(PC_IP));

% Reiniciar nodo por si acaso estaba en un estado zombie
try
    rosshutdown;
catch
    % Ignorar si no había nodo activo
end
rosinit();

fprintf('Subscribing to ROS topics...\n');
odom   = rossubscriber('/pose');
sonar0 = rossubscriber('/sonar_0', 'sensor_msgs/Range');
sonar1 = rossubscriber('/sonar_1', 'sensor_msgs/Range');
sonar2 = rossubscriber('/sonar_2', 'sensor_msgs/Range');
sonar3 = rossubscriber('/sonar_3', 'sensor_msgs/Range');
sonar4 = rossubscriber('/sonar_4', 'sensor_msgs/Range');
sonar5 = rossubscriber('/sonar_5', 'sensor_msgs/Range');
sonar6 = rossubscriber('/sonar_6', 'sensor_msgs/Range');
sonar7 = rossubscriber('/sonar_7', 'sensor_msgs/Range');
laser  = rossubscriber('/scan',    'sensor_msgs/LaserScan');

pub     = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub);

% ── CORRECCIÓN PRINCIPAL: definir publisher y mensaje de enable_motor ──
pub_enable        = rospublisher('/cmd_motor_state', 'std_msgs/Int32');
msg_enable_motor  = rosmessage(pub_enable);

% Activación de los motores enviando enable_motor = 1
msg_enable_motor.Data = 1;
send(pub_enable, msg_enable_motor);
fprintf('Motors enabled.\n');

fprintf('Waiting for robot messages (up to 10 s)...\n');
pause(3);

timeout = tic;
while true
    msg = odom.LatestMessage;
    if ~isempty(msg) && strcmp(msg.ChildFrameId, 'base_link')
        fprintf('Odometry received from robot. Connection OK.\n');
        break;
    end
    if toc(timeout) > 10
        error('[main_robot] Timeout: no odometry from robot0. Check ROS connection.');
    end
    pause(0.2);
end