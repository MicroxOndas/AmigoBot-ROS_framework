fprintf('Initializing new ROS connection...\n');
    
    PC_IP  = "192.168.100.158";
    IP_SIM = "192.168.100.166";
    setenv('ROS_MASTER_URI', 'http://' + IP_SIM + ':11311');
    setenv('ROS_IP', PC_IP);
    
    % Reiniciar nodo por si acaso estaba en un estado zombie
    rosshutdown; 
    rosinit();
    
    fprintf('Subscribing to ROS topics...\n');
    odom   = rossubscriber('/robot0/odom');
    sonar0 = rossubscriber('/robot0/sonar_0', rostype.sensor_msgs_Range);
    sonar1 = rossubscriber('/robot0/sonar_1', rostype.sensor_msgs_Range);
    sonar2 = rossubscriber('/robot0/sonar_2', rostype.sensor_msgs_Range);
    sonar3 = rossubscriber('/robot0/sonar_3', rostype.sensor_msgs_Range);
    sonar4 = rossubscriber('/robot0/sonar_4', rostype.sensor_msgs_Range);
    sonar5 = rossubscriber('/robot0/sonar_5', rostype.sensor_msgs_Range);
    sonar6 = rossubscriber('/robot0/sonar_6', rostype.sensor_msgs_Range);
    sonar7 = rossubscriber('/robot0/sonar_7', rostype.sensor_msgs_Range);
    laser  = rossubscriber('/robot0/laser_1', rostype.sensor_msgs_LaserScan);
    
    pub     = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
    msg_vel = rosmessage(pub);
    
    fprintf('Waiting for robot messages (up to 10 s)...\n');
    pause(3);
    timeout = tic;
    while true
        msg = odom.LatestMessage;
        if ~isempty(msg) && strcmp(msg.ChildFrameId, 'robot0')
            break;
        end
        if toc(timeout) > 10
            error('[main_robot] Timeout: no odometry from robot0. Check ROS connection.');
        end
        pause(0.2);
    end