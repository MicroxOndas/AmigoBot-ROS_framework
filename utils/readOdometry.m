function pose = readOdometry(odom_sub)
%READODOMETRY Parse the latest ROS odometry message into a pose struct.
%
%  pose = readOdometry(odom_sub)
%
%  Input:
%    odom_sub  - rossubscriber handle for /robot0/odom
%
%  Output struct fields:
%    pose.x       - X position in world frame [m]
%    pose.y       - Y position in world frame [m]
%    pose.theta   - Yaw angle (heading) in world frame [rad]
%    pose.vx      - Forward linear velocity [m/s]
%    pose.wz      - Angular (yaw) velocity [rad/s]
%    pose.stamp   - Message timestamp [s]

msg = odom_sub.LatestMessage;

% ---- Position -------------------------------------------------------
pose.x = msg.Pose.Pose.Position.X;
pose.y = msg.Pose.Pose.Position.Y;

% ---- Yaw from quaternion (standard ZYX extraction) ------------------
q = msg.Pose.Pose.Orientation;
pose.theta = atan2(2*(q.W*q.Z + q.X*q.Y), ...
                   1 - 2*(q.Y^2 + q.Z^2));

% ---- Velocities (in robot body frame) --------------------------------
pose.vx = msg.Twist.Twist.Linear.X;
pose.wz = msg.Twist.Twist.Angular.Z;

% ---- Timestamp -------------------------------------------------------
pose.stamp = double(msg.Header.Stamp.Sec) + ...
             double(msg.Header.Stamp.Nsec) * 1e-9;
end
