function sendVelocity(pub, msg, v, w)
%SENDVELOCITY Publish a geometry_msgs/Twist velocity command.
%
%  sendVelocity(pub, msg, v, w)
%
%  Inputs:
%    pub  - rospublisher handle for /robot0/cmd_vel
%    msg  - Pre-allocated Twist message (from rosmessage(pub))
%    v    - Linear velocity  [m/s]  (forward positive)
%    w    - Angular velocity [rad/s] (CCW/left positive)

msg.Linear.X  = v;
msg.Linear.Y  = 0;
msg.Linear.Z  = 0;
msg.Angular.X = 0;
msg.Angular.Y = 0;
msg.Angular.Z = w;
send(pub, msg);
end
