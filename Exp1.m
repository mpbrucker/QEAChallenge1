d = .24765; % Wheelbase of robot (m)
omega = pi/5; % Angular velocity (rad/s)
time_tot = (2*pi)/omega; % Total time (
R = d/2; % Turning radius (m)

V = omega*R; % Linear velocity (m/s)
V_R = omega*(V/omega + d/2); % Velocity of left wheel (m/s)
V_L = omega*(V/omega - d/2); % Velocity of right wheel (m/s)

pub = rospublisher('raw_vel');
msg = rosmessage(pub);

msg.Data = [V_L, V_R];
send(pub, msg);

start_time = tic;

elapsed = 0;
while (elapsed < time_tot)
    elapsed = toc(start_time);
end

msg.Data = [0, 0];
send(pub, msg);