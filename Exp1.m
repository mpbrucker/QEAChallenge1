d = .24765; % Wheelbase of robot (m)
omega = pi/5; % Angular velocity (rad/s)
time_tot = (2*pi)/omega; % Total time (
R = d; % Turning radius (m)

V = omega*R; % Linear velocity (m/s)
V_R = omega*(V/omega + d/2); % Velocity of left wheel (m/s)
V_L = omega*(V/omega - d/2); % Velocity of right wheel (m/s)

pub = rospublisher('raw_vel');
msg = rosmessage(pub);

msg.Data = [V_L, V_R];
send(pub, msg);

startTime = tic;

elapsed = 0;

robotOrigin = [0; 0];
calcTheta = 0;
curWheels = [0; 0];

encSub = rossubscriber('/encoders');
while 1
    encMessage = receive(encSub);
    if any(encMessage.Data)
        cur_L = encMessage.Data(1);
        cur_R = encMessage.Data(2);
        break;
    end
end

prev_L = 0;
prev_R = 0;

VL_cur = 0;
VR_cur = 0;
clf;
curTime = tic;
while (elapsed < time_tot)
    loopTime = toc(curTime); % Value of deltaT (s)
    curTime = tic;
    while 1
        encMessage = receive(encSub);
        if any(encMessage.Data)
            cur_L = encMessage.Data(1);
            cur_R = encMessage.Data(2);
            VL_cur = (cur_L - prev_L)/loopTime % Compute current VL 
            VR_cur = (cur_R - prev_R)/loopTime % Compute current VR
            break;
        end
    end
    prev_L = cur_L;
    prev_R = cur_R;
    
    
    
    omegaComp = (VL_cur-VR_cur)/d; % Computed angular velocity (rad/s)
    angleSwept = omegaComp*loopTime; % Angle swept around ICC
    
    rotMatrix = [cos(angleSwept) -sin(angleSwept); sin(angleSwept) cos(angleSwept)]; % Rotation matrix
    translationMatrix = robotOrigin+[sin(omegaComp)*R; cos(omegaComp)*R]; % Translation matrix
    robotOrigin = (rotMatrix*(robotOrigin-translationMatrix))+translationMatrix; % Perform rotation of robot position around ICC
    calcTheta = calcTheta + angleSwept;
    plot(robotOrigin(1), robotOrigin(2), 'ro');
    hold on;
    elapsed = toc(startTime);

end

msg.Data = [0, 0];
send(pub, msg);