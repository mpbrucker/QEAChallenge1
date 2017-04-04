d = .25; % Wheelbase of robot (m)
vMult = .2; % Speed multiplier
tFinal = (pi+.8)/vMult; % Final execution time
syms t a l; % Symbolic variables
assume(t, 'real'); assume(t, 'positive'); assume(a, 'real'); assume(a, 'positive'); assume(l, 'real'); assume(l, 'positive');

x = symfun(-2*a*((l-cos(vMult*t))*cos(vMult*t)+(1-l)), [t a l]); % Equation for x position as a function of time
y = symfun(2*a*(l-cos(vMult*t))*sin(vMult*t), [t a l]); % Equation for y position as a function of time

r = [x y 0]; % Vector function
vel = simplify(diff(r, t)) % Derivative of position function
spd = simplify(norm(vel))

tHat = simplify(vel/spd) % Unit tangent vector
N = simplify(diff(tHat, t)) % Normal vector (not normalized)
omega = simplify(cross(tHat, N)) % Calculate angular velocity as a function of time

a = .4; % Setting the constant a
l = .4; % Setting the constant l


pub = rospublisher('raw_vel');
msg = rosmessage(pub);


startTime = tic;
t = 0; % Elapsed time in the loop
    
robotOrigin = [0; 0]; % Values for calculating the position of the robot based on VL and VR
calcTheta = 0;
curTime = tic;
loopNum = 0;

while (t < tFinal)
    plot(robotOrigin(1), robotOrigin(2), 'ro');
    loopTime = toc(curTime); % Value of deltaT (s)
    curTime = tic;
    
    
    t = toc(startTime); % Current time (s)
    curSpd = double(spd(t, a, l)) % Current spd(m/s)
    omegaCalc = double(omega(t, a, l)) % Current angular velocity(rad/s)
    curOmega = -omegaCalc(3);
    R = curSpd/curOmega;
    VL = (R+(d/2))*curOmega; % Speed of left wheel (m/s)
    VR = (R-(d/2))*curOmega; % Speed of right wheel (m/s)
    
    msg.Data = [VL, VR];
    send(pub, msg)
    
    omegaComp = -(VL-VR)/d; % Computed angular velocity (rad/s)
    deltTheta = omegaComp*loopTime; % Angle swept around ICC (delta theta)
    rotMatrix = [cos(deltTheta) -sin(deltTheta); sin(deltTheta) cos(deltTheta)]; % Rotation matrix
    translationMatrix = robotOrigin+[-sin(calcTheta)*R; cos(calcTheta)*R]; % Translation matrix
    if (loopNum == 1) % This is a flag to avoid a large jump in theta at the beginning
        robotOrigin = (rotMatrix*(robotOrigin-translationMatrix))+translationMatrix; % Perform rotation of robot position around ICC
        calcTheta = calcTheta + deltTheta;
    end
    loopNum = 1;
    axis([-1 1 -1 1]);
    hold on;


end

msg.Data = [0, 0];
send(pub, msg);