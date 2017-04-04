syms t

% Describes parametric function
x = symfun(0.5 * cos(t),t);
y = symfun(0.75 * sin(t),t);
r = [x,y,0];

% Calculate Tangent Vector & Speed
T = diff(r);
speed = simplify(norm(T));

T_hat = simplify(T/speed); % Unit Tangent vector

N = simplify(diff(T_hat));
omega = simplify(cross(T_hat,N));

t = linspace(0.001,4,10);
for i=2:10
disp(double(speed(t(i))))
disp(double(omega(t(i))))
end