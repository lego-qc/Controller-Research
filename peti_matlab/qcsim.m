
% inertia of quadcopter
m = 2;

arm_len = 0.25;
Ixx = arm_len^2/2 * 0.2 * 4;
Iyy = Ixx;
Izz = arm_len^2 * 0.2 * 4;
I = [Ixx, 0, 0;...
    0, Iyy, 0;...
    0, 0, Izz];

% state of the quadcopter
p = [0 0 0]'; % position
v = [0 0 0]'; % velocity
a = [0 0 0]'; % acceleration

q = [1 0 0 0]'; % rotation
dq = [0 0 0 0]'; % time derivative of rotation
w = [0 0 0]'; % angular velocity
dw = [0 0 0]'; % angular acceleration
w_ = [0 0 0]';
dw_ = [0 0 0]';

% simulation parameters
timestep = 0.01;
steps = 1000;

% input force & torque over time
forces = zeros(3, steps);
torques = zeros(3, steps);

forces = repmat([0, 0, 9.81*m]', 1, steps);
forces(:, 101:120) = repmat([0, 0, 55]', 1, 20);
forces(:, 121:140) = repmat([0, 0, 0]', 1, 20);

torques(:, 101:120) = repmat([0, 0.1, 0.5]', 1, 20);
torques(:, 121:160) = repmat([0, -0.1, -0.5]', 1, 40);
torques(:, 161:180) = repmat([0, 0.1, 0.5]', 1, 20);

% input rotor rpms
rpms = zeros(4, steps);
brpm = 3763;
Erpm = brpm + 200;
rpms = repmat([brpm, brpm, brpm, brpm]', 1, steps);
rpms(:, 101:120) = repmat([brpm, Erpm, brpm, Erpm]', 1, 20);
rpms(:, 121:160) = repmat([Erpm, brpm, Erpm, brpm]', 1, 40);
rpms(:, 161:180) = repmat([brpm, Erpm, brpm, Erpm]', 1, 20);

% plot states
times = (1:steps) * timestep;
positions = zeros(3, steps);
rpys = zeros(3, steps);

% iterate state
for i=1:steps
    Fdrag = -v * norm(v);
    Fgravity = [0, 0, -9.81*m]';
    % --- F = forces(:, i) + Fdrag + Fgravity;
    % --- T = torques(:, i);
    F_ = rotor_lift(rpms(:, i));
    T_ = rotor_torque(rpms(:, i), arm_len);
    F = quatmultiply(quatmultiply(q', [0 0 0 F_]), quatconj(q'));
    T = quatmultiply(quatmultiply(q', [0 T_']), quatconj(q'));    
    F = F(2:4)';
    T = T(2:4)';
    
    F = F + Fdrag + Fgravity;
    
    % advance position
    a = F/m;
    v = v + timestep*a;
    p = p + timestep*v;
    
    % advance rotation
    G = [-q(2) q(1) q(4) -q(3);...
        -q(3) -q(4) q(1) q(2);...
        -q(4) q(3) -q(2) q(1)];
    w_ = 2*G*dq;
    T_ = quatmultiply(quatmultiply(quatconj(q'), [0 T']), q');
    T_ = T_(2:4)';
    
    dw_ = inv(I)*T_ - inv(I)*cross(w_, I*w_);
    w_ = w_ + timestep*dw_;
    dq = 0.5 * G'*w_;
    q = q + timestep*dq;
    q = q/norm(q);
    
    % save for plot
    positions(:, i) = p;
    [angle_y, angle_r, angle_p] = quat2angle(q');
    rpys(:, i) = [angle_p, angle_r, angle_y]';
end

% plot results
figure(1);
hold on;
colors = [1 0 0; 0 1 0; 0.3 0.3 1];
for i=1:3
    plot(times, positions(i, :), 'Color', colors(i, :));
end
figure(2);
hold on;
colors = [1 0 0; 0 1 0; 0.3 0.3 1];
for i=1:3
    plot(times, rpys(i, :), 'Color', colors(i, :));
end
