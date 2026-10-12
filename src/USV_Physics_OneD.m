function [X, Y, psi, r_deg, u, ship_vertices] = USV_Physics_OneD(throttle, delta_cmd, n, p, sensor_flag, const_speed_flag)
% USV_Physics_OneD
% 一维标定用的简化无人船动力学/运动学模型
%
% 输入:
%   throttle         - 油门序列 [0,1]
%   delta_cmd        - 舵角命令 [deg]
%   n                - 仿真步数
%   p                - 参数结构体
%   sensor_flag      - 是否添加测量污染 (0/1)
%   const_speed_flag - 是否恒速
%
% 输出:
%   X, Y             - 平面轨迹 [m]
%   psi              - 航向角 [deg]
%   r_deg            - 转弯角速度 [deg/s]
%   u                - 前向速度 [m/s]
%   ship_vertices    - 动画顶点

if nargin < 5
    sensor_flag = 0;
end
if nargin < 6
    const_speed_flag = false;
end

dt = 0.01;

K     = p.K;
T     = p.T;
tau   = round(max(0, p.tau));
K_u   = p.K_u;
tau_u = p.tau_u;

L = p.L;
W = p.W;

if isfield(p, 'noise')
    sensor_noise = p.noise;
else
    sensor_noise = 0.1;
end

if isfield(p, 'quant')
    sensor_quant = p.quant;
else
    sensor_quant = 0.02;
end

if isfield(p, 'delay')
    sensor_delay = round(max(0, p.delay));
else
    sensor_delay = 0;
end

X = zeros(n,1);
Y = zeros(n,1);
psi = zeros(n,1);      % [deg]
r = zeros(n,1);        % [rad/s]
u = zeros(n,1);        % [m/s]

if tau > 0
    delta_buffer = zeros(tau,1);
else
    delta_buffer = [];
end

% 初值
X(1) = 0;
Y(1) = 0;
psi(1) = 0;
r(1) = 0;
u(1) = 0;

% 主循环
for i = 2:n
    % 1) 舵机/控制延迟
    if tau > 0
        delta_buffer = [delta_cmd(i-1); delta_buffer(1:end-1)];
        delta_eff = delta_buffer(end);
    else
        delta_eff = delta_cmd(i-1);
    end

    % 2) 转向一阶模型
    % dr/dt = (-r + K*delta)/T
    r(i) = r(i-1) + dt * (-r(i-1) + K * delta_eff * pi/180) / max(T, 1e-6);

    % 3) 速度一阶模型
    if const_speed_flag
        throttle_eff = 1.0;
    else
        throttle_eff = throttle(i-1);
    end
    u(i) = u(i-1) + dt * (-u(i-1) + K_u * throttle_eff) / max(tau_u, 1e-6);
    if u(i) < 0
        u(i) = 0;
    end

    % 4) 航向与位置更新
    psi(i) = psi(i-1) + r(i) * dt * 180/pi;

    X(i) = X(i-1) + u(i) * cosd(psi(i)) * dt;
    Y(i) = Y(i-1) + u(i) * sind(psi(i)) * dt;
end

r_deg = r * 180/pi;

% 传感器污染
if sensor_flag
    X = X + randn(n,1) * sensor_noise * 0.02;
    Y = Y + randn(n,1) * sensor_noise * 0.02;
    psi = psi + randn(n,1) * sensor_noise;
    u = u + randn(n,1) * sensor_noise * 0.03;

    if sensor_quant > 0
        X = round(X / sensor_quant) * sensor_quant;
        Y = round(Y / sensor_quant) * sensor_quant;
        psi = round(psi / sensor_quant) * sensor_quant;
        u = round(u / sensor_quant) * sensor_quant;
    end

    if sensor_delay > 0 && sensor_delay < n
        X = [X(sensor_delay+1:end); repmat(X(end), sensor_delay, 1)];
        Y = [Y(sensor_delay+1:end); repmat(Y(end), sensor_delay, 1)];
        psi = [psi(sensor_delay+1:end); repmat(psi(end), sensor_delay, 1)];
        u = [u(sensor_delay+1:end); repmat(u(end), sensor_delay, 1)];
    end
end

% 动画顶点
ship_vertices = zeros(n, 4, 2);
half_l = L / 2;
half_w = W / 2;

body_local = [
    -half_l, -half_w;
     half_l, -half_w;
     half_l,  half_w;
    -half_l,  half_w
];

for i = 1:n
    c = cosd(psi(i));
    s = sind(psi(i));
    R = [c, -s; s, c];
    verts = body_local * R' + [X(i), Y(i)];
    ship_vertices(i,:,:) = verts;
end
end