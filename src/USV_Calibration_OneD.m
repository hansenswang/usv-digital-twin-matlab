function [K_est, T_est, tau_est, K_u_est, tau_u_est, fval] = USV_Calibration_OneD( ...
    measured_file, throttle, delta, dt, p0, lb, ub, const_speed_flag, sensor_delay)
% USV_Calibration_OneD
% 标定五个参数: [K, T, tau, K_u, tau_u]
%
% 重点改进：
% 1. 加入边界惩罚，避免 fminsearch 跑飞
% 2. 标定阶段降采样，显著缩短运行时间
% 3. 对关键动态段加权，改善 T/tau_u 的可辨识性
% 4. 统一与 USV_Physics_OneD 的模型结构，避免“主函数一套、标定一套”

data = load(measured_file);
psi_meas_full = data.psi_meas(:);
u_meas_full = data.u_meas(:);

% ---- 降采样设置（提速关键）----
ds = 5;   % 原 dt=0.01, 标定内部使用 dt_cal=0.05
idx = 1:ds:length(psi_meas_full);

psi_meas = psi_meas_full(idx);
u_meas   = u_meas_full(idx);
throttle_ds = throttle(idx);
delta_ds    = delta(idx);

dt_cal = dt * ds;
n = length(idx);

% 航向解缠，避免跨越 ±180° 时误差炸裂
psi_meas_cont = unwrap(psi_meas * pi/180) * 180/pi;

% 角速度测量
om_meas = local_diff(psi_meas_cont) / dt_cal;
om_meas = local_smooth5(om_meas);

% 构造关键阶段权重：转向建立/回正/大圆段更重要
w = ones(n,1);

t_cal = (0:n-1)' * dt_cal;
w(t_cal > 18 & t_cal <= 30) = 2.2;
w(t_cal > 38 & t_cal <= 52) = 2.2;
w(t_cal > 62 & t_cal <= 98) = 2.8;
w(t_cal > 98 & t_cal <= 110) = 1.8;

fun = @(p) objective_oned( ...
    p, throttle_ds, delta_ds, n, dt_cal, ...
    psi_meas_cont, u_meas, om_meas, ...
    const_speed_flag, sensor_delay, lb, ub, w);

options = optimset( ...
    'Display', 'iter', ...
    'MaxFunEvals', 1800, ...
    'MaxIter', 1200, ...
    'TolFun', 1e-5, ...
    'TolX', 1e-4);

[p_opt, fval] = fminsearch(fun, p0, options);

% 最终参数裁剪回边界内
p_opt = min(max(p_opt(:)', lb(:)'), ub(:)');

K_est     = p_opt(1);
T_est     = p_opt(2);
tau_est   = round(p_opt(3));
K_u_est   = p_opt(4);
tau_u_est = p_opt(5);

end

%% 目标函数
function cost = objective_oned(p, throttle, delta, n, dt, psi_meas, u_meas, om_meas, const_speed_flag, sensor_delay, lb, ub, w)

p = p(:)';

% ---- 边界惩罚 ----
bound_penalty = 0;
for k = 1:numel(p)
    if p(k) < lb(k)
        bound_penalty = bound_penalty + 1e6 * (lb(k) - p(k))^2;
    elseif p(k) > ub(k)
        bound_penalty = bound_penalty + 1e6 * (p(k) - ub(k))^2;
    end
end

K       = p(1);
T       = p(2);
tau_f   = p(3);
K_u     = p(4);
tau_u   = p(5);

if T <= 0 || tau_u <= 0 || K < 0 || K_u < 0
    cost = 1e20 + bound_penalty;
    return;
end

tau = round(max(0, tau_f));
tau_penalty = 5e4 * (tau_f - tau)^2;   % 轻惩罚到整数

% ---- 仿真（与物理函数保持一致）----
r = zeros(n,1);        % [rad/s]
psi_sim = zeros(n,1);  % [deg]
u_sim = zeros(n,1);    % [m/s]

if tau > 0
    delta_buffer = zeros(tau,1);
else
    delta_buffer = [];
end

for i = 2:n
    if tau > 0
        delta_buffer = [delta(i-1); delta_buffer(1:end-1)];
        delta_eff = delta_buffer(end);
    else
        delta_eff = delta(i-1);
    end

    r(i) = r(i-1) + dt * (-r(i-1) + K * delta_eff * pi/180) / max(T, 1e-6);
    psi_sim(i) = psi_sim(i-1) + r(i) * dt * 180/pi;

    if const_speed_flag
        throttle_eff = 1.0;
    else
        throttle_eff = throttle(i-1);
    end
    u_sim(i) = u_sim(i-1) + dt * (-u_sim(i-1) + K_u * throttle_eff) / max(tau_u, 1e-6);

    if u_sim(i) < 0
        u_sim(i) = 0;
    end
end

om_sim = local_diff(psi_sim) / dt;
om_sim = local_smooth5(om_sim);

% ---- 对齐传感器延迟 ----
delay_ds = round(sensor_delay / max(round(dt/dt),1)); %#ok<NASGU>
% 实际上 sensor_delay 是原始步数，这里对应降采样后的步数：
delay_cal = round(sensor_delay / 5);
start_idx = delay_cal + 1;

if start_idx > n
    cost = 1e20 + bound_penalty;
    return;
end

psi_sim_a = psi_sim(start_idx:end);
psi_meas_a = psi_meas(start_idx:end);
u_sim_a = u_sim(start_idx:end);
u_meas_a = u_meas(start_idx:end);
om_sim_a = om_sim(start_idx:end);
om_meas_a = om_meas(start_idx:end);
w_a = w(start_idx:end);

% ---- 误差计算 ----
% 航向角误差采用 wrap 到 [-180,180]，避免角度跨界
err_psi = psi_sim_a - psi_meas_a;
err_psi = mod(err_psi + 180, 360) - 180;

err_u  = u_sim_a - u_meas_a;
err_om = om_sim_a - om_meas_a;

% ---- 组合代价 ----
% 航向角 / 速度 / 角速度 三项联合约束
cost_psi = sum(w_a .* (err_psi.^2));
cost_u   = sum((0.8*w_a) .* (err_u.^2));
cost_om  = sum((1.5*w_a) .* (err_om.^2));

% 稳态/范围约束，防止出现几乎不转的假解
peak_r = max(abs(om_sim_a));
if peak_r < 0.15
    weak_turn_penalty = 2e4 * (0.15 - peak_r)^2;
else
    weak_turn_penalty = 0;
end

cost = cost_psi + 80*cost_u + 12*cost_om + tau_penalty + weak_turn_penalty + bound_penalty;

if ~isfinite(cost)
    cost = 1e20;
end

end

%% 一阶差分，避免直接依赖 diff 的奇怪环境问题
function y = local_diff(x)
x = x(:);
n = length(x);
y = zeros(n,1);
if n >= 2
    y(2:end) = x(2:end) - x(1:end-1);
end
end

%% 简单5点平滑，避免依赖 smoothdata
function y = local_smooth5(x)
x = x(:);
n = length(x);
y = x;
for i = 1:n
    i1 = max(1, i-2);
    i2 = min(n, i+2);
    y(i) = mean(x(i1:i2));
end
end