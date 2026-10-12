%% USV_Main_OneD.m - 一维无人船动态标定系统（小幅优化版）
clear; clc; close all;

% 固定随机种子（保证真实参数可复现）
rng(1234, 'twister');

%% 1. 用户输入
fprintf('========== 一维无人船参数输入 ==========\n');

L_def = 2.0;
B_def = 1.0;
L = input(sprintf('请输入船长 [m] (默认 %.1f): ', L_def));
if isempty(L), L = L_def; end
B = input(sprintf('请输入船宽 [m] (默认 %.1f): ', B_def));
if isempty(B), B = B_def; end

fprintf('\n--- 推力参数设置 ---\n');
thrust_def = 500;   % 具象推力输入，默认500
max_thrust = input(sprintf('请输入最大推力 [N] (默认 %d): ', thrust_def));
if isempty(max_thrust), max_thrust = thrust_def; end

const_speed_flag = false; % 使用油门变化，不使用恒速模式

fprintf('\n--- 传感器参数设置 ---\n');
noise_def = 0.10;
sensor_noise = input(sprintf('请输入传感器噪声标准差 [deg] (默认 %.2f): ', noise_def));
if isempty(sensor_noise), sensor_noise = noise_def; end

sensor_delay_def = 2;
sensor_delay = input(sprintf('请输入传感器延迟步数 (默认 %d): ', sensor_delay_def));
if isempty(sensor_delay), sensor_delay = sensor_delay_def; end

sensor_quant = 0.02;

fprintf('\n--- 推荐测试值（用于论文对比，非强制）---\n');
fprintf('船型对比建议：\n');
fprintf('  小船: L=1.4, B=0.7, 推力=420 N\n');
fprintf('  中船: L=2.0, B=1.0, 推力=500 N\n');
fprintf('  大船: L=3.0, B=1.5, 推力=720 N\n');
fprintf('同一中船推力对比建议：350 / 500 / 750 N\n');
fprintf('同一中船噪声对比建议：0.03 / 0.10 / 0.25\n');
fprintf('同一中船延迟对比建议：1 / 2 / 5\n');

%% 2. 生成真实动力学参数
scale_L = L / 2.0;
scale_B = B / 1.0;

K_true = (0.32 + 0.10*rand) / max(scale_L^0.7, 0.6);
T_true = (2.2 + 0.8*rand) * max(scale_L^0.7, 0.8);
tau_true = randi([1,3]);

thrust_scale = max_thrust / 500;
hull_scale = (L * B) / (2.0 * 1.0);

K_u_true = (1.20 + 0.25*rand) * thrust_scale / max(hull_scale^0.55, 0.7);
tau_u_true = (2.8 + 0.8*rand) * max(scale_L^0.45, 0.8);

fprintf('\n--- 真实动力学参数（自动生成）---\n');
fprintf('K = %.3f, T = %.3f, tau = %d, K_u = %.3f, tau_u = %.3f\n', ...
    K_true, T_true, tau_true, K_u_true, tau_u_true);

true_params = struct( ...
    'K', K_true, ...
    'T', T_true, ...
    'tau', tau_true, ...
    'K_u', K_u_true, ...
    'tau_u', tau_u_true, ...
    'L', L, ...
    'W', B, ...
    'noise', sensor_noise, ...
    'quant', sensor_quant, ...
    'delay', sensor_delay, ...
    'max_thrust', max_thrust);

%% 3. 仿真设置
dt = 0.01;
T_sim = 160;
n = round(T_sim/dt) + 1;
t = (0:n-1)' * dt;

%% 4. 设计激励信号（轨迹进一步优化版）
% 目标：
% 1) 先直行
% 2) 第一次向下偏转更明显（幅度较原方案增加约30%）
% 3) 第二次向上偏转更明显（幅度较原方案增加约30%）
% 4) 最后一段尽量接近完整圆，而不是只画70%左右

delta = zeros(n,1);      % 舵角命令 [deg]
throttle = zeros(n,1);   % 油门命令 [0,1]

throttle(:) = 0.16;

% 0~18s：直行
delta(t >= 0   & t <= 18) = 0;
throttle(t >= 0 & t <= 18) = 0.16;

% 18~29s：第一次下偏（右转）
% 原来 -5.0 deg，这里提高到约 +30% -> -6.5 deg
% 时间略延长1s，让偏折更充分，但仍不至于进入连续绕圈
delta(t > 18 & t <= 29) = -6.5;
throttle(t > 18 & t <= 29) = 0.19;

% 29~39s：回正稳定
delta(t > 29 & t <= 39) = 0;
throttle(t > 29 & t <= 39) = 0.16;

% 39~51s：第二次上偏（左转）
% 原来 +5.5 deg，这里提高到约 +30% -> +7.1 deg
% 时间也略延长，使第二段上偏更清楚
delta(t > 39 & t <= 51) = +7.1;
throttle(t > 39 & t <= 51) = 0.19;

% 51~64s：回正稳定
delta(t > 51 & t <= 64) = 0;
throttle(t > 51 & t <= 64) = 0.16;

% 64~124.3s：持续较大打舵（+5秒），确保达到约75%甚至更接近掉头
delta(t > 64 & t <= 124.3) = +11.0;
throttle(t > 64 & t <= 124.3) = 0.22;

% 124.3~126.3s：短暂回舵直行，利用惯性继续前出
delta(t > 124.3 & t <= 126.3) = 0.0;
throttle(t > 124.3 & t <= 126.3) = 0.20;

% 126.3~138.0s：明显反向打舵，把船头往水平向左拉回来（继续延长）
delta(t > 126.3 & t <= 138.0) = -6.5;
throttle(t > 126.3 & t <= 138.0) = 0.18;

% 138.0~146.0s：减小反打舵幅度，继续细调方向
delta(t > 138.0 & t <= 146.0) = -3.0;
throttle(t > 138.0 & t <= 146.0) = 0.17;

% 146.0~150.0s：轻微回舵，避免继续下切
delta(t > 146.0 & t <= 150.0) = 1.5;
throttle(t > 146.0 & t <= 150.0) = 0.16;

% 150.0~154.0s：更轻微修正，平滑拉到接近水平向左
delta(t > 150.0 & t <= 154.0) = 0.8;
throttle(t > 150.0 & t <= 154.0) = 0.16;

% 154.0s之后：回正直行收尾
delta(t > 154.0) = 0.0;
throttle(t > 154.0) = 0.16;

ctrl_delta = delta;
ctrl_throttle = throttle;
%% 5. 生成理想和测量数据
[X_id, Y_id, psi_id, r_id, u_id, ~] = USV_Physics_OneD( ...
    throttle, delta, n, true_params, 0, const_speed_flag);

rng(42);
[~, ~, psi_meas, ~, u_meas, ~] = USV_Physics_OneD( ...
    throttle, delta, n, true_params, 1, const_speed_flag);

save('measured_oned.mat', 'psi_meas', 'u_meas');

%% 6. 标定参数设置
p0 = [0.35, 2.5, 2, 1.2, 3.0];
lb = [0.05, 0.5, 0, 0.3, 0.5];
ub = [1.20, 8.0, 8, 3.0, 8.0];

fprintf('\n开始标定...\n');
[K_est, T_est, tau_est, K_u_est, tau_u_est, fval] = USV_Calibration_OneD( ...
    'measured_oned.mat', throttle, delta, dt, p0, lb, ub, const_speed_flag, sensor_delay);

fprintf('\n=== 标定结果 ===\n');
fprintf('参数              真实值      估计值      误差%%\n');
fprintf('转向增益 K:       %8.3f   %8.3f   %9.1f\n', K_true, K_est, (K_est-K_true)/K_true*100);
fprintf('转向时间 T:       %8.3f   %8.3f   %9.1f\n', T_true, T_est, (T_est-T_true)/T_true*100);

if tau_true ~= 0
    tau_err_percent = (round(tau_est)-tau_true)/tau_true*100;
else
    tau_err_percent = 0;
end
fprintf('舵机延迟 tau:     %8d   %8d   %9.1f\n', tau_true, round(tau_est), tau_err_percent);

fprintf('速度增益 K_u:     %8.3f   %8.3f   %9.1f\n', K_u_true, K_u_est, (K_u_est-K_u_true)/K_u_true*100);
fprintf('速度时间 tau_u:   %8.3f   %8.3f   %9.1f\n', tau_u_true, tau_u_est, (tau_u_est-tau_u_true)/tau_u_true*100);

%% 7. 用估计参数重新仿真
est_params = true_params;
est_params.K = K_est;
est_params.T = T_est;
est_params.tau = round(tau_est);
est_params.K_u = K_u_est;
est_params.tau_u = tau_u_est;

[X_opt, Y_opt, psi_opt, r_opt, u_opt, ~] = USV_Physics_OneD( ...
    throttle, delta, n, est_params, 0, const_speed_flag);

%% 8. 计算性能指标
max_turn_rate_real = max(abs(r_id));
max_turn_rate_est  = max(abs(r_opt));

idx_est = abs(r_opt) > 0.05;
if any(idx_est)
    radius_est = u_opt(idx_est) ./ max(abs(r_opt(idx_est))*pi/180, 1e-6);
    min_radius_est = min(radius_est);
    mean_radius_est = mean(radius_est);
    max_radius_est = max(radius_est);
else
    min_radius_est = NaN;
    mean_radius_est = NaN;
    max_radius_est = NaN;
end

idx_real = abs(r_id) > 0.05;
if any(idx_real)
    radius_real = u_id(idx_real) ./ max(abs(r_id(idx_real))*pi/180, 1e-6);
    min_radius_real = min(radius_real);
    mean_radius_real = mean(radius_real);
    max_radius_real = max(radius_real);
else
    min_radius_real = NaN;
    mean_radius_real = NaN;
    max_radius_real = NaN;
end

pos_error = sqrt((X_opt - X_id).^2 + (Y_opt - Y_id).^2);
max_pos_error = max(pos_error);
mean_pos_error = mean(pos_error);

% 新增：最大速度、最大加速度
max_speed_real = max(u_id);
max_speed_est  = max(u_opt);

acc_real = [0; diff(u_id)] / dt;
acc_est  = [0; diff(u_opt)] / dt;

max_acc_real = max(abs(acc_real));
max_acc_est  = max(abs(acc_est));

fprintf('\n=== 性能指标 ===\n');
fprintf('最大转弯速率 真实: %.2f deg/s, 估计: %.2f deg/s, 误差: %+.1f%%\n', ...
    max_turn_rate_real, max_turn_rate_est, ...
    100*(max_turn_rate_est-max_turn_rate_real)/max(max_turn_rate_real,1e-6));

if ~isnan(min_radius_real) && ~isnan(min_radius_est)
    fprintf('最小转弯半径 真实: %.2f m, 估计: %.2f m, 误差: %+.1f%%\n', ...
        min_radius_real, min_radius_est, ...
        100*(min_radius_est-min_radius_real)/max(min_radius_real,1e-6));
else
    fprintf('最小转弯半径 真实: %.2f m, 估计: %.2f m, 误差: NaN%%\n', min_radius_real, min_radius_est);
end

if ~isnan(mean_radius_real) && ~isnan(mean_radius_est)
    fprintf('平均转弯半径 真实: %.2f m, 估计: %.2f m, 误差: %+.1f%%\n', ...
        mean_radius_real, mean_radius_est, ...
        100*(mean_radius_est-mean_radius_real)/max(mean_radius_real,1e-6));
else
    fprintf('平均转弯半径 真实: %.2f m, 估计: %.2f m, 误差: NaN%%\n', mean_radius_real, mean_radius_est);
end

if ~isnan(max_radius_real) && ~isnan(max_radius_est)
    fprintf('最大转弯半径 真实: %.2f m, 估计: %.2f m, 误差: %+.1f%%\n', ...
        max_radius_real, max_radius_est, ...
        100*(max_radius_est-max_radius_real)/max(max_radius_real,1e-6));
else
    fprintf('最大转弯半径 真实: %.2f m, 估计: %.2f m, 误差: NaN%%\n', max_radius_real, max_radius_est);
end

fprintf('最大速度     真实: %.3f m/s, 估计: %.3f m/s, 误差: %+.1f%%\n', ...
    max_speed_real, max_speed_est, ...
    100*(max_speed_est-max_speed_real)/max(max_speed_real,1e-6));

fprintf('最大加速度   真实: %.3f m/s^2, 估计: %.3f m/s^2, 误差: %+.1f%%\n', ...
    max_acc_real, max_acc_est, ...
    100*(max_acc_est-max_acc_real)/max(max_acc_real,1e-6));

fprintf('轨迹最大位置误差: %.3f m\n', max_pos_error);
fprintf('轨迹平均位置误差: %.3f m\n', mean_pos_error);

%% 9. 可视化
figure('Position', [100, 100, 1600, 900]);

plot_stride = 80;
idx_plot = 1:plot_stride:n;

subplot(3,3,1);
plot(X_id, Y_id, 'g-', 'LineWidth', 1.8); hold on;
plot(X_opt, Y_opt, 'b--', 'LineWidth', 1.6);
% 增加轨迹采样点（小红点）
plot(X_id(idx_plot), Y_id(idx_plot), 'r.', 'MarkerSize', 4);
legend('真实轨迹', '标定后轨迹', '采样点', 'Location', 'best');
xlabel('X [m]'); ylabel('Y [m]');
title('轨迹对比');
grid on; axis equal;

subplot(3,3,2);
plot(t, psi_id, 'g-', 'LineWidth', 1.4); hold on;
plot(t(idx_plot), psi_meas(idx_plot), 'r.', 'MarkerSize', 6);
plot(t, psi_opt, 'b--', 'LineWidth', 1.4);
xlabel('时间 [s]'); ylabel('艏向角 [deg]');
title('航向角对比');
legend('真实', '测量', '标定后', 'Location', 'best');
grid on;

subplot(3,3,3);
% 单独设置更稀疏采样点（避免过密）
idx_plot_u = 1:160:n;
plot(t, u_id, 'g-', 'LineWidth', 1.4); hold on;
plot(t(idx_plot_u), u_meas(idx_plot_u), 'r.', 'MarkerSize', 5);
plot(t, u_opt, 'b--', 'LineWidth', 1.4);
xlabel('时间 [s]'); ylabel('速度 [m/s]');
title('速度对比');
legend('真实', '测量', '标定后', 'Location', 'best');
grid on;

subplot(3,3,4);
param_names = {'K','T','tau','K_u','tau_u'};
true_vals = [K_true, T_true, tau_true, K_u_true, tau_u_true];
est_vals = [K_est, T_est, tau_est, K_u_est, tau_u_est];
error_vals = 100*(est_vals - true_vals) ./ max(abs(true_vals), 1e-6);
bar(error_vals);
set(gca, 'XTickLabel', param_names);
ylabel('误差 [%]');
title('参数估计相对误差');
grid on;

subplot(3,3,5);
metrics_real = [max_turn_rate_real, min_radius_real, mean_radius_real, max_radius_real, max_speed_real, max_acc_real];
metrics_est  = [max_turn_rate_est,  min_radius_est,  mean_radius_est,  max_radius_est,  max_speed_est,  max_acc_est];

% 改为标准分组柱状图（确保最大速度、加速度显示清晰）
metrics_mat = [metrics_real(:), metrics_est(:)];
bar(metrics_mat);

set(gca, 'XTick', 1:6);
set(gca, 'XTickLabel', {'最大速率','最小半径','平均半径','最大半径','最大速度','最大加速度'});
ylabel('数值');
title('运动性能指标对比');
legend('真实值', '标定后估算', 'Location', 'northwest');
grid on;

subplot(3,3,6);
yyaxis left;
plot(t, delta, 'k-', 'LineWidth', 1.2);
ylabel('舵角 [deg]');
yyaxis right;
plot(t, throttle, 'm-', 'LineWidth', 1.2);
ylabel('油门 [-]');
xlabel('时间 [s]');
title('控制输入');
grid on;

psi_error = psi_opt - psi_id;
psi_error = mod(psi_error + 180, 360) - 180;
subplot(3,3,7);
plot(t, psi_error, 'LineWidth', 1.2);
xlabel('时间 [s]'); ylabel('航向角误差 [deg]');
title('航向角误差');
grid on;

subplot(3,3,8);
plot(t, pos_error, 'LineWidth', 1.2);
xlabel('时间 [s]'); ylabel('位置误差 [m]');
title('轨迹偏差');
grid on;

subplot(3,3,9);
text(0.08, 0.92, sprintf('最佳代价: %.2e', fval), 'FontSize', 11);
text(0.08, 0.82, sprintf('K = %.3f', K_est), 'FontSize', 10);
text(0.08, 0.73, sprintf('T = %.3f', T_est), 'FontSize', 10);
text(0.08, 0.64, sprintf('tau = %d', round(tau_est)), 'FontSize', 10);
text(0.08, 0.55, sprintf('K_u = %.3f', K_u_est), 'FontSize', 10);
text(0.08, 0.46, sprintf('tau_u = %.3f', tau_u_est), 'FontSize', 10);
text(0.08, 0.34, sprintf('最大速度 = %.3f m/s', max_speed_est), 'FontSize', 10);
text(0.08, 0.25, sprintf('最大加速度 = %.3f m/s^2', max_acc_est), 'FontSize', 10);
text(0.08, 0.16, sprintf('平均轨迹误差 = %.3f m', mean_pos_error), 'FontSize', 10);
axis off;
title('标定摘要');

%% 10. 保存结果
save_opt = input('\n是否保存本次标定结果到文件？(y/n，默认n): ', 's');
if strcmpi(save_opt, 'y')
    default_name = sprintf('result_oned_L%.1f_B%.1f_T%.0f.mat', L, B, max_thrust);
    [file, path] = uiputfile('*.mat', '保存结果文件为', default_name);
    if ~isequal(file, 0)
        save(fullfile(path, file), ...
            'L', 'B', 'max_thrust', 'sensor_noise', 'sensor_delay', ...
            'K_true', 'T_true', 'tau_true', 'K_u_true', 'tau_u_true', ...
            'K_est', 'T_est', 'tau_est', 'K_u_est', 'tau_u_est', ...
            'max_turn_rate_real', 'max_turn_rate_est', ...
            'min_radius_real', 'min_radius_est', ...
            'mean_radius_real', 'mean_radius_est', ...
            'max_radius_real', 'max_radius_est', ...
            'max_speed_real', 'max_speed_est', ...
            'max_acc_real', 'max_acc_est', ...
            'max_pos_error', 'mean_pos_error', ...
            'X_id', 'Y_id', 'psi_id', 'X_opt', 'Y_opt', 'psi_opt', 't', ...
            'delta', 'throttle');
        fprintf('结果已保存。\n');
    end
end

%% 11. 动画
play_anim = input('\n是否播放动画？(y/n，默认n): ', 's');
if strcmpi(play_anim, 'y')
    [X_id_a, Y_id_a, psi_id_a, ~, ~, vertices_id] = USV_Physics_OneD( ...
        throttle, delta, n, true_params, 0, const_speed_flag);
    [X_opt_a, Y_opt_a, psi_opt_a, ~, ~, vertices_opt] = USV_Physics_OneD( ...
        throttle, delta, n, est_params, 0, const_speed_flag);

    fig_anim = figure('Name', '船体动画 - 真实(绿) vs 标定后(蓝)');
    ax = axes('Parent', fig_anim);
    hold(ax, 'on'); axis equal;

    x_min = min([X_id_a; X_opt_a]); x_max = max([X_id_a; X_opt_a]);
    y_min = min([Y_id_a; Y_opt_a]); y_max = max([Y_id_a; Y_opt_a]);

    span = max(x_max - x_min, y_max - y_min);
    if span < 1
        span = 1;
    end
    margin = 0.15 * span;

    xlim(ax, [x_min-margin, x_max+margin]);
    ylim(ax, [y_min-margin, y_max+margin]);

    plot(ax, X_id_a, Y_id_a, 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);

    h_traj_real = plot(ax, X_id_a(1), Y_id_a(1), 'g-', 'LineWidth', 1.5);
    h_traj_opt  = plot(ax, X_opt_a(1), Y_opt_a(1), 'b--', 'LineWidth', 1.5);

    h_ship_real = patch(ax, 'XData', squeeze(vertices_id(1,:,1)), ...
        'YData', squeeze(vertices_id(1,:,2)), 'FaceColor', 'g', 'FaceAlpha', 0.30);
    h_ship_opt = patch(ax, 'XData', squeeze(vertices_opt(1,:,1)), ...
        'YData', squeeze(vertices_opt(1,:,2)), 'FaceColor', 'b', 'FaceAlpha', 0.30);

    heading_len = max(0.6*L, 0.8);

    h_heading_real = plot(ax, [X_id_a(1), X_id_a(1)+heading_len*cosd(psi_id_a(1))], ...
        [Y_id_a(1), Y_id_a(1)+heading_len*sind(psi_id_a(1))], 'g-', 'LineWidth', 1.2);
    h_heading_opt  = plot(ax, [X_opt_a(1), X_opt_a(1)+heading_len*cosd(psi_opt_a(1))], ...
        [Y_opt_a(1), Y_opt_a(1)+heading_len*sind(psi_opt_a(1))], 'b-', 'LineWidth', 1.2);

    h_rudder_real = plot(ax, [X_id_a(1), X_id_a(1)+heading_len*cosd(psi_id_a(1)+delta(1))], ...
        [Y_id_a(1), Y_id_a(1)+heading_len*sind(psi_id_a(1)+delta(1))], 'r-', 'LineWidth', 1.0);
    h_rudder_opt = plot(ax, [X_opt_a(1), X_opt_a(1)+heading_len*cosd(psi_opt_a(1)+delta(1))], ...
        [Y_opt_a(1), Y_opt_a(1)+heading_len*sind(psi_opt_a(1)+delta(1))], 'm-', 'LineWidth', 1.0);

    h_title = title(ax, sprintf('时间 = %.1f s', t(1)));

    try
        for i = 1:10:n
            if ~isvalid(h_traj_real), break; end

            set(h_traj_real, 'XData', X_id_a(1:i), 'YData', Y_id_a(1:i));
            set(h_traj_opt,  'XData', X_opt_a(1:i), 'YData', Y_opt_a(1:i));

            set(h_ship_real, 'XData', squeeze(vertices_id(i,:,1)), ...
                'YData', squeeze(vertices_id(i,:,2)));
            set(h_ship_opt, 'XData', squeeze(vertices_opt(i,:,1)), ...
                'YData', squeeze(vertices_opt(i,:,2)));

            set(h_heading_real, 'XData', [X_id_a(i), X_id_a(i)+heading_len*cosd(psi_id_a(i))], ...
                'YData', [Y_id_a(i), Y_id_a(i)+heading_len*sind(psi_id_a(i))]);
            set(h_heading_opt, 'XData', [X_opt_a(i), X_opt_a(i)+heading_len*cosd(psi_opt_a(i))], ...
                'YData', [Y_opt_a(i), Y_opt_a(i)+heading_len*sind(psi_opt_a(i))]);

            set(h_rudder_real, 'XData', [X_id_a(i), X_id_a(i)+heading_len*cosd(psi_id_a(i)+delta(i))], ...
                'YData', [Y_id_a(i), Y_id_a(i)+heading_len*sind(psi_id_a(i)+delta(i))]);
            set(h_rudder_opt, 'XData', [X_opt_a(i), X_opt_a(i)+heading_len*cosd(psi_opt_a(i)+delta(i))], ...
                'YData', [Y_opt_a(i), Y_opt_a(i)+heading_len*sind(psi_opt_a(i)+delta(i))]);

            set(h_title, 'String', sprintf('时间 = %.1f s', t(i)));
            drawnow;
            pause(0.01);
        end
    catch ME
        fprintf('动画中断: %s\n', ME.message);
    end
end

fprintf('\n演示完成！\n');