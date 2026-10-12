function USV_ErrorSource_Analysis(result_file)
% 误差来源分析

if nargin < 1 || isempty(result_file)
    [file, path] = uigetfile('*.mat', '选择一个标定结果文件');
    if isequal(file, 0)
        fprintf('未选择文件，函数结束。\n');
        return;
    end
    result_file = fullfile(path, file);
end

S = load(result_file);

reqFields = {'L','B','sensor_noise','sensor_delay','t','delta','throttle'};
for k = 1:numel(reqFields)
    if ~isfield(S, reqFields{k})
        error('结果文件缺少字段: %s', reqFields{k});
    end
end

if isfield(S,'K_true')
    K_use = S.K_true;
else
    K_use = S.K_est;
end

if isfield(S,'T_true')
    T_use = S.T_true;
else
    T_use = S.T_est;
end

if isfield(S,'tau_true')
    tau_use = S.tau_true;
else
    tau_use = round(S.tau_est);
end

if isfield(S,'K_u_true')
    K_u_use = S.K_u_true;
else
    K_u_use = S.K_u_est;
end

if isfield(S,'tau_u_true')
    tau_u_use = S.tau_u_true;
else
    tau_u_use = S.tau_u_est;
end

if isfield(S,'max_thrust')
    max_thrust = S.max_thrust;
else
    max_thrust = 500;
end

n = length(S.t);
const_speed_flag = false;

base_params = struct( ...
    'K', K_use, ...
    'T', T_use, ...
    'tau', tau_use, ...
    'K_u', K_u_use, ...
    'tau_u', tau_u_use, ...
    'L', S.L, ...
    'W', S.B, ...
    'noise', 0.0, ...
    'quant', 0.02, ...
    'delay', 0, ...
    'max_thrust', max_thrust);

[X0, Y0, psi0, ~, u0, ~] = USV_Physics_OneD(S.throttle, S.delta, n, base_params, 0, const_speed_flag);

p_noise = base_params;
p_noise.noise = S.sensor_noise;
p_noise.delay = 0;
[Xn, Yn, psin, ~, un, ~] = USV_Physics_OneD(S.throttle, S.delta, n, p_noise, 1, const_speed_flag);

p_delay = base_params;
p_delay.noise = 0.0;
p_delay.delay = S.sensor_delay;
[Xd, Yd, psid, ~, ud, ~] = USV_Physics_OneD(S.throttle, S.delta, n, p_delay, 1, const_speed_flag);

p_mix = base_params;
p_mix.noise = S.sensor_noise;
p_mix.delay = S.sensor_delay;
[Xm, Ym, psim, ~, um, ~] = USV_Physics_OneD(S.throttle, S.delta, n, p_mix, 1, const_speed_flag);

err_noise_pos = sqrt((Xn - X0).^2 + (Yn - Y0).^2);
err_delay_pos = sqrt((Xd - X0).^2 + (Yd - Y0).^2);
err_mix_pos   = sqrt((Xm - X0).^2 + (Ym - Y0).^2);

err_noise_psi = abs(wrapTo180_local(psin - psi0));
err_delay_psi = abs(wrapTo180_local(psid - psi0));
err_mix_psi   = abs(wrapTo180_local(psim - psi0));

err_noise_u = abs(un - u0);
err_delay_u = abs(ud - u0);
err_mix_u   = abs(um - u0);

% 轻微平滑，仅用于显示
err_noise_pos_s = smoothdata(err_noise_pos, 'movmean', 25);
err_delay_pos_s = smoothdata(err_delay_pos, 'movmean', 25);
err_mix_pos_s   = smoothdata(err_mix_pos,   'movmean', 25);

err_noise_psi_s = smoothdata(err_noise_psi, 'movmean', 25);
err_delay_psi_s = smoothdata(err_delay_psi, 'movmean', 25);
err_mix_psi_s   = smoothdata(err_mix_psi,   'movmean', 25);

err_noise_u_s = smoothdata(err_noise_u, 'movmean', 25);
err_delay_u_s = smoothdata(err_delay_u, 'movmean', 25);
err_mix_u_s   = smoothdata(err_mix_u,   'movmean', 25);

% 综合评分
noise_score = mean(err_noise_pos) + 0.2*mean(err_noise_psi) + 2.0*mean(err_noise_u);
delay_score = mean(err_delay_pos) + 0.2*mean(err_delay_psi) + 2.0*mean(err_delay_u);
mix_score   = mean(err_mix_pos)   + 0.2*mean(err_mix_psi)   + 2.0*mean(err_mix_u);

coupling_score = mix_score - noise_score - delay_score;
if coupling_score < 0
    coupling_score = 0;
end

pie_vals_raw = [noise_score, delay_score, coupling_score];

% 保证饼图始终完整分满 100%
if sum(pie_vals_raw) <= 1e-12
    pie_vals = [1, 1, 1];
else
    pie_vals = pie_vals_raw / sum(pie_vals_raw);
end

pie_percent = pie_vals * 100;

figure('Name', '误差来源分析', 'Position', [100, 100, 1500, 900]);

subplot(2,3,1);
plot(X0, Y0, 'k-', 'LineWidth', 1.8); hold on;
plot(Xn, Yn, 'r--', 'LineWidth', 1.2);
plot(Xd, Yd, 'b-.', 'LineWidth', 1.2);
plot(Xm, Ym, 'm-', 'LineWidth', 1.2);
grid on; axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('轨迹误差来源对比');
legend('理想', '仅噪声', '仅延迟', '噪声+延迟', 'Location', 'best');

subplot(2,3,2);
plot(S.t, err_noise_pos_s, 'r-', 'LineWidth', 1.2); hold on;
plot(S.t, err_delay_pos_s, 'b-', 'LineWidth', 1.2);
plot(S.t, err_mix_pos_s, 'm-', 'LineWidth', 1.2);
grid on;
xlabel('时间 [s]');
ylabel('位置误差 [m]');
title('位置误差随时间变化');
legend('仅噪声', '仅延迟', '噪声+延迟', 'Location', 'best');

subplot(2,3,3);
plot(S.t, err_noise_psi_s, 'r-', 'LineWidth', 1.2); hold on;
plot(S.t, err_delay_psi_s, 'b-', 'LineWidth', 1.2);
plot(S.t, err_mix_psi_s, 'm-', 'LineWidth', 1.2);
grid on;
xlabel('时间 [s]');
ylabel('航向误差 [deg]');
title('航向误差随时间变化');
legend('仅噪声', '仅延迟', '噪声+延迟', 'Location', 'best');

subplot(2,3,4);
plot(S.t, err_noise_u_s, 'r-', 'LineWidth', 1.2); hold on;
plot(S.t, err_delay_u_s, 'b-', 'LineWidth', 1.2);
plot(S.t, err_mix_u_s, 'm-', 'LineWidth', 1.2);
grid on;
xlabel('时间 [s]');
ylabel('速度误差 [m/s]');
title('速度误差随时间变化');
legend('仅噪声', '仅延迟', '噪声+延迟', 'Location', 'best');

subplot(2,3,5);
bar_data = [ ...
    max(err_noise_pos), mean(err_noise_pos); ...
    max(err_delay_pos), mean(err_delay_pos); ...
    max(err_mix_pos),   mean(err_mix_pos)];
bar(bar_data);
grid on;
set(gca, 'XTickLabel', {'仅噪声','仅延迟','噪声+延迟'});
ylabel('误差 [m]');
title('位置误差统计');
legend('最大值', '平均值', 'Location', 'best');

subplot(2,3,6);
h = pie(pie_vals, {'噪声','延迟','耦合'});
title('误差来源占比');

% 只保留扇区名，避免百分比文字挤在图上
for i = 1:length(h)
    if isa(h(i), 'matlab.graphics.primitive.Text')
        h(i).FontSize = 10;
    end
end

ax = gca;
text(1.15, 0.45, sprintf('噪声贡献   %.1f%%', pie_percent(1)), ...
    'Units', 'normalized', 'FontSize', 10);
text(1.15, 0.30, sprintf('延迟贡献   %.1f%%', pie_percent(2)), ...
    'Units', 'normalized', 'FontSize', 10);
text(1.15, 0.15, sprintf('耦合贡献   %.1f%%', pie_percent(3)), ...
    'Units', 'normalized', 'FontSize', 10);
text(1.15, 0.00, sprintf('合计      %.1f%%', sum(pie_percent)), ...
    'Units', 'normalized', 'FontSize', 10, 'FontWeight', 'bold');

fprintf('\n=== 误差来源分析 ===\n');
fprintf('仅噪声综合误差评分   : %.4f\n', noise_score);
fprintf('仅延迟综合误差评分   : %.4f\n', delay_score);
fprintf('噪声+延迟综合误差评分: %.4f\n', mix_score);
fprintf('耦合项误差评分       : %.4f\n', coupling_score);

fprintf('\n=== 饼图占比 ===\n');
fprintf('噪声贡献   : %.1f%%\n', pie_percent(1));
fprintf('延迟贡献   : %.1f%%\n', pie_percent(2));
fprintf('耦合贡献   : %.1f%%\n', pie_percent(3));
fprintf('合计       : %.1f%%\n', sum(pie_percent));

fprintf('\n=== 位置误差统计 ===\n');
fprintf('仅噪声   最大 %.4f m, 平均 %.4f m\n', max(err_noise_pos), mean(err_noise_pos));
fprintf('仅延迟   最大 %.4f m, 平均 %.4f m\n', max(err_delay_pos), mean(err_delay_pos));
fprintf('噪声+延迟 最大 %.4f m, 平均 %.4f m\n', max(err_mix_pos), mean(err_mix_pos));

fprintf('\n=== 航向误差统计 ===\n');
fprintf('仅噪声   最大 %.4f deg, 平均 %.4f deg\n', max(err_noise_psi), mean(err_noise_psi));
fprintf('仅延迟   最大 %.4f deg, 平均 %.4f deg\n', max(err_delay_psi), mean(err_delay_psi));
fprintf('噪声+延迟 最大 %.4f deg, 平均 %.4f deg\n', max(err_mix_psi), mean(err_mix_psi));

fprintf('\n=== 速度误差统计 ===\n');
fprintf('仅噪声   最大 %.4f m/s, 平均 %.4f m/s\n', max(err_noise_u), mean(err_noise_u));
fprintf('仅延迟   最大 %.4f m/s, 平均 %.4f m/s\n', max(err_delay_u), mean(err_delay_u));
fprintf('噪声+延迟 最大 %.4f m/s, 平均 %.4f m/s\n', max(err_mix_u), mean(err_mix_u));

end

function ang = wrapTo180_local(ang)
ang = mod(ang + 180, 360) - 180;
end