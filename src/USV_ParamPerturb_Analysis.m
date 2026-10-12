function USV_ParamPerturb_Analysis(result_file)
% 参数扰动分析

if nargin < 1 || isempty(result_file)
    [file, path] = uigetfile('*.mat', '选择一个标定结果文件');
    if isequal(file, 0)
        fprintf('未选择文件，函数结束。\n');
        return;
    end
    result_file = fullfile(path, file);
end

S = load(result_file);

reqFields = {'t','delta','throttle','L','B'};
for k = 1:numel(reqFields)
    if ~isfield(S, reqFields{k})
        error('结果文件缺少字段: %s', reqFields{k});
    end
end

K0     = get_field_safe(S, 'K_est',     get_field_safe(S, 'K_true',     0.35));
T0     = get_field_safe(S, 'T_est',     get_field_safe(S, 'T_true',     2.5));
tau0   = round(get_field_safe(S, 'tau_est',   get_field_safe(S, 'tau_true',   2)));
Ku0    = get_field_safe(S, 'K_u_est',   get_field_safe(S, 'K_u_true',   1.2));
tauu0  = get_field_safe(S, 'tau_u_est', get_field_safe(S, 'tau_u_true', 3.0));

noise0 = get_field_safe(S, 'sensor_noise', 0.10);
delay0 = get_field_safe(S, 'sensor_delay', 2);
quant0 = get_field_safe(S, 'sensor_quant', 0.02);
thrust0 = get_field_safe(S, 'max_thrust', 500);

n = length(S.t);
const_speed_flag = false;

base_params = struct( ...
    'K', K0, ...
    'T', T0, ...
    'tau', tau0, ...
    'K_u', Ku0, ...
    'tau_u', tauu0, ...
    'L', S.L, ...
    'W', S.B, ...
    'noise', noise0, ...
    'quant', quant0, ...
    'delay', delay0, ...
    'max_thrust', thrust0);

[X0, Y0, psi0, r0, u0, ~] = USV_Physics_OneD(S.throttle, S.delta, n, base_params, 0, const_speed_flag);

pert_ratio = [-0.20, -0.10, 0.10, 0.20];
labels = {'-20%', '-10%', '+10%', '+20%'};

param_names = {'K', 'T', 'tau', 'K_u', 'tau_u'};
param_cn = {'转向增益 K', '转向时间 T', '舵机延迟 tau', '速度增益 K_u', '速度时间 tau_u'};

mean_pos_err = zeros(length(param_names), length(pert_ratio));
max_pos_err  = zeros(length(param_names), length(pert_ratio));
mean_psi_err = zeros(length(param_names), length(pert_ratio));
mean_u_err   = zeros(length(param_names), length(pert_ratio));

traj_store = cell(length(param_names), length(pert_ratio));

for i = 1:length(param_names)
    for j = 1:length(pert_ratio)
        p = base_params;
        ratio = pert_ratio(j);

        switch param_names{i}
            case 'K'
                p.K = max(0.01, base_params.K * (1 + ratio));
            case 'T'
                p.T = max(0.10, base_params.T * (1 + ratio));
            case 'tau'
                p.tau = max(0, round(base_params.tau + sign(ratio) * max(1, round(abs(base_params.tau * ratio)))));
            case 'K_u'
                p.K_u = max(0.05, base_params.K_u * (1 + ratio));
            case 'tau_u'
                p.tau_u = max(0.10, base_params.tau_u * (1 + ratio));
        end

        [Xp, Yp, psip, ~, up, ~] = USV_Physics_OneD(S.throttle, S.delta, n, p, 0, const_speed_flag);

        pos_err = sqrt((Xp - X0).^2 + (Yp - Y0).^2);
        psi_err = abs(wrapTo180_local(psip - psi0));
        u_err   = abs(up - u0);

        mean_pos_err(i,j) = mean(pos_err);
        max_pos_err(i,j)  = max(pos_err);
        mean_psi_err(i,j) = mean(psi_err);
        mean_u_err(i,j)   = mean(u_err);

        traj_store{i,j} = [Xp, Yp];
    end
end

figure('Name', '参数扰动分析', 'Position', [100, 80, 1600, 900]);

subplot(2,3,1);
bar(mean_pos_err');
grid on;
set(gca, 'XTickLabel', labels);
xlabel('扰动幅度');
ylabel('平均位置误差 [m]');
title('参数扰动对平均位置误差的影响');
legend(param_cn, 'Location', 'northwest');

subplot(2,3,2);
bar(max_pos_err');
grid on;
set(gca, 'XTickLabel', labels);
xlabel('扰动幅度');
ylabel('最大位置误差 [m]');
title('参数扰动对最大位置误差的影响');
legend(param_cn, 'Location', 'northwest');

subplot(2,3,3);
bar(mean_psi_err');
grid on;
set(gca, 'XTickLabel', labels);
xlabel('扰动幅度');
ylabel('平均航向误差 [deg]');
title('参数扰动对航向误差的影响');
legend(param_cn, 'Location', 'northwest');

subplot(2,3,4);
bar(mean_u_err');
grid on;
set(gca, 'XTickLabel', labels);
xlabel('扰动幅度');
ylabel('平均速度误差 [m/s]');
title('参数扰动对速度误差的影响');
legend(param_cn, 'Location', 'northwest');

subplot(2,3,5);
plot(X0, Y0, 'k-', 'LineWidth', 1.8); hold on;
idx_show = [1, 4];
for i = 1:length(param_names)
    tr1 = traj_store{i, idx_show(1)};
    tr2 = traj_store{i, idx_show(2)};
    plot(tr1(:,1), tr1(:,2), '--', 'LineWidth', 1.0);
    plot(tr2(:,1), tr2(:,2), '-',  'LineWidth', 1.0);
end
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('典型扰动轨迹对比');
legend_entries = [{'基准轨迹'}, strcat(param_cn, ' -20%'), strcat(param_cn, ' +20%')];
legend(legend_entries, 'Location', 'bestoutside');

subplot(2,3,6);
sens_score = mean_pos_err + 0.2 * mean_psi_err + 2.0 * mean_u_err;
score_mean = mean(sens_score, 2);
bar(score_mean);
grid on;
set(gca, 'XTickLabel', param_names);
ylabel('综合敏感度');
title('各参数综合敏感度');

fprintf('\n=== 参数扰动分析 ===\n');
for i = 1:length(param_names)
    fprintf('\n%s:\n', param_cn{i});
    for j = 1:length(pert_ratio)
        fprintf('  扰动 %s -> 平均位置误差 %.4f m, 最大位置误差 %.4f m, 平均航向误差 %.4f deg, 平均速度误差 %.4f m/s\n', ...
            labels{j}, mean_pos_err(i,j), max_pos_err(i,j), mean_psi_err(i,j), mean_u_err(i,j));
    end
end

fprintf('\n=== 综合敏感度排序 ===\n');
[score_sorted, idx_sorted] = sort(score_mean, 'descend');
for i = 1:length(idx_sorted)
    fprintf('%d. %s : %.4f\n', i, param_cn{idx_sorted(i)}, score_sorted(i));
end

end

function val = get_field_safe(S, name, default_val)
if isfield(S, name)
    val = S.(name);
else
    val = default_val;
end
end

function ang = wrapTo180_local(ang)
ang = mod(ang + 180, 360) - 180;
end