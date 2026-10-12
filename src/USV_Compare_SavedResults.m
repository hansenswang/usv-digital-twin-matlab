function USV_Compare_SavedResults(file1, file2)
% USV_Compare_SavedResults
% 读取两个保存结果 MAT 文件，在同一窗口中进行增强对比显示
%
% 功能：
% 1) 轨迹对比（含采样点）
% 2) 航向角对比
% 3) 误差/速度/加速度等关键指标对比
% 4) 参数对比（附中文解释）
% 5) 右下角摘要采用左右并排布局，增强可读性
%
% 用法：
%   USV_Compare_SavedResults
%   USV_Compare_SavedResults('a.mat', 'b.mat')

clc;

%% 1. 选择文件
if nargin < 1 || isempty(file1)
    [f1, p1] = uigetfile('*.mat', '选择第一个结果文件');
    if isequal(f1, 0)
        fprintf('已取消选择第一个文件。\n');
        return;
    end
    file1 = fullfile(p1, f1);
end

if nargin < 2 || isempty(file2)
    [f2, p2] = uigetfile('*.mat', '选择第二个结果文件');
    if isequal(f2, 0)
        fprintf('已取消选择第二个文件。\n');
        return;
    end
    file2 = fullfile(p2, f2);
end

%% 2. 读取数据
S1 = load(file1);
S2 = load(file2);

name1 = get_file_name_only(file1);
name2 = get_file_name_only(file2);

R1 = build_result_struct(S1, name1);
R2 = build_result_struct(S2, name2);

% 尝试补充派生量
R1 = enrich_metrics(R1);
R2 = enrich_metrics(R2);

%% 3. 控制台打印
fprintf('\n================ 两组结果对比 ================\n');
fprintf('文件1：%s\n', file1);
fprintf('文件2：%s\n', file2);

fprintf('\n------------ 参数对比（附中文解释） ------------\n');
fprintf('%-18s %-16s %-16s %-30s\n', '参数名', R1.name, R2.name, '中文意义');
fprintf('%-18s %-16s %-16s %-30s\n', 'K_est',      num2str_safe(R1.K_est),      num2str_safe(R2.K_est),      '转向增益：舵角引起角速度的能力');
fprintf('%-18s %-16s %-16s %-30s\n', 'T_est',      num2str_safe(R1.T_est),      num2str_safe(R2.T_est),      '转向时间常数：转向响应快慢');
fprintf('%-18s %-16s %-16s %-30s\n', 'tau_est',    num2str_safe(R1.tau_est),    num2str_safe(R2.tau_est),    '舵机延迟步数：控制输入滞后');
fprintf('%-18s %-16s %-16s %-30s\n', 'K_u_est',    num2str_safe(R1.K_u_est),    num2str_safe(R2.K_u_est),    '速度增益：油门转化为速度的能力');
fprintf('%-18s %-16s %-16s %-30s\n', 'tau_u_est',  num2str_safe(R1.tau_u_est),  num2str_safe(R2.tau_u_est),  '速度时间常数：速度建立快慢');

fprintf('\n------------ 关键性能结果对比 ------------\n');
fprintf('%-22s %-16s %-16s %-28s\n', '指标', R1.name, R2.name, '中文意义');
fprintf('%-22s %-16s %-16s %-28s\n', '最大速度 [m/s]',     num2str_safe(R1.max_speed_est),      num2str_safe(R2.max_speed_est),      '船体达到的最高前进速度');
fprintf('%-22s %-16s %-16s %-28s\n', '最大加速度 [m/s^2]', num2str_safe(R1.max_acc_est),        num2str_safe(R2.max_acc_est),        '速度变化最剧烈的程度');
fprintf('%-22s %-16s %-16s %-28s\n', '最大转弯速率',       num2str_safe(R1.max_turn_rate_est),  num2str_safe(R2.max_turn_rate_est),  '单位时间内航向变化最快值');
fprintf('%-22s %-16s %-16s %-28s\n', '最小转弯半径',       num2str_safe(R1.min_radius_est),     num2str_safe(R2.min_radius_est),     '转弯最急时的圆弧半径');
fprintf('%-22s %-16s %-16s %-28s\n', '平均转弯半径',       num2str_safe(R1.mean_radius_est),    num2str_safe(R2.mean_radius_est),    '整体转弯尺度平均值');
fprintf('%-22s %-16s %-16s %-28s\n', '最大转弯半径',       num2str_safe(R1.max_radius_est),     num2str_safe(R2.max_radius_est),     '转弯最缓时的圆弧半径');
fprintf('%-22s %-16s %-16s %-28s\n', '最大轨迹误差 [m]',   num2str_safe(R1.max_pos_error),      num2str_safe(R2.max_pos_error),      '两条轨迹最大偏离距离');
fprintf('%-22s %-16s %-16s %-28s\n', '平均轨迹误差 [m]',   num2str_safe(R1.mean_pos_error),     num2str_safe(R2.mean_pos_error),     '整体轨迹平均偏离程度');

%% 4. 作图
figure('Name', '无人船双结果对比（增强版）', ...
    'Position', [60, 40, 1700, 950], 'Color', 'w');

c1 = [0.00 0.45 0.74];
c2 = [0.85 0.33 0.10];
c1_light = [0.65 0.82 0.95];
c2_light = [0.98 0.78 0.64];
mk1 = '.';
mk2 = 'o';

%% 子图1：轨迹对比 + 采样点
subplot(2,3,1); hold on; grid on; axis equal;
title('轨迹对比（含采样点）');
xlabel('X [m]'); ylabel('Y [m]');

if ~isempty(R1.X_id) && ~isempty(R1.Y_id)
    plot(R1.X_id, R1.Y_id, '-', 'Color', c1_light, 'LineWidth', 1.0);
end
if ~isempty(R1.X_opt) && ~isempty(R1.Y_opt)
    plot(R1.X_opt, R1.Y_opt, '--', 'Color', c1, 'LineWidth', 2.0);
    idx1 = sample_index(length(R1.X_opt), 40);
    plot(R1.X_opt(idx1), R1.Y_opt(idx1), mk1, 'Color', c1, 'MarkerSize', 10);
end

if ~isempty(R2.X_id) && ~isempty(R2.Y_id)
    plot(R2.X_id, R2.Y_id, '-', 'Color', c2_light, 'LineWidth', 1.0);
end
if ~isempty(R2.X_opt) && ~isempty(R2.Y_opt)
    plot(R2.X_opt, R2.Y_opt, '--', 'Color', c2, 'LineWidth', 2.0);
    idx2 = sample_index(length(R2.X_opt), 40);
    plot(R2.X_opt(idx2), R2.Y_opt(idx2), mk2, 'Color', c2, 'MarkerSize', 4);
end

legend_text = {};
if ~isempty(R1.X_id),  legend_text{end+1} = [R1.name ' 真实轨迹']; end
if ~isempty(R1.X_opt), legend_text{end+1} = [R1.name ' 标定轨迹']; end
if ~isempty(R2.X_id),  legend_text{end+1} = [R2.name ' 真实轨迹']; end
if ~isempty(R2.X_opt), legend_text{end+1} = [R2.name ' 标定轨迹']; end
if ~isempty(legend_text)
    legend(legend_text, 'Location', 'best');
end

%% 子图2：航向角对比
subplot(2,3,2); hold on; grid on;
title('航向角对比');
xlabel('时间 [s]'); ylabel('航向角 [deg]');

if ~isempty(R1.t) && ~isempty(R1.psi_id)
    plot(R1.t, R1.psi_id, '-', 'Color', c1_light, 'LineWidth', 1.2);
end
if ~isempty(R1.t) && ~isempty(R1.psi_opt)
    plot(R1.t, R1.psi_opt, '--', 'Color', c1, 'LineWidth', 2.0);
end
if ~isempty(R2.t) && ~isempty(R2.psi_id)
    plot(R2.t, R2.psi_id, '-', 'Color', c2_light, 'LineWidth', 1.2);
end
if ~isempty(R2.t) && ~isempty(R2.psi_opt)
    plot(R2.t, R2.psi_opt, '--', 'Color', c2, 'LineWidth', 2.0);
end

legend_text = {};
if ~isempty(R1.psi_id),  legend_text{end+1} = [R1.name ' 真实']; end
if ~isempty(R1.psi_opt), legend_text{end+1} = [R1.name ' 标定']; end
if ~isempty(R2.psi_id),  legend_text{end+1} = [R2.name ' 真实']; end
if ~isempty(R2.psi_opt), legend_text{end+1} = [R2.name ' 标定']; end
if ~isempty(legend_text)
    legend(legend_text, 'Location', 'best');
end

%% 子图3：关键误差对比
subplot(2,3,3); hold on; grid on;
title('关键误差对比');
err_mat = [safe_num(R1.max_pos_error), safe_num(R1.mean_pos_error); ...
           safe_num(R2.max_pos_error), safe_num(R2.mean_pos_error)];
bh = bar(err_mat');
bh(1).FaceColor = c1;
bh(2).FaceColor = c2;
set(gca, 'XTickLabel', {'最大轨迹误差','平均轨迹误差'});
ylabel('误差 [m]');
legend({R1.name, R2.name}, 'Location', 'best');

%% 子图4：标定参数对比（附说明）
subplot(2,3,4);

param_names = {'K','T','tau','K_u','tau_u'};
param_desc  = {'转向增益','转向时间常数','舵机延迟','速度增益','速度时间常数'};

vals1 = [getv(S1,'K_est',NaN), getv(S1,'T_est',NaN), getv(S1,'tau_est',NaN), ...
         getv(S1,'K_u_est',NaN), getv(S1,'tau_u_est',NaN)];
vals2 = [getv(S2,'K_est',NaN), getv(S2,'T_est',NaN), getv(S2,'tau_est',NaN), ...
         getv(S2,'K_u_est',NaN), getv(S2,'tau_u_est',NaN)];
bar_data = [vals1(:), vals2(:)];
hb = bar(bar_data, 'grouped');
grid on;
title('标定参数对比');
ylabel('估计值');
set(gca, 'XTick', 1:numel(param_names), 'XTickLabel', param_names);

legend([hb(1), hb(2)], {name1, name2}, 'Location', 'northwest');

yl = ylim;
y_span = yl(2) - yl(1);
if y_span <= 0
    y_span = 1;
end

for i = 1:numel(param_names)
    x1 = hb(1).XEndPoints(i);
    y1 = hb(1).YEndPoints(i);
    x2 = hb(2).XEndPoints(i);
    y2 = hb(2).YEndPoints(i);

    if ~isnan(y1)
        text(x1, y1 + 0.02*y_span, sprintf('%.2f', vals1(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8);
    end
    if ~isnan(y2)
        text(x2, y2 + 0.02*y_span, sprintf('%.2f', vals2(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8);
    end
end

for i = 1:numel(param_names)
    text(i, yl(1) - 0.10*y_span, param_desc{i}, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontSize', 8);
end

ylim([yl(1) - 0.16*y_span, yl(2) + 0.08*y_span]);

%% 子图5：运动性能指标对比（修复图例/标签）
subplot(2,3,5); hold on; grid on;
title('运动性能指标对比');
metric_mat = [safe_num(R1.max_turn_rate_est), safe_num(R1.min_radius_est), safe_num(R1.mean_radius_est), safe_num(R1.max_radius_est); ...
              safe_num(R2.max_turn_rate_est), safe_num(R2.min_radius_est), safe_num(R2.mean_radius_est), safe_num(R2.max_radius_est)];
bh = bar(metric_mat');
bh(1).FaceColor = c1;
bh(2).FaceColor = c2;
set(gca, 'XTickLabel', {'最大转弯速率','最小转弯半径','平均转弯半径','最大转弯半径'});
ylabel('数值');
legend({R1.name, R2.name}, 'Location', 'best');

%% 子图6：左右并排摘要
subplot(2,3,6);
axis off;
title('关键结果摘要（左右对比）');

% 左右两栏标题
text(0.05, 0.95, R1.name, 'Color', c1, 'FontWeight', 'bold', 'FontSize', 12, 'Interpreter', 'none');
text(0.55, 0.95, R2.name, 'Color', c2, 'FontWeight', 'bold', 'FontSize', 12, 'Interpreter', 'none');

% 中间字段名
labels = { ...
    '最大速度 [m/s]'; ...
    '最大加速度 [m/s^2]'; ...
    '最大转弯速率'; ...
    '最小转弯半径 [m]'; ...
    '平均转弯半径 [m]'; ...
    '最大转弯半径 [m]'; ...
    '最大轨迹误差 [m]'; ...
    '平均轨迹误差 [m]'; ...
    'K（转向增益）'; ...
    'T（转向时常）'; ...
    'tau（舵延迟）'; ...
    'Ku（速度增益）'; ...
    'tau_u（速度时常）'};

vals1 = { ...
    num2str_safe(R1.max_speed_est); ...
    num2str_safe(R1.max_acc_est); ...
    num2str_safe(R1.max_turn_rate_est); ...
    num2str_safe(R1.min_radius_est); ...
    num2str_safe(R1.mean_radius_est); ...
    num2str_safe(R1.max_radius_est); ...
    num2str_safe(R1.max_pos_error); ...
    num2str_safe(R1.mean_pos_error); ...
    num2str_safe(R1.K_est); ...
    num2str_safe(R1.T_est); ...
    num2str_safe(R1.tau_est); ...
    num2str_safe(R1.K_u_est); ...
    num2str_safe(R1.tau_u_est)};

vals2 = { ...
    num2str_safe(R2.max_speed_est); ...
    num2str_safe(R2.max_acc_est); ...
    num2str_safe(R2.max_turn_rate_est); ...
    num2str_safe(R2.min_radius_est); ...
    num2str_safe(R2.mean_radius_est); ...
    num2str_safe(R2.max_radius_est); ...
    num2str_safe(R2.max_pos_error); ...
    num2str_safe(R2.mean_pos_error); ...
    num2str_safe(R2.K_est); ...
    num2str_safe(R2.T_est); ...
    num2str_safe(R2.tau_est); ...
    num2str_safe(R2.K_u_est); ...
    num2str_safe(R2.tau_u_est)};

y0 = 0.88;
dy = 0.06;
for i = 1:length(labels)
    y = y0 - (i-1)*dy;
    text(0.04, y, vals1{i}, 'Color', c1, 'FontSize', 10, 'HorizontalAlignment', 'left');
    text(0.37, y, labels{i}, 'Color', [0 0 0], 'FontSize', 10, 'HorizontalAlignment', 'center');
    text(0.70, y, vals2{i}, 'Color', c2, 'FontSize', 10, 'HorizontalAlignment', 'left');
end

sgtitle('无人船两组保存结果对比分析（增强版）', 'FontWeight', 'bold', 'FontSize', 14);

fprintf('\n对比图已生成。\n');

end

%% ======================= 辅助函数 =======================

function R = build_result_struct(S, default_name)
R = struct();
R.name = default_name;

R.L = getfield_safe(S, 'L', []);
R.B = getfield_safe(S, 'B', []);
R.sensor_noise = getfield_safe(S, 'sensor_noise', []);
R.sensor_delay = getfield_safe(S, 'sensor_delay', []);

R.K_true = getfield_safe(S, 'K_true', []);
R.T_true = getfield_safe(S, 'T_true', []);
R.tau_true = getfield_safe(S, 'tau_true', []);
R.K_u_true = getfield_safe(S, 'K_u_true', []);
R.tau_u_true = getfield_safe(S, 'tau_u_true', []);

R.K_est = getfield_safe(S, 'K_est', []);
R.T_est = getfield_safe(S, 'T_est', []);
R.tau_est = getfield_safe(S, 'tau_est', []);
R.K_u_est = getfield_safe(S, 'K_u_est', []);
R.tau_u_est = getfield_safe(S, 'tau_u_est', []);

R.max_turn_rate_real = getfield_safe(S, 'max_turn_rate_real', []);
R.max_turn_rate_est = getfield_safe(S, 'max_turn_rate_est', []);
R.min_radius_real = getfield_safe(S, 'min_radius_real', []);
R.min_radius_est = getfield_safe(S, 'min_radius_est', []);
R.mean_radius_real = getfield_safe(S, 'mean_radius_real', []);
R.mean_radius_est = getfield_safe(S, 'mean_radius_est', []);
R.max_radius_real = getfield_safe(S, 'max_radius_real', []);
R.max_radius_est = getfield_safe(S, 'max_radius_est', []);

R.max_pos_error = getfield_safe(S, 'max_pos_error', []);
R.mean_pos_error = getfield_safe(S, 'mean_pos_error', []);

R.X_id = getfield_safe(S, 'X_id', []);
R.Y_id = getfield_safe(S, 'Y_id', []);
R.psi_id = getfield_safe(S, 'psi_id', []);
R.X_opt = getfield_safe(S, 'X_opt', []);
R.Y_opt = getfield_safe(S, 'Y_opt', []);
R.psi_opt = getfield_safe(S, 'psi_opt', []);
R.t = getfield_safe(S, 't', []);

% 可选：如果你后面主函数保存了这些变量，会自动显示
R.u_id = getfield_safe(S, 'u_id', []);
R.u_opt = getfield_safe(S, 'u_opt', []);
R.max_speed_est = getfield_safe(S, 'max_speed_est', []);
R.max_acc_est = getfield_safe(S, 'max_acc_est', []);
end

function R = enrich_metrics(R)
% 尝试自动补充最大速度、最大加速度

% 1) 最大速度
if isempty(R.max_speed_est)
    if ~isempty(R.u_opt)
        R.max_speed_est = max(abs(R.u_opt));
    elseif ~isempty(R.X_opt) && ~isempty(R.Y_opt) && ~isempty(R.t) && length(R.t) > 1
        dt = mean(diff(R.t));
        vx = [0; diff(R.X_opt)] / dt;
        vy = [0; diff(R.Y_opt)] / dt;
        v = sqrt(vx.^2 + vy.^2);
        R.max_speed_est = max(v);
    else
        R.max_speed_est = [];
    end
end

% 2) 最大加速度
if isempty(R.max_acc_est)
    if ~isempty(R.u_opt) && ~isempty(R.t) && length(R.t) > 1
        dt = mean(diff(R.t));
        a = [0; diff(R.u_opt)] / dt;
        R.max_acc_est = max(abs(a));
    elseif ~isempty(R.X_opt) && ~isempty(R.Y_opt) && ~isempty(R.t) && length(R.t) > 2
        dt = mean(diff(R.t));
        vx = [0; diff(R.X_opt)] / dt;
        vy = [0; diff(R.Y_opt)] / dt;
        v = sqrt(vx.^2 + vy.^2);
        a = [0; diff(v)] / dt;
        R.max_acc_est = max(abs(a));
    else
        R.max_acc_est = [];
    end
end
end

function val = getfield_safe(S, field_name, default_val)
if isfield(S, field_name)
    val = S.(field_name);
else
    val = default_val;
end
end

function s = num2str_safe(x)
if isempty(x)
    s = 'N/A';
elseif isnumeric(x) && isscalar(x)
    if isnan(x)
        s = 'NaN';
    elseif isinf(x)
        s = 'Inf';
    else
        s = sprintf('%.3f', x);
    end
else
    s = 'N/A';
end
end

function x = safe_num(v)
if isempty(v) || ~isnumeric(v) || ~isscalar(v) || isnan(v) || isinf(v)
    x = 0;
else
    x = v;
end
end

function idx = sample_index(n, target_points)
if n <= 0
    idx = [];
    return;
end
step = max(1, floor(n / target_points));
idx = 1:step:n;
if idx(end) ~= n
    idx(end+1) = n;
end
end

function name = get_file_name_only(filepath)
[~, name, ext] = fileparts(filepath);
name = [name ext];
end
function v = getv(S, field, default_val)
    if isfield(S, field)
        v = S.(field);
    else
        v = default_val;
    end
end