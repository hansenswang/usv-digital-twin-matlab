% 绘制参数计算流程图
figure('Color','white','Position',[100,100,600,500]);
hold on; axis off;

% 定义框位置（手动调整可实现非对称）
% 输入框
rectangle('Position',[0.35,0.85,0.3,0.08],'EdgeColor','k','LineWidth',1);
text(0.5,0.89,'输入：L, B, T_{max}','HorizontalAlignment','center','FontSize',11);

% 分支箭头
annotation('arrow',[0.5,0.35],[0.85,0.75]);  % 左分支
annotation('arrow',[0.5,0.65],[0.85,0.75]);  % 右分支

% 左分支框
rectangle('Position',[0.15,0.65,0.4,0.08],'EdgeColor','k','LineWidth',1);
text(0.35,0.69,'尺度因子 = L/2.0','HorizontalAlignment','center','FontSize',10);

% K
rectangle('Position',[0.15,0.52,0.4,0.08],'EdgeColor','k','LineWidth',1);
text(0.35,0.56,'K = 0.32 / (尺度因子^{0.7})','HorizontalAlignment','center','FontSize',10);

% T
rectangle('Position',[0.15,0.39,0.4,0.08],'EdgeColor','k','LineWidth',1);
text(0.35,0.43,'T = 2.2 \times 尺度因子^{0.7}','HorizontalAlignment','center','FontSize',10);

% tau_u
rectangle('Position',[0.15,0.26,0.4,0.08],'EdgeColor','k','LineWidth',1);
text(0.35,0.30,'\tau_u = 2.8 \times 尺度因子^{0.45}','HorizontalAlignment','center','FontSize',10);

% 右分支
rectangle('Position',[0.5,0.65,0.35,0.08],'EdgeColor','k','LineWidth',1);
text(0.675,0.69,'船型系数 = L \times B / 2.0','HorizontalAlignment','center','FontSize',10);

rectangle('Position',[0.5,0.52,0.35,0.08],'EdgeColor','k','LineWidth',1);
text(0.675,0.56,'推力因子 = T_{max} / 500','HorizontalAlignment','center','FontSize',10);

rectangle('Position',[0.5,0.39,0.35,0.12],'EdgeColor','k','LineWidth',1);
text(0.675,0.44,'K_u = 1.2 \times 推力因子','HorizontalAlignment','center','FontSize',10);
text(0.675,0.41,'/ (船型系数^{0.55})','HorizontalAlignment','center','FontSize',10);

% 添加箭头连接（手动调整）
annotation('arrow',[0.35,0.35],[0.65,0.6]);
annotation('arrow',[0.35,0.35],[0.52,0.47]);
annotation('arrow',[0.35,0.35],[0.39,0.34]);
annotation('arrow',[0.675,0.675],[0.65,0.6]);
annotation('arrow',[0.675,0.675],[0.52,0.51]);

% 保存图片（可取消注释）
% exportgraphics(gcf,'param_flow.png','Resolution',150);