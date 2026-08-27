function draw_k_picture()
    close all;
    %% 1. 加载 K 分量数据
    data = load('K_component_vs_leg_data.mat');

    leg = data.leg;
    leg_dense = linspace(min(leg), max(leg), 200);

    %% 2. 第一行 K：T 对 6 个状态的反馈增益
    figure;

    subplot(3,2,1);
    draw_one_k(leg, data.k11, leg_dense, 'k11');

    subplot(3,2,2);
    draw_one_k(leg, data.k12, leg_dense, 'k12');

    subplot(3,2,3);
    draw_one_k(leg, data.k13, leg_dense, 'k13');

    subplot(3,2,4);
    draw_one_k(leg, data.k14, leg_dense, 'k14');

    subplot(3,2,5);
    draw_one_k(leg, data.k15, leg_dense, 'k15');

    subplot(3,2,6);
    draw_one_k(leg, data.k16, leg_dense, 'k16');

    sgtitle('First row of K: wheel torque T gains');

    %% 3. 第二行 K：Tp 对 6 个状态的反馈增益
    figure;

    subplot(3,2,1);
    draw_one_k(leg, data.k21, leg_dense, 'k21');

    subplot(3,2,2);
    draw_one_k(leg, data.k22, leg_dense, 'k22');

    subplot(3,2,3);
    draw_one_k(leg, data.k23, leg_dense, 'k23');

    subplot(3,2,4);
    draw_one_k(leg, data.k24, leg_dense, 'k24');

    subplot(3,2,5);
    draw_one_k(leg, data.k25, leg_dense, 'k25');

    subplot(3,2,6);
    draw_one_k(leg, data.k26, leg_dense, 'k26');

    sgtitle('Second row of K: hip torque Tp gains');

end


function draw_one_k(leg, k_data, leg_dense, name)

    %% 1. 三次拟合
    p = polyfit(leg, k_data, 3);
    k_fit = polyval(p, leg_dense);

    %% 2. 画原始数据点
    plot(leg, k_data, 'o', 'LineWidth', 1.5);
    hold on;

    %% 3. 画三次拟合曲线
    plot(leg_dense, k_fit, '-', 'LineWidth', 1.5);

    %% 4. 图像设置
    grid on;
    xlabel('leg length / m');
    ylabel(name);
    title([name ' cubic fit']);
    legend('raw data', 'cubic fit', 'Location', 'best');

end