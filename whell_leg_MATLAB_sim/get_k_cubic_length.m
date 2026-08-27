function K = get_k_cubic_length(leg_length)

    %% 1. 读取三次拟合系数
    persistent coeff

    if isempty(coeff)
        coeff = load('K_cubic_fit_coeffs.mat');
    end
    %% 2. 由三次函数计算 12 个 K 分量
    k11 = polyval(coeff.p11, leg_length);
    k12 = polyval(coeff.p12, leg_length);
    k13 = polyval(coeff.p13, leg_length);
    k14 = polyval(coeff.p14, leg_length);
    k15 = polyval(coeff.p15, leg_length);
    k16 = polyval(coeff.p16, leg_length);

    k21 = polyval(coeff.p21, leg_length);
    k22 = polyval(coeff.p22, leg_length);
    k23 = polyval(coeff.p23, leg_length);
    k24 = polyval(coeff.p24, leg_length);
    k25 = polyval(coeff.p25, leg_length);
    k26 = polyval(coeff.p26, leg_length);

    %% 3. 重新组装成 2×6 的 K 矩阵
    K = [
        k11, k12, k13, k14, k15, k16;
        k21, k22, k23, k24, k25, k26
    ];

end