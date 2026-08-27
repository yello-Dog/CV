function poly_fit_ks()
%之所以把计算12个三次函数拟合之后的系数和真正的根据腿长实时获得K的
%方程分开，是因为这个只需要算一次，实时获取K的需要重复调用
    data = load('K_component_vs_leg_data.mat');
    
    leg = data.leg;
    
    % 2. 对每一个 K 分量做三次拟合
    p11 = polyfit(leg, data.k11, 3);
    p12 = polyfit(leg, data.k12, 3);
    p13 = polyfit(leg, data.k13, 3);
    p14 = polyfit(leg, data.k14, 3);
    p15 = polyfit(leg, data.k15, 3);
    p16 = polyfit(leg, data.k16, 3);
    
    p21 = polyfit(leg, data.k21, 3);
    p22 = polyfit(leg, data.k22, 3);
    p23 = polyfit(leg, data.k23, 3);
    p24 = polyfit(leg, data.k24, 3);
    p25 = polyfit(leg, data.k25, 3);
    p26 = polyfit(leg, data.k26, 3);
    
    % 3. 保存三次拟合系数
    save('K_cubic_fit_coeffs.mat', ...
        'p11', 'p12', 'p13', 'p14', 'p15', 'p16', ...
        'p21', 'p22', 'p23', 'p24', 'p25', 'p26');
    
    disp('三次拟合系数已经保存到 K_cubic_fit_coeffs.mat');
%% 打印全部系数
     coeff_names = {
        'p11', 'p12', 'p13', 'p14', 'p15', 'p16', ...
        'p21', 'p22', 'p23', 'p24', 'p25', 'p26'
    };

    coeff_values = {
        p11, p12, p13, p14, p15, p16, ...
        p21, p22, p23, p24, p25, p26
    };

    fprintf('\n========== K三次拟合系数 ==========\n');
    fprintf('系数顺序：[三次项, 二次项, 一次项, 常数项]\n\n');

    for i = 1:numel(coeff_names)
        p = coeff_values{i};

        fprintf('%s = [%.10ef, %.10ef, %.10ef, %.10ef]\n', ...
            coeff_names{i}, p(1), p(2), p(3), p(4));
    end

end