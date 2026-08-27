function [KP_theta, KD_theta] = calc_fai0_diff_PD(fn, zeta, L_L0, R_L0)
% 把KP KD参数进行进一步映射，映射成更加具有物理意义的参数去调

% calc_theta_split_PD
% 计算左右摆角差分控制器的旋转弹簧阻尼参数
%
% 输入:
%   fn    : 目标无阻尼自然频率, Hz
%   zeta  : 阻尼比
%   L_L0  : 左腿当前等效腿长, m
%   R_L0  : 右腿当前等效腿长, m
%
% 输出:
%   KP_theta : 摆角差分旋转弹簧刚度, N*m/rad
%   KD_theta : 摆角差分旋转阻尼, N*m*s/rad

    %% ===== 质量参数 =====
    m_rod   = 0.075 * 2;   % 单侧两根杆总质量, kg
    m_point = 0.625;         % 单侧轮子/端部等效质点质量, kg

    %% ===== 左右单侧转动惯量 =====
    % 杆近似为绕一端转动的细杆: I = 1/3*m*L^2
    % 轮子/端部质量近似为端点质点: I = m*L^2

    I_L_rod   = (1/3) * m_rod * L_L0^2;
    I_L_point = m_point * L_L0^2;
    I_L       = I_L_rod + I_L_point;

    I_R_rod   = (1/3) * m_rod * R_L0^2;
    I_R_point = m_point * R_L0^2;
    I_R       = I_R_rod + I_R_point;

    %% ===== 相对摆角差分模态等效转动惯量 =====
    % 两个转动惯量之间通过虚拟旋转弹簧阻尼连接
    % 等效惯量为约化转动惯量
    I_eq = (I_L * I_R) / (I_L + I_R);

    %% ===== 自然频率 =====
    omega_n = 2*pi*fn;

    %% ===== 旋转弹簧阻尼参数 =====
    KP_theta = I_eq * omega_n^2;
    KD_theta = 2 * zeta * I_eq * omega_n;

end