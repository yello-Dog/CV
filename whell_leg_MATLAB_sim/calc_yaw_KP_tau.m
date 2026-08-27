function [KP_yaw_rate, Iz] = calc_yaw_KP_tau(tau_yaw, r_w)
% calc_yaw_KP_tau
% 根据 yaw_rate 一阶闭环时间常数计算 P 增益
%
% 输入:
%   tau_yaw : yaw_rate 闭环时间常数, s
%   r_w     : 轮子半径, m
%
% 输出:
%   KP_yaw_rate : yaw_rate P 增益, N*m/(rad/s)
%   Iz          : yaw 转动惯量, kg*m^2

    %% ===== 机器人结构参数 =====
    track_width = 0.18;   % 左右轮距, m

    m_body      = 1.44;   % 机体质量, kg
    body_length = 0.135;  % 机体前后长度, m
    body_width  = 0.16;   % 机体左右宽度, m

    %% ===== 单侧腿和轮端质量 =====
    m_rod   = 0.045 * 2;  % 单侧两根杆质量, kg
    m_point = 0.6;        % 单侧轮端等效质点质量, kg
    m_side  = m_rod + m_point;

    %% ===== yaw 转动惯量 =====
    Iz_side = 2 * m_side * (track_width/2)^2;

    Iz_body = (1/12) * m_body * (body_length^2 + body_width^2);

    Iz = Iz_side + Iz_body;

    %% ===== 根据时间常数计算 Kp =====
    KP_yaw_rate = Iz * r_w / (track_width * tau_yaw);

end