clear; clc;


%% 0. 缓存文件名
cacheFile = 'wheel_leg_dynamic_cache.mat';


%% 1. 如果已经保存了那么直接加载，没有的话计算
if isfile(cacheFile)

    disp('发现缓存文件，直接读取已经简化好的动力学和线性化结果...');
    load(cacheFile);

else
    % 这个是状态变量
    syms theta dtheta x dx phi dphi real
    % 这个是所需的状态变量的导数项，需要分别求解为状态变量和输入的方程。求解之后可以拼出完整的动力学
    syms ddtheta ddx ddphi real
    % 这个数输入项
    syms T Tp real
    % 这个是被动力，是动力学建模得到的八个方程里面要消除的项
    syms Nf N P NM PM real
    % 这个是轮足系统参数
    syms Rw Iw mw M L LM ell mp g Ip IM real
    
    %% 2. 写加速度表达式
    a_px = ddx + L*cos(theta)*ddtheta - L*sin(theta)*dtheta^2;
    a_py = -L*sin(theta)*ddtheta - L*cos(theta)*dtheta^2;
    
    a_mx = ddx + (L + LM)*cos(theta)*ddtheta ...
              - (L + LM)*sin(theta)*dtheta^2 ...
              - ell*cos(phi)*ddphi ...
              + ell*sin(phi)*dphi^2;
    
    a_my = -(L + LM)*sin(theta)*ddtheta ...
           - (L + LM)*cos(theta)*dtheta^2 ...
           - ell*sin(phi)*ddphi ...
           - ell*cos(phi)*dphi^2;
    
    %% 3. 写 8 个动力学方程
    % 驱动轮的两个方程
    eq1 = mw*ddx == Nf - N;
    
    eq2 = Iw*ddx/Rw == T - Nf*Rw;
    
    % 摆杆的三个方程
    eq3 = N - NM == mp*a_px;
    
    eq4 = P - PM - mp*g == mp*a_py;
    
    eq5 = Ip*ddtheta == (P*L + PM*LM)*sin(theta) ...
                       - (N*L + NM*LM)*cos(theta) ...
                       - T + Tp;
    % 机体的三个方程
    eq6 = NM == M*a_mx;
    
    eq7 = PM - M*g == M*a_my;
    
    eq8 = IM*ddphi == Tp + NM*ell*cos(phi) + PM*ell*sin(phi);
    
    eqs = [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8];
    
    %% 4. 指定要求解的变量
    unknowns = [ddtheta, ddx, ddphi, Nf, N, P, NM, PM];
    
    %% 5. solve 求解
    sol = solve(eqs, unknowns);
    
    %% 6. 取出你关心的加速度
    f_ddtheta = simplify(sol.ddtheta);
    f_ddx     = simplify(sol.ddx);
    f_ddphi   = simplify(sol.ddphi);
    
    %% 7. 拼状态空间方程 Xdot = f(X,u)
    X = [theta; dtheta; x; dx; phi; dphi];
    U = [T; Tp];
    
    Xdot = [
        dtheta;
        f_ddtheta;
        dx;
        f_ddx;
        dphi;
        f_ddphi
    ];
    
    % 最后Xdot就是状态空间方程
    Xdot = simplify(Xdot);
    
    %% 8. 求符号雅可比
    A_lsym = simplify(jacobian(Xdot, X));
    B_lsym = simplify(jacobian(Xdot, U));
    
    %% 9. 指定线性化点
    X0 = [0; 0; x; 0; 0; 0];
    U0 = [0; 0];
    
    %% 10. 在该点代入
    A_0 = subs(A_lsym, [X; U], [X0; U0]);
    B_0 = subs(B_lsym, [X; U], [X0; U0]);
    %% 11. 保存结果
    disp('正在保存缓存文件...');

    save(cacheFile, ...
        'X', 'U', 'X0', 'U0', ...
        'Xdot', ...
        'f_ddtheta', 'f_ddx', 'f_ddphi', ...
        'A_lsym', 'B_lsym', ...
        'A_0', 'B_0');

    disp('保存完成。');
end
%% 12. 打印 A、B 矩阵
disp('A_num = ');
disp(A_0);

disp('B_num = ');
disp(B_0);
%=============================动力学模型计算完事================================%

%% 13 计算变腿长LQR增益，判断是适合三次拟合

% 1. 腿长序列
leg = 0.10:0.01:0.21;
n = length(leg);

% 2. 预分配 K 的每个分量
k11 = zeros(1, n);
k12 = zeros(1, n);
k13 = zeros(1, n);
k14 = zeros(1, n);
k15 = zeros(1, n);
k16 = zeros(1, n);

k21 = zeros(1, n);
k22 = zeros(1, n);
k23 = zeros(1, n);
k24 = zeros(1, n);
k25 = zeros(1, n);
k26 = zeros(1, n);

% 3. 遍历不同腿长，计算 K
j = 1;

for i = leg
    K = get_k_length(i);

    k11(j) = K(1,1);
    k12(j) = K(1,2);
    k13(j) = K(1,3);
    k14(j) = K(1,4);
    k15(j) = K(1,5);
    k16(j) = K(1,6);

    k21(j) = K(2,1);
    k22(j) = K(2,2);
    k23(j) = K(2,3);
    k24(j) = K(2,4);
    k25(j) = K(2,5);
    k26(j) = K(2,6);

    j = j + 1;
end
% 4. 保存腿长和每个 K 分量
save('K_component_vs_leg_data.mat', ...
    'leg', ...
    'k11', 'k12', 'k13', 'k14', 'k15', 'k16', ...
    'k21', 'k22', 'k23', 'k24', 'k25', 'k26');

disp('K 分量数据已经保存到 K_component_vs_leg_data.mat');
% 5. 调用画图函数
draw_k_picture();
% 6. 三次函数拟合系数计算
poly_fit_ks();

%============到此为止，成果是有了给定腿长就能获得K的函数==========
%% 五连杆运动学解算 
% 1. 定义符号变量
% 输入角
syms phi1 phi4 real

% IMU 机体角
syms phi real

% 结构参数
LAE=0.08;
l1=0.075;
l2=0.14;
%l3=l2;
l4=l1;

% 要求解的输出变量
syms XB YB XD YD L0 phi0 phi2 phi3 real

% 0. 如果已经保存了那么直接加载，没有的话计算
cacheFile = 'fivebar_kinematics_symbolic.mat';
if isfile(cacheFile)
    disp('发现缓存文件，直接加载计算过的腿长和theta关于髋电机角度的方程...');
    load(cacheFile)
else
    % 1. 直接按几何关系求解，不用 solve 三角方程

    % 4 个 B、D 点坐标
    XB_sol = simplify(l1*cos(phi1));
    YB_sol = simplify(l1*sin(phi1));

    XD_sol = simplify(LAE + l4*cos(phi4));
    YD_sol = simplify(l4*sin(phi4));

    % B 到 D 的向量
    dxBD = simplify(XD_sol - XB_sol);
    dyBD = simplify(YD_sol - YB_sol);
    dBD2 = simplify(dxBD^2 + dyBD^2);
    assumeAlso(dBD2 > 0);        % 防止除以 0
    assumeAlso(dBD2 < 4*l2^2);   % 保证两圆有真实交点
    dBD = simplify(sqrt(dBD2));

    % 两圆交点公式
    % C 点满足：
    % |C-B| = l2
    % |C-D| = l3

    a = simplify(dBD/2);
    h2 = simplify(l2^2 - dBD2/4);
    assumeAlso(h2 > 0);          % 保证 h 是实数正数
    
    h = simplify(sqrt(h2));
    
    % 选择装配分支，上方交点
    branch = 1;

    XC_sol = simplify(XB_sol + a*dxBD/dBD - branch*h*dyBD/dBD);
    YC_sol = simplify(YB_sol + a*dyBD/dBD + branch*h*dxBD/dBD);

    % 虚拟腿：AE 中点到 C 点
    XO = LAE/2;
    YO = 0;

    L0_sq = simplify((XC_sol - XO)^2 + (YC_sol - YO)^2);
    assumeAlso(L0_sq > 0);       % 防止虚拟腿长度为 0
    
    L0_sol_f1 = simplify(sqrt(L0_sq));

    % phi0：虚拟腿相对 x 轴角度
    phi0_sol_f2 = atan2(YC_sol - YO, XC_sol - XO);

    % phi2：B 到 C 的杆角
    phi2_sol = atan2(YC_sol - YB_sol, XC_sol - XB_sol);

    % phi3：D 到 C 的杆角
    phi3_sol = atan2(YC_sol - YD_sol, XC_sol - XD_sol);

    % theta 方程
    theta_sol = simplify(phi0_sol_f2 - pi/2 - phi);

    % 2. 保存
    save(cacheFile, ...
        'XB_sol', 'YB_sol', 'XD_sol', 'YD_sol', ...
        'XC_sol', 'YC_sol', ...
        'L0_sol_f1', 'phi0_sol_f2', 'phi2_sol', 'phi3_sol', ...
        'theta_sol');

end
% 8. 打印结果
disp('L0(phi1, phi4) = ');
disp(L0_sol_f1);

disp('phi0(phi1, phi4) = ');
disp(phi0_sol_f2);

disp('theta(phi1, phi4, phi) = ');
disp(theta_sol);

%% 五连杆动力学/VMC映射
% 已知：
% L0_sol_f1   = f1(phi1, phi4)
% phi0_sol_f2 = f2(phi1, phi4)
%
% 目标：
% [T1; T4] = J' * [F; TP]

% 1. 定义虚拟广义力
syms F TP real

    % 1. 直接
% 2. 广义坐标
q_motor = [phi1; phi4];

% 3. 虚拟腿变量
x_virtual = [L0_sol_f1; phi0_sol_f2];

cacheFile = 'fivebar_vmc_symbolic.mat';
if isfile(cacheFile)
    disp('发现缓存文件，直接加载计算过的髋电机T1T2关于F和TP的方程...');
    load(cacheFile)
else
% 4. 手写实数雅可比，不让 MATLAB 对 atan2 自动求导

XO = LAE/2;
YO = 0;

x_leg = simplify(XC_sol - XO);
y_leg = simplify(YC_sol - YO);

L0_sq = simplify(x_leg^2 + y_leg^2);
assumeAlso(L0_sq > 0);

dx_dphi1 = simplify(diff(x_leg, phi1));
dx_dphi4 = simplify(diff(x_leg, phi4));

dy_dphi1 = simplify(diff(y_leg, phi1));
dy_dphi4 = simplify(diff(y_leg, phi4));

% L0 = sqrt(x^2 + y^2)
dL0_dphi1 = simplify((x_leg*dx_dphi1 + y_leg*dy_dphi1) / sqrt(L0_sq), ...
                     'IgnoreAnalyticConstraints', true);

dL0_dphi4 = simplify((x_leg*dx_dphi4 + y_leg*dy_dphi4) / sqrt(L0_sq), ...
                     'IgnoreAnalyticConstraints', true);

% phi0 = atan2(y, x)
% dphi0 = (x dy - y dx)/(x^2 + y^2)
dphi0_dphi1 = simplify((x_leg*dy_dphi1 - y_leg*dx_dphi1) / L0_sq, ...
                       'IgnoreAnalyticConstraints', true);

dphi0_dphi4 = simplify((x_leg*dy_dphi4 - y_leg*dx_dphi4) / L0_sq, ...
                       'IgnoreAnalyticConstraints', true);

J_vmc = [
    dL0_dphi1,   dL0_dphi4;
    dphi0_dphi1, dphi0_dphi4
];

Q_virtual = [F; TP];

T_motor = simplify(J_vmc.' * Q_virtual, ...
                   'IgnoreAnalyticConstraints', true);

T1_sol = simplify(T_motor(1), 'IgnoreAnalyticConstraints', true);
T4_sol = simplify(T_motor(2), 'IgnoreAnalyticConstraints', true);

    % 6. 保存
    save('fivebar_vmc_symbolic.mat', ...
        'J_vmc', ...
        'dL0_dphi1', 'dL0_dphi4', ...
        'dphi0_dphi1', 'dphi0_dphi4', ...
        'T1_sol', 'T4_sol');
end
% 7. 打印结构形式，不建议打印完整展开式，太长
disp('T1_sol = ');
disp(T1_sol);

disp('T4_sol = ');
disp(T4_sol);
%% 检查是否包含复数
% check_fivebar_symbolic_realness.m
% 检查五连杆符号表达式里是否混入 real/imag/复数逻辑
% 前提：工作区里已经有 L0_sol_f1, phi0_sol_f2, T1_sol, T4_sol 等符号表达式

% ---------- 自动加载五连杆符号表达式 ----------
kinCache = 'fivebar_kinematics_symbolic.mat';
vmcCache = 'fivebar_vmc_symbolic.mat';

if isfile(kinCache)
    fprintf('加载 %s ...\n', kinCache);
    load(kinCache);
else
    error('找不到 %s，请先运行五连杆运动学符号推导脚本。', kinCache);
end

if isfile(vmcCache)
    fprintf('加载 %s ...\n', vmcCache);
    load(vmcCache);
else
    error('找不到 %s，请先运行五连杆 VMC 符号推导脚本。', vmcCache);
end

syms dphi1 dphi4 real

if ~exist('dL0_sol', 'var')
    if exist('dL0_dphi1', 'var') && exist('dL0_dphi4', 'var')
        dL0_sol = simplify(dL0_dphi1*dphi1 + dL0_dphi4*dphi4);
    end
end

if ~exist('dphi0_sol', 'var')
    if exist('dphi0_dphi1', 'var') && exist('dphi0_dphi4', 'var')
        dphi0_sol = simplify(dphi0_dphi1*dphi1 + dphi0_dphi4*dphi4);
    end
end

fprintf('符号表达式加载完成。\n');
fprintf('\n========== 检查符号表达式 real/imag ==========\n');

exprNames = {};
exprList  = {};

% 运动学
if exist('L0_sol_f1', 'var')
    exprNames{end+1} = 'L0_sol_f1';
    exprList{end+1}  = L0_sol_f1;
end

if exist('phi0_sol_f2', 'var')
    exprNames{end+1} = 'phi0_sol_f2';
    exprList{end+1}  = phi0_sol_f2;
end

% VMC
if exist('T1_sol', 'var')
    exprNames{end+1} = 'T1_sol';
    exprList{end+1}  = T1_sol;
end

if exist('T4_sol', 'var')
    exprNames{end+1} = 'T4_sol';
    exprList{end+1}  = T4_sol;
end

% 速度映射
if exist('dL0_sol', 'var')
    exprNames{end+1} = 'dL0_sol';
    exprList{end+1}  = dL0_sol;
end

if exist('dphi0_sol', 'var')
    exprNames{end+1} = 'dphi0_sol';
    exprList{end+1}  = dphi0_sol;
end

if isempty(exprList)
    error('没有找到要检查的符号表达式。请先运行符号推导脚本。');
end

hasProblem = false;

for i = 1:numel(exprList)

    name = exprNames{i};
    expr = exprList{i};

    exprStr = char(expr);

    hasRealInSym = ~isempty(regexp(exprStr, '\<real\s*\(', 'once'));
    hasImagInSym = ~isempty(regexp(exprStr, '\<imag\s*\(', 'once'));

    % 再检查 ccode 生成结果里是否会出现 real/imag
    cCodeStr = getCCodeString(expr);

    hasRealInC = ~isempty(regexp(cCodeStr, '\<real\s*\(', 'once'));
    hasImagInC = ~isempty(regexp(cCodeStr, '\<imag\s*\(', 'once'));

    fprintf('\n[%s]\n', name);

    if hasRealInSym || hasImagInSym
        fprintf('  符号表达式：FAIL，含 real/imag\n');
        hasProblem = true;
    else
        fprintf('  符号表达式：OK，未发现 real/imag\n');
    end

    if hasRealInC || hasImagInC
        fprintf('  ccode 结果：FAIL，含 real/imag\n');
        hasProblem = true;
    else
        fprintf('  ccode 结果：OK，未发现 real/imag\n');
    end

    % 如果有问题，打印一小段定位
    if hasRealInSym || hasImagInSym
        fprintf('  符号表达式中出现位置附近：\n');
        printKeywordContext(exprStr, {'real(', 'imag('});
    end

    if hasRealInC || hasImagInC
        fprintf('  C code 中出现位置附近：\n');
        printKeywordContext(cCodeStr, {'real(', 'imag('});
    end
end

fprintf('\n========== 总结 ==========\n');

if hasProblem
    fprintf('结果：FAIL，有表达式混入 real/imag，不建议生成给 STM32 用的 C。\n');
else
    fprintf('结果：OK，未发现 real/imag，可以继续生成 C。\n');
end


% ============================================================
% 本地函数：获取 ccode 字符串
% ============================================================
function codeStr = getCCodeString(expr)

    tmpFile = [tempname, '.c'];

    ccode(expr, 'File', tmpFile);

    codeStr = fileread(tmpFile);

    delete(tmpFile);
end


% ============================================================
% 本地函数：打印关键词附近内容
% ============================================================
function printKeywordContext(str, keywords)

    for k = 1:numel(keywords)
        key = keywords{k};
        idx = strfind(str, key);

        if ~isempty(idx)
            for j = 1:min(numel(idx), 3)
                left  = max(1, idx(j) - 80);
                right = min(length(str), idx(j) + 120);

                snippet = str(left:right);
                fprintf('    ...%s...\n', snippet);
            end
        end
    end
end
%% 生成五连杆运动学和动力学两个数值函数
% 把由符号变量组成，建模推导整理成的符号表达式，变成实际的数值函数。
% 这个说是不建议直接转C做mcu运行代码，应该手写更加直观可靠

% 生成五连杆运动学数值函数
% 输入：
%   phi1, phi4
% 输出：
%   L0, phi0

matlabFunction( ...
    L0_sol_f1, phi0_sol_f2, ...
    'File', 'fivebar_kinematics_func', ...
    'Vars', {phi1, phi4}, ...
    'Outputs', {'L0', 'phi0'} ...
);


% 生成五连杆动力学/VMC数值函数
% 输入：
%   phi1, phi4, F, TP
% 输出：
%   T1, T4

matlabFunction( ...
    T1_sol, T4_sol, ...
    'File', 'fivebar_vmc_torque_func', ...
    'Vars', {phi1, phi4, F, TP}, ...
    'Outputs', {'T1', 'T4'} ...
);

% 生成五连杆速度映射数值函数
% 输入：
%   phi1, phi4, dphi1, dphi4
% 输出：
%   dL0, dphi0

syms dphi1 dphi4 real

dL0_sol = simplify(dL0_dphi1*dphi1 + dL0_dphi4*dphi4);

dphi0_sol = simplify(dphi0_dphi1*dphi1 + dphi0_dphi4*dphi4);

matlabFunction( ...
    dL0_sol, dphi0_sol, ...
    'File', 'fivebar_velocity_func', ...
    'Vars', {phi1, phi4, dphi1, dphi4}, ...
    'Outputs', {'dL0', 'dphi0'} ...
);

% 生成 theta 和 dtheta 数值函数
% 输入：
%   phi0, dphi0, phi, dphi
% 输出：
%   theta, dtheta

syms phi0_sym dphi0_sym body_phi body_dphi real

theta_sol_2  = simplify(phi0_sym - pi/2 - body_phi);

dtheta_sol_2 = simplify(dphi0_sym - body_dphi);

matlabFunction( ...
    theta_sol_2, dtheta_sol_2, ...
    'File', 'fivebar_theta_func', ...
    'Vars', {phi0_sym, dphi0_sym, body_phi, body_dphi}, ...
    'Outputs', {'theta', 'dtheta'} ...
);

%% 阶段性验证，主要验证五连杆动力学和运动学结算

% 验证1：画图检查五连杆几何
% 主要查看C点是在上面，也就是整个五连杆是一个凸起的状态不是反向
clear; clc; close all;

% 结构参数
LAE = 0.08;
l1  = 0.075;
l2  = 0.14;
l3  = l2;
l4  = l1;

% 测试角度
phi1_now = 2*pi/3;
phi4_now = pi/3;

% 基座点
A = [0; 0];
E = [LAE; 0];
O = [LAE/2; 0];

% B、D点
B = [l1*cos(phi1_now);
     l1*sin(phi1_now)];

D = [LAE + l4*cos(phi4_now);
     l4*sin(phi4_now)];

% 两圆交点求 C
BD = D - B;
dBD = norm(BD);

a = dBD/2;
h = sqrt(l2^2 - a^2);

u = BD / dBD;

branch = 1;

C = B + a*u + branch*h*[-u(2); u(1)];

% 画图
figure;
hold on; axis equal; grid on;

plot([A(1), B(1)], [A(2), B(2)], 'o-', 'LineWidth', 2);
plot([B(1), C(1)], [B(2), C(2)], 'o-', 'LineWidth', 2);
plot([E(1), D(1)], [E(2), D(2)], 'o-', 'LineWidth', 2);
plot([D(1), C(1)], [D(2), C(2)], 'o-', 'LineWidth', 2);
plot([O(1), C(1)], [O(2), C(2)], 'k--', 'LineWidth', 2);

text(A(1), A(2), ' A');
text(B(1), B(2), ' B');
text(C(1), C(2), ' C');
text(D(1), D(2), ' D');
text(E(1), E(2), ' E');
text(O(1), O(2), ' O');

xlabel('x / m');
ylabel('y / m');
title('Five-bar geometry check');
% 验证2：闭链距离检查

err_AB = norm(B - A) - l1;
err_ED = norm(D - E) - l4;
err_BC = norm(C - B) - l2;
err_DC = norm(C - D) - l3;

disp('闭链距离误差：');
disp(['err_AB = ', num2str(err_AB)]);
disp(['err_ED = ', num2str(err_ED)]);
disp(['err_BC = ', num2str(err_BC)]);
disp(['err_DC = ', num2str(err_DC)]);

% 验证3：对称姿态检查
% 有点像应试经典演算步骤：带一个特殊情况查看
phi1_now = 2*pi/3;
phi4_now = pi/3;
phi_now  = 0;

[L0_now, phi0_now] = fivebar_kinematics_func(phi1_now, phi4_now);

theta_now = phi0_now - pi/2 - phi_now;

disp('对称姿态检查：');
disp(['L0    = ', num2str(L0_now)]);
disp(['phi0  = ', num2str(phi0_now)]);
disp(['theta = ', num2str(theta_now)]);

disp(['phi0 - pi/2 = ', num2str(phi0_now - pi/2)]);

% 验证4：符号雅可比 vs 有限差分雅可比

% 加载符号雅可比
load('fivebar_vmc_symbolic.mat', 'J_vmc');

syms phi1 phi4 real

% 选一个非奇异测试点
phi1_now = 1.2;
phi4_now = 1.9;

% 符号雅可比数值化
J_sym = double(subs(J_vmc, [phi1, phi4], [phi1_now, phi4_now]));

% 有限差分
eps = 1e-6;

[L0_p1, phi0_p1] = fivebar_kinematics_func(phi1_now + eps, phi4_now);
[L0_m1, phi0_m1] = fivebar_kinematics_func(phi1_now - eps, phi4_now);

[L0_p4, phi0_p4] = fivebar_kinematics_func(phi1_now, phi4_now + eps);
[L0_m4, phi0_m4] = fivebar_kinematics_func(phi1_now, phi4_now - eps);

dL0_dphi1_fd   = (L0_p1   - L0_m1)   / (2*eps);
dphi0_dphi1_fd = (phi0_p1 - phi0_m1) / (2*eps);

dL0_dphi4_fd   = (L0_p4   - L0_m4)   / (2*eps);
dphi0_dphi4_fd = (phi0_p4 - phi0_m4) / (2*eps);

J_fd = [dL0_dphi1_fd,   dL0_dphi4_fd;
        dphi0_dphi1_fd, dphi0_dphi4_fd];

disp('J_sym = ');
disp(J_sym);

disp('J_fd = ');
disp(J_fd);

disp('J_sym - J_fd = ');
disp(J_sym - J_fd);
% 验证5：虚功等价检查

% 测试点
phi1_now = 1.2;
phi4_now = 1.9;

% 任意给定虚拟广义力
F_now  = 5.0;
TP_now = 0.2;

% 由你的 VMC 函数得到电机力矩
[T1_now, T4_now] = fivebar_vmc_torque_func(phi1_now, phi4_now, F_now, TP_now);

T_motor = [T1_now; T4_now];
Q_virtual = [F_now; TP_now];

% 使用前面算出来的 J_sym
J_sym = double(subs(J_vmc, [phi1, phi4], [phi1_now, phi4_now]));

% 给一个小虚位移
dq = [1e-4; -2e-4];

% 虚拟腿变量的小变化
dx_virtual = J_sym * dq;

work_motor = T_motor.' * dq;
work_virtual = Q_virtual.' * dx_virtual;
% 两边虚功

disp('虚功等价检查：');
disp(['work_motor  = ', num2str(work_motor)]);
disp(['work_virtual = ', num2str(work_virtual)]);
disp(['difference  = ', num2str(work_motor - work_virtual)]);
%% 仿真要用到的常数
M_L_support = 1.44/2 + 0.045*2
g = 9.8;


0.81*9.8/0.01



