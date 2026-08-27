%% 开始写仿真
%到目前为止，有三个函数
%1.给腿长就给你K矩阵：get_k_cubic_length（轮足参数已经包含进了这个函数里）
%2.给fai1 fai4 就给你L0与fai0：fivebar_kinematics_func（五连杆参数已经包含进了这个函数里）
%3.给TP F就给你T1 T4：fivebar_vmc_torque_func（五连杆参数已经包含进了这个函数里）
%还有一个动力学方程Xdot，暂时没有转化成关于X和U的函数

%下面定义变量：
%-------------------状态变量：X = [theta; dtheta; x; dx; phi; dphi];
% X = [theta; dtheta; x; dx; phi; dphi]

theta  = 0;      % 虚拟腿摆角
dtheta = 0;      % 虚拟腿摆角速度

x      = 0;      % 轮轴/轮心水平位置
dx     = 0;      % 轮轴/轮心水平速度

phi    = 0;      % 机体 pitch 角
dphi   = 0;      % 机体 pitch 角速度

%-------------------输入U：U = [T; Tp];
% Xd = [theta_d; dtheta_d; x_d; dx_d; phi_d; dphi_d]

theta_d  = 0;
dtheta_d = 0;

x_d      = 0;
dx_d     = 0;

phi_d    = 0;
dphi_d   = 0;

%-------------------反馈矩阵K：U = -K(X - Xd)；Xd为参考
% U = [T; Tp]

T  = 0;          % 轮毂电机等效力矩
Tp = 0;          % 虚拟腿角度方向等效力矩

%五连杆等效参数
L0 = 0;
fai0 = pi/2;
F = 0;

%-------------------可以读取的电机参数: 先搞一边的，另一边直接复制这一边 最简验证就这样做
fai1 = 0;
dfai1 = 0;
faiw = 0;
dfai2 = 0;
fai4 = 0;
dfai4 = 0;

%------------------轮足总重量: 单边的，单位是KG
M_oneside = 0.6 + 1.44 + 0.045;
g = 9.8;

%先计算腿长
[L0,fai0] = fivebar_kinematics_func(fai1,fai4);
%然后根据腿长获得增益
K = get_k_cubic_length(L0);
%然后根据增益获得U
X = [theta; dtheta; x; dx; phi; dphi];
Xd = [theta_d; dtheta_d; x_d; dx_d; phi_d; dphi_d];
U = -K*(X - Xd);
T = U(1);
Tp = U(2);
%根据目前的姿态计算F：目前只考虑前馈
F = M_oneside*g/cos(theta);
%然后根据五连杆动力学获得T1 T4
T14 = fivebar_vmc_torque_func(fai1,fai4,F,Tp);
T1 = T14(1);
T4 = T14(4);