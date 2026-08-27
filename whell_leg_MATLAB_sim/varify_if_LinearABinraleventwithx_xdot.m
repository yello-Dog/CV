%% 验证 A、B 是否和线性化点 x0, dx0 无关

syms x0 dx0 real

% 状态顺序：
% X = [theta; dtheta; x; dx; phi; dphi];

X0_general = [0; 0; x0; dx0; 0; 0];
U0 = [0; 0];

A_general = simplify(subs(A_lsym, [X; U], [X0_general; U0]));
B_general = simplify(subs(B_lsym, [X; U], [X0_general; U0]));

% 静止点
X0_static = [0; 0; 0; 0; 0; 0];

A_static = simplify(subs(A_lsym, [X; U], [X0_static; U0]));
B_static = simplify(subs(B_lsym, [X; U], [X0_static; U0]));

% 比较差值
A_diff = simplify(A_general - A_static);
B_diff = simplify(B_general - B_static);

disp('A_general - A_static = ');
disp(A_diff);

disp('B_general - B_static = ');
disp(B_diff);

%% 检查 A_general, B_general 里面是否还含有 x0, dx0
has_x0_in_A  = has(A_general, x0);
has_dx0_in_A = has(A_general, dx0);

has_x0_in_B  = has(B_general, x0);
has_dx0_in_B = has(B_general, dx0);

disp('A 是否含 x0: ');
disp(has_x0_in_A);

disp('A 是否含 dx0: ');
disp(has_dx0_in_A);

disp('B 是否含 x0: ');
disp(has_x0_in_B);

disp('B 是否含 dx0: ');
disp(has_dx0_in_B);

%% 最终判断
if isequal(A_diff, sym(zeros(size(A_diff)))) && isequal(B_diff, sym(zeros(size(B_diff))))
    disp('验证通过：A、B 与线性化点 x0、dx0 无关。');
else
    disp('验证不通过：A、B 至少有一部分和 x0 或 dx0 有关。');
end