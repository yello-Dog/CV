function K = get_k_length(leg_length)
    %% 1. 读取之前缓存的符号线性化结果
    persistent coeff

    if isempty(coeff)
        data = load('wheel_leg_dynamic_cache.mat', 'A_0', 'B_0');
    end
    A_0 = data.A_0;
    B_0 = data.B_0;

    %% 2. 在函数内部重新定义符号参数
    
    %定义QR矩阵
    Q=diag([1 0.5 30 15 300 0.6]);
    R=[8 0;0 1];
    
    %定义系统参数
    syms Rw Iw mw M L LM ell mp g Ip IM real
    params = [Rw, Iw, mw, M, L, LM, ell, mp, g, Ip, IM];
    values = [ ...
        0.0603, ...                                           % Rw
        0.5 * 0.622 * 0.0603^2, ...                              % Iw
        0.22, ...                                               % mw
        2.027/2, ...                                              % M
        leg_length/2, ...                                      % L
        leg_length/2, ...                                      % LM
        0.011, ...                                             % ell
        0.075*2, ...                                             % mp
        9.8, ...                                               % g
        0.075*2 * (leg_length^2 + 0.048^2) / 12.0, ...           % Ip
        2.027/2 * (0.18^2 + 0.066^2) / 12.0 ...                 % IM
    ];
    % 这个subs函数是变量名字一样就能替换
    A = double(subs(A_0, params, values));
    B = double(subs(B_0, params, values));
   
    % 15. 检查可控性
    Co = ctrb(A, B);
    rank_Co = rank(Co);
    if rank_Co == 6
        disp('此系统可控');
    end
    
    % 16. 解连续时间 Riccati 方程
    K=lqr(A,B,Q,R);
end