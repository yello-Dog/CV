

N_filter = 300
%% ===================== 基本设置 =====================
model = 'wheel_leg';

N_list = [100 120 150 200 300 400 500 600 700 800 900 1000];
StopTime = '10';

% 频谱设置
Fs = 1000;        % 重采样频率，按 1 ms 控制周期
t_start = 0.5;    % 去掉前 0.5s 接触/启动瞬态
f_max = 250;      % 频谱显示到 250 Hz

% 是否保存每张图
save_figures = false;

if ~bdIsLoaded(model)
    load_system(model);
end

set_param(model, 'StopTime', StopTime);

% 关闭 Simscape logging，减少拖慢
try
    set_param(model, 'SimscapeLogType', 'none');
catch
end

%% ===================== 自动仿真 =====================
results = struct();

for i = 1:length(N_list)

    N_filter = N_list(i);
    fc = N_filter / (2*pi);

    fprintf('\n========== Running N = %g rad/s, fc = %.2f Hz ==========\n', ...
        N_filter, fc);

    simIn = Simulink.SimulationInput(model);

    % 给 Simulink 模型传入滤波器系数
    simIn = simIn.setVariable('N_filter', N_filter);
    simIn = simIn.setModelParameter('StopTime', StopTime);

    % 跑仿真
    out = sim(simIn);

    % 保存本次结果
    results(i).N  = N_filter;
    results(i).fc = fc;

    results(i).dL0_raw      = get_ts(out, 'dL0_raw');
    results(i).dL0_filterd  = get_ts(out, 'dL0_filterd');
    results(i).F_command    = get_ts(out, 'F_command_out');
    results(i).L0           = get_ts(out, 'L0');

    % 新增：目标速度和实际速度
    results(i).dxd          = get_ts(out, 'dxd');   % target velocity
    results(i).dx           = get_ts(out, 'dx');    % actual velocity

end

%% ===================== 保存所有数据 =====================
save('N_sweep_results.mat', 'results', 'N_list');

%% ===================== 每个 N 单独画一个窗口 =====================
for i = 1:length(results)

    N_filter = results(i).N;
    fc = results(i).fc;

    ts_raw  = results(i).dL0_raw;
    ts_filt = results(i).dL0_filterd;
    ts_F    = results(i).F_command;
    ts_L0   = results(i).L0;
    ts_dxd  = results(i).dxd;
    ts_dx   = results(i).dx;

    % dL0 raw 和 filterd 的频谱
    [f_raw, A_raw]   = get_spectrum_from_timeseries(ts_raw,  Fs, t_start);
    [f_filt, A_filt] = get_spectrum_from_timeseries(ts_filt, Fs, t_start);

    % 速度目标和实际速度对齐
    [t_v, dxd_data, dx_data] = align_two_timeseries(ts_dxd, ts_dx, Fs);
    e_dx = dxd_data - dx_data;

    % 新建窗口
    fig = figure('Name', sprintf('N = %g, fc = %.2f Hz', N_filter, fc), ...
                 'Color', 'w');

    tiledlayout(3, 2, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    %% ===== 1. dL0 raw 频谱 =====
    nexttile;
    plot(f_raw, A_raw, 'LineWidth', 1.0);
    grid on;
    xlim([0 f_max]);
    xlabel('Frequency (Hz)');
    ylabel('|dL0 raw|');
    title('dL0 raw spectrum');

    %% ===== 2. dL0 filterd 频谱 =====
    nexttile;
    plot(f_filt, A_filt, 'LineWidth', 1.0);
    grid on;
    xlim([0 f_max]);
    xlabel('Frequency (Hz)');
    ylabel('|dL0 filterd|');
    title('dL0 filterd spectrum');

    %% ===== 3. F command out 时域 =====
    nexttile;
    plot(ts_F.Time, squeeze(ts_F.Data), 'LineWidth', 1.0);
    grid on;
    xlabel('Time (s)');
    ylabel('F');
    title('F command out');

    %% ===== 4. L0 时域 =====
    nexttile;
    plot(ts_L0.Time, squeeze(ts_L0.Data), 'LineWidth', 1.0);
    grid on;
    xlabel('Time (s)');
    ylabel('L0 (m)');
    title('L0');

    %% ===== 5. 目标速度 vs 实际速度 =====
    nexttile;
    plot(t_v, dxd_data, 'LineWidth', 1.2);
    hold on;
    plot(t_v, dx_data, 'LineWidth', 1.0);
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('dx\_d', 'dx', 'Location', 'best');
    title('Target velocity vs actual velocity');

    %% ===== 6. 速度误差 =====
    nexttile;
    plot(t_v, e_dx, 'LineWidth', 1.0);
    grid on;
    xlabel('Time (s)');
    ylabel('dx\_d - dx');
    title('Velocity tracking error');

    sgtitle(sprintf('N = %g rad/s, fc = %.2f Hz', N_filter, fc));

    if save_figures
        filename = sprintf('N_%g_fc_%.2fHz.png', N_filter, fc);
        exportgraphics(fig, filename, 'Resolution', 200);
    end

end

%% ===================== 生成简单指标表 =====================
metrics = table();

for i = 1:length(results)

    N_filter = results(i).N;
    fc = results(i).fc;

    ts_F   = results(i).F_command;
    ts_L0  = results(i).L0;
    ts_dxd = results(i).dxd;
    ts_dx  = results(i).dx;

    % 对齐速度
    [t_v, dxd_data, dx_data] = align_two_timeseries(ts_dxd, ts_dx, Fs);
    e_dx = dxd_data - dx_data;

    % F 指标
    F_data = squeeze(ts_F.Data);
    F_data = F_data(:);

    % L0 指标
    L0_data = squeeze(ts_L0.Data);
    L0_data = L0_data(:);

    metrics.N(i,1) = N_filter;
    metrics.fc_Hz(i,1) = fc;
    metrics.F_peak(i,1) = max(abs(F_data));
    metrics.F_rms(i,1) = rms(F_data);
    metrics.dx_error_peak(i,1) = max(abs(e_dx));
    metrics.dx_error_rms(i,1) = rms(e_dx);
    metrics.L0_min(i,1) = min(L0_data);
    metrics.L0_max(i,1) = max(L0_data);

end

disp(metrics);
save('N_sweep_metrics.mat', 'metrics');

%% ===================== 局部函数：从 out 里取 timeseries =====================
function ts = get_ts(out, name)

    % Single simulation output 下 out.name
    try
        ts = out.(name);
        return;
    catch
    end

    % logsout 情况
    try
        sig = out.logsout.get(name);
        ts = sig.Values;
        return;
    catch
    end

    error('找不到信号 "%s"。请检查 To Workspace 的 Variable name 是否完全一致。', name);

end

%% ===================== 局部函数：timeseries 转频谱 =====================
function [f, A] = get_spectrum_from_timeseries(ts, Fs, t_start)

    t = ts.Time;
    x = squeeze(ts.Data);
    x = x(:);

    % 去掉启动阶段
    idx = t >= t_start;
    t = t(idx);
    x = x(idx);

    % 防止时间点重复
    [t, unique_idx] = unique(t, 'stable');
    x = x(unique_idx);

    % 重采样成等间隔
    Ts = 1/Fs;
    t_uniform = t(1):Ts:t(end);
    x_uniform = interp1(t, x, t_uniform, 'linear', 'extrap');

    % 去趋势，避免 0Hz 附近太大
    x_uniform = detrend(x_uniform);
    x_uniform = x_uniform - mean(x_uniform);

    % FFT
    N = length(x_uniform);
    Y = fft(x_uniform);

    P2 = abs(Y / N);
    A = P2(1:floor(N/2)+1);
    A(2:end-1) = 2 * A(2:end-1);

    f = Fs * (0:floor(N/2)) / N;

end

%% ===================== 局部函数：两个 timeseries 对齐 =====================
function [t_uniform, y1, y2] = align_two_timeseries(ts1, ts2, Fs)

    t1 = ts1.Time;
    x1 = squeeze(ts1.Data);
    x1 = x1(:);

    t2 = ts2.Time;
    x2 = squeeze(ts2.Data);
    x2 = x2(:);

    [t1, idx1] = unique(t1, 'stable');
    x1 = x1(idx1);

    [t2, idx2] = unique(t2, 'stable');
    x2 = x2(idx2);

    t_start = max(t1(1), t2(1));
    t_end   = min(t1(end), t2(end));

    Ts = 1/Fs;
    t_uniform = t_start:Ts:t_end;

    y1 = interp1(t1, x1, t_uniform, 'linear', 'extrap');
    y2 = interp1(t2, x2, t_uniform, 'linear', 'extrap');

end