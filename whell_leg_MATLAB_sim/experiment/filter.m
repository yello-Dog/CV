%% 一阶低通滤波
N = 100;          % filter coefficient, rad/s
s = tf('s');

H = N/(s+N);

figure;
bode(H);
grid on;
title('Low-pass Filter: N/(s+N)');

%% D项加上一阶低通滤波
Kd = 1000;
N = 100;
s = tf('s');

D_filter = Kd * (N*s)/(s + N);

figure;
bode(D_filter);
grid on;
title('Filtered Derivative Term: Kd*N*s/(s+N)');

%% 下面研究怎么设计轮足控制器的D后面滤波器截止频率
% 读取 dL0 timeseries
ts = out.dL0;

t = ts.Time;
x = ts.Data;
x = x(:);

% 去掉前面接触/启动瞬态
t_start = 0.2;   % 你可以改成 0.2 或 0.5
idx = t >= t_start;

t = t(idx);
x = x(idx);

% 重采样成等间隔
Fs = 1000;       % 采样频率，假设你控制器 1 ms，则 Fs=1000 Hz
Ts = 1/Fs;

t_uniform = t(1):Ts:t(end);
x_uniform = interp1(t, x, t_uniform, 'linear');

% 去直流/趋势
x_uniform = x_uniform - mean(x_uniform);

% FFT
N = length(x_uniform);
Y = fft(x_uniform);

P2 = abs(Y / N);
P1 = P2(1:floor(N/2)+1);
P1(2:end-1) = 2 * P1(2:end-1);

f = Fs * (0:floor(N/2)) / N;

% 画频谱
figure;
plot(f, P1, 'LineWidth', 1.2);
grid on;
xlabel('Frequency (Hz)');
ylabel('|dL0| amplitude');
title('Spectrum of dL0');
xlim([0 200]);   % 先看 0~200 Hz
