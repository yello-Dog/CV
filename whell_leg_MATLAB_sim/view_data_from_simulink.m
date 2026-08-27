%命令行窗口保存to workspace的仿真数据的代码
%L0_MATLAB_Nofilter = out.L0;
%F_MATLAB_Nofilter  = out.F_command_out;

%save('run_MATLAB_Nofilter.mat', ...
%     'L0_MATLAB_Nofilter', ...
%     'F_MATLAB_Nofilter');

load('run_jaccobian_Nofilter.mat');
load('run_MATLAB_with_filter.mat');

figure;
plot(L0_jaccobian_Nofilter.Time, L0_jaccobian_Nofilter.Data, 'LineWidth', 1.5);
hold on;
plot(L0_MATLAB_with_filter.Time, L0_MATLAB_with_filter.Data, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('L0 (m)');
legend('My PD', 'MATLAB Controller');
title('L0 Comparison');

figure;
plot(F_jaccobian_Nofilter.Time, F_jaccobian_Nofilter.Data, 'LineWidth', 1.5);
hold on;
plot(F_MATLAB_with_filter.Time, F_MATLAB_with_filter.Data, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Controller Output F');
legend('My PD', 'MATLAB Controller');
title('Controller Output Comparison');
