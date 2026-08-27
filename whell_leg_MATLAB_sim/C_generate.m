%% ---------- 自动加载五连杆符号表达式 ----------
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

%% ---------- 生成速度映射表达式 ----------
syms dphi1 dphi4 real

if ~exist('dL0_sol', 'var')
    if exist('dL0_dphi1', 'var') && exist('dL0_dphi4', 'var')
        dL0_sol = simplify(dL0_dphi1*dphi1 + dL0_dphi4*dphi4);
    else
        error('找不到 dL0_dphi1 / dL0_dphi4，无法生成 dL0_sol。');
    end
end

if ~exist('dphi0_sol', 'var')
    if exist('dphi0_dphi1', 'var') && exist('dphi0_dphi4', 'var')
        dphi0_sol = simplify(dphi0_dphi1*dphi1 + dphi0_dphi4*dphi4);
    else
        error('找不到 dphi0_dphi1 / dphi0_dphi4，无法生成 dphi0_sol。');
    end
end

fprintf('符号表达式加载完成，可以开始生成 C。\n');

%% generate_fivebar_c_stm32.m
% 直接把五连杆符号表达式生成纯 C 函数
% 适合 STM32H7 / arm-none-eabi-gcc
%
% 前提：工作区已经有：
%   L0_sol_f1, phi0_sol_f2
%   T1_sol, T4_sol
%   dL0_sol, dphi0_sol

outDir = "fivebar_c_stm32";

if ~exist(outDir, "dir")
    mkdir(outDir);
end

%% 1. 运动学
writeScalarCFunction_COnly( ...
    outDir, ...
    "fivebar_kinematics_func", ...
    ["phi1", "phi4"], ...
    ["L0", "phi0"], ...
    [L0_sol_f1, phi0_sol_f2] ...
);

%% 2. VMC 力矩
writeScalarCFunction_COnly( ...
    outDir, ...
    "fivebar_vmc_torque_func", ...
    ["phi1", "phi4", "F", "TP"], ...
    ["T1", "T4"], ...
    [T1_sol, T4_sol] ...
);

%% 3. 速度映射
writeScalarCFunction_COnly( ...
    outDir, ...
    "fivebar_velocity_func", ...
    ["phi1", "phi4", "dphi1", "dphi4"], ...
    ["dL0", "dphi0"], ...
    [dL0_sol, dphi0_sol] ...
);

fprintf("\nC 文件生成完成，路径：%s\n", outDir);


%% ============================================================
% 纯 C 函数生成器
% ============================================================
function writeScalarCFunction_COnly(outDir, funcName, inputNames, outputNames, exprs)

    funcName    = string(funcName);
    inputNames  = string(inputNames);
    outputNames = string(outputNames);

    cFile = fullfile(outDir, funcName + ".c");
    hFile = fullfile(outDir, funcName + ".h");

    if numel(outputNames) ~= numel(exprs)
        error("outputNames 和 exprs 数量不一致");
    end

    guardName = upper(char(funcName)) + "_H";
    guardName = regexprep(guardName, '[^A-Z0-9_]', '_');

    proto = makePrototype_COnly(funcName, inputNames, outputNames);

    %% ---------- 写 .h ----------
    fid = fopen(hFile, "w");
    if fid < 0
        error("无法创建头文件: %s", hFile);
    end

    fprintf(fid, '#ifndef %s\n', guardName);
    fprintf(fid, '#define %s\n\n', guardName);

    % 纯 C：这里故意不写 extern "C"

    fprintf(fid, '%s;\n\n', char(proto));

    fprintf(fid, '#endif\n');

    fclose(fid);

    %% ---------- 写 .c ----------
    fid = fopen(cFile, "w");
    if fid < 0
        error("无法创建源文件: %s", cFile);
    end

    fprintf(fid, '#include <math.h>\n');
    fprintf(fid, '#include "%s.h"\n\n', char(funcName));

    % STM32 工程里有些 math.h 不定义 M_PI，兜底一下
    fprintf(fid, '#ifndef M_PI\n');
    fprintf(fid, '#define M_PI 3.14159265358979323846\n');
    fprintf(fid, '#endif\n\n');

    fprintf(fid, '%s\n', char(proto));
    fprintf(fid, '{\n');

    for k = 1:numel(exprs)

        expr = exprs(k);

        %% ---------- 先检查符号表达式本身 ----------
        exprStr = char(expr);

        if contains(exprStr, 'real(') || contains(exprStr, 'imag(')
            fclose(fid);
            error("表达式 %s 里含 real/imag。先修符号推导，不生成 C。", outputNames(k));
        end

        %% ---------- 生成 C code fragment ----------
        code = getCCodeFragment(expr);
        codeChar = char(code);

        %% ---------- 再检查生成后的 C ----------
        if contains(code, 'real(') || contains(code, 'imag(') || ...
           contains(code, 'creal') || contains(code, 'cimag')
            fclose(fid);
            error("表达式 %s 的 C 代码含 real/imag/complex。先修符号推导，不生成 C。", outputNames(k));
        end


%% ---------- 提取所有被赋值的临时变量，并声明 ----------
assignVars = regexp(codeChar, '(?m)^\s*([A-Za-z_]\w*)\s*=', 'tokens');

if isempty(assignVars)
    fclose(fid);
    error("表达式 %s 没有找到赋值变量，无法生成 C。", outputNames(k));
end

tVars = strings(1, numel(assignVars));
for ii = 1:numel(assignVars)
    tVars(ii) = string(assignVars{ii}{1});
end

tVars = unique(tVars, "stable");

fprintf(fid, '    {\n');
fprintf(fid, '        double ');

for ii = 1:numel(tVars)
    if ii < numel(tVars)
        fprintf(fid, '%s, ', char(tVars(ii)));
    else
        fprintf(fid, '%s', char(tVars(ii)));
    end
end

fprintf(fid, ';\n');

        %% ---------- 写入 ccode 生成的计算语句 ----------
        codeLines = splitlines(code);

        for i = 1:numel(codeLines)
            line = strtrim(codeLines(i));

            if strlength(line) == 0
                continue;
            end

            fprintf(fid, '        %s\n', char(line));
        end

        %% ---------- 把 t0 写到输出指针 ----------
        fprintf(fid, '        *%s = t0;\n', char(outputNames(k)));
        fprintf(fid, '    }\n\n');
    end

    fprintf(fid, '}\n');

    fclose(fid);
end


%% ============================================================
% 生成纯 C 函数原型
% ============================================================
function proto = makePrototype_COnly(funcName, inputNames, outputNames)

    args = strings(1, numel(inputNames) + numel(outputNames));

    for i = 1:numel(inputNames)
        args(i) = "double " + inputNames(i);
    end

    for i = 1:numel(outputNames)
        args(numel(inputNames) + i) = "double *" + outputNames(i);
    end

    proto = "void " + funcName + "(" + strjoin(args, ", ") + ")";
end


%% ============================================================
% 用 ccode 生成 C 片段
% ============================================================
function code = getCCodeFragment(expr)

    tmpFile = [tempname, '.c'];

    ccode(expr, 'File', tmpFile);

    code = fileread(tmpFile);

    delete(tmpFile);

    code = string(code);
end