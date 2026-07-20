%% =====================================================
%%  АВТОМАТИЧЕСКИЙ ТЮНИНГ PID (Surrogate Optimization)
%%  Упрощённая и надёжная версия
%%  - Убраны искусственные штрафы за большое IAE
%%  - IAE_final возвращается "как есть" (даже если 5000+)
%%  - Оставлены только защиты от вылетов и утечек памяти
%% =====================================================

clear; clc; close all;

%% ==================== НАСТРОЙКИ ====================

test_mode = true;           % true = быстро (60 итераций), false = полный прогон

modelName      = 'AutoPID_FOC_Model_Current';
stopTime       = 3;
metricSignal   = 'IAE_final';   % <-- что оптимизируем (IAE)

% === Параметры для тюнинга ===
paramsToTune = {
    'angle_P',     1,   200,   35;
%      'angle_I',     0,   500,   0;
%     'PID_vel_P',   1,   50,   20;
%     'PID_vel_I',   0, 3000,  400;
%     'PID_vel_D',   0,    5, 0.01;
%     'PID_curr_P', 0.1,   15,    3;
%     'PID_curr_I',  0, 5000,  300;
};

numParams  = size(paramsToTune, 1);
lb         = cell2mat(paramsToTune(:,2))';
ub         = cell2mat(paramsToTune(:,3))';
paramNames = paramsToTune(:,1);

%% ==================== НАСТРОЙКИ ОПТИМИЗАТОРА ====================

if test_mode
    maxEvals     = 60;
    minSurrogate = 15;
    fprintf('>>> РЕЖИМ: ТЕСТОВЫЙ (быстрый)\n\n');
else
    maxEvals     = 400;
    minSurrogate = 40;
    fprintf('>>> РЕЖИМ: ПОЛНЫЙ (400 итераций)\n\n');
end

options = optimoptions('surrogateopt', ...
    'MaxFunctionEvaluations', maxEvals, ...
    'MinSurrogatePoints',     minSurrogate, ...
    'Display',                'iter', ...
    'PlotFcn',                [], ...
    'UseParallel',            false);

%% ==================== ПРОДОЛЖЕНИЕ (RESUME) ====================

checkpointFile = 'surrogate_tuning_checkpoint.mat';

InitialPoints = struct('X', [], 'Fval', []);

if exist(checkpointFile, 'file')
    fprintf('Найден файл продолжения. Загружаю предыдущие результаты...\n');
    load(checkpointFile, 'X_all', 'Fval_all', '-mat');
    
    if exist('X_all','var') && ~isempty(X_all)
        InitialPoints.X    = X_all;
        InitialPoints.Fval = Fval_all;
        fprintf('Загружено %d ранее оценённых точек.\n\n', size(X_all, 1));
    end
else
    fprintf('Начинаем новый запуск.\n\n');
end

%% ==================== ЗАПУСК ОПТИМИЗАЦИИ ====================

costFcn = @(x) costFunction(x, paramNames, modelName, stopTime, metricSignal);

if ~isempty(InitialPoints.X)
    [x_best, fval_best, ~, output] = surrogateopt(costFcn, lb, ub, options, InitialPoints);
else
    [x_best, fval_best, ~, output] = surrogateopt(costFcn, lb, ub, options);
end

%% ==================== СОХРАНЕНИЕ РЕЗУЛЬТАТОВ ====================

% Надёжное объединение старых + новых точек
if ~isempty(InitialPoints.X)
    if isfield(output, 'X') && ~isempty(output.X)
        X_all    = [InitialPoints.X;    output.X];
        Fval_all = [InitialPoints.Fval; output.Fval];
    else
        X_all    = InitialPoints.X;
        Fval_all = InitialPoints.Fval;
    end
else
    if isfield(output, 'X') && ~isempty(output.X)
        X_all    = output.X;
        Fval_all = output.Fval;
    else
        X_all    = [];
        Fval_all = [];
    end
end

save(checkpointFile, 'X_all', 'Fval_all', 'paramNames', 'lb', 'ub', '-v7.3');

fprintf('\n============================================\n');
fprintf('           РЕЗУЛЬТАТЫ ТЮНИНГА\n');
fprintf('============================================\n');
fprintf('Лучшее значение %s = %.4f\n\n', metricSignal, fval_best);
fprintf('Оптимальные параметры:\n');
for i = 1:numParams
    fprintf('  %-12s = %10.4f\n', paramNames{i}, x_best(i));
end
fprintf('\nВсего оценено точек: %d\n', size(X_all, 1));
fprintf('Чекпоинт сохранён: %s\n', checkpointFile);

%% ==================== ФУНКЦИЯ СТОИМОСТИ ====================

function cost = costFunction(x, paramNames, modelName, stopTime, metricSignal)
    % Каждый раз заново инициализируем параметры модели
    evalin('base', 'run(''init_params3.m'')');
    
    % Присваиваем новые значения параметров тюнинга
    for i = 1:length(paramNames)
        assignin('base', paramNames{i}, x(i));
    end
    
    try
        simOut = sim(modelName, ...
            'StopTime',     num2str(stopTime), ...
            'SrcWorkspace', 'base', ...
            'FastRestart',  'off');
        
        % === Получаем метрику IAE_final ===
        if isprop(simOut, metricSignal)
            val = simOut.(metricSignal);
            if isa(val, 'timeseries')
                metricValue = val.Data(end);
            else
                metricValue = val;
            end
        else
            warning('Сигнал %s не найден в simOut!', metricSignal);
            metricValue = NaN;
        end
        
        % Возвращаем значение "как есть".
        % Только если NaN / Inf / пусто — ставим большой штраф,
        % чтобы оптимизатор понимал, что точка плохая.
        if isnan(metricValue) || isinf(metricValue) || isempty(metricValue)
            cost = 1e9;                    % большой штраф при ошибке
        else
            cost = metricValue;            % <-- ЧИСТОЕ значение IAE (даже 3000+)
        end
        
    catch ME
        warning('Ошибка симуляции:\n%s', getReport(ME, 'basic'));
        cost = 1e9;                        % штраф при краше
    end
    
    % === ЖЁСТКАЯ ОЧИСТКА ПАМЯТИ (важно при 100+ итерациях) ===
    if exist('simOut', 'var')
        simOut = [];
    end
    clear simOut;
    evalin('base', 'clear simOut tout xout yout;');
end