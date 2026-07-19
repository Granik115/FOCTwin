function foctwin_tune(requestPath, resultPath)
%FOCTWIN_TUNE Bounded surrogate tuning with native MATLAB checkpoint resume.

startedAt = datetime('now', 'TimeZone', 'UTC');
try
    request = jsondecode(fileread(requestPath));
    [baseConfig, metric, parameters, optimizer] = parseRequest(request, requestPath);
    [lowerBounds, upperBounds] = parameterBounds(parameters);

    options = optimoptions('surrogateopt', ...
        'MaxFunctionEvaluations', optimizer.max_evaluations, ...
        'MinSurrogatePoints', optimizer.min_surrogate_points, ...
        'CheckpointFile', optimizer.checkpoint_file, ...
        'Display', 'iter', ...
        'PlotFcn', [], ...
        'UseParallel', logical(optimizer.use_parallel));

    if optimizer.resume && isfile(optimizer.checkpoint_file)
        [xBest, fBest, exitFlag, output, trials] = ...
            surrogateopt(optimizer.checkpoint_file, options);
        resumed = true;
    else
        objective = @(x) evaluateCandidate(x, baseConfig, parameters, metric);
        [xBest, fBest, exitFlag, output, trials] = ...
            surrogateopt(objective, lowerBounds, upperBounds, options);
        resumed = false;
    end

    result.success = true;
    result.metric = metric;
    result.resumed = resumed;
    result.checkpoint_file = optimizer.checkpoint_file;
    result.best.value = fBest;
    result.best.parameters = namedValues(parameters, xBest);
    result.top = topCandidates(trials, parameters, optimizer.top_count);
    result.exit_flag = exitFlag;
    result.optimizer.evaluations = outputField(output, 'funccount', NaN);
    result.optimizer.message = char(string(outputField(output, 'message', '')));
catch exception
    result.success = false;
    result.error.identifier = exception.identifier;
    result.error.message = exception.message;
    result.error.report = getReport(exception, 'extended', 'hyperlinks', 'off');
end

result.started_at = char(startedAt, 'yyyy-MM-dd''T''HH:mm:ss.SSSXXX');
result.finished_at = char(datetime('now', 'TimeZone', 'UTC'), 'yyyy-MM-dd''T''HH:mm:ss.SSSXXX');
foctwin_write_json(resultPath, result);
end

function [config, metric, parameters, optimizer] = parseRequest(request, requestPath)
defaults = foctwin_default_config();
if isfield(request, 'simulation')
    config = foctwin_merge_structs(defaults, request.simulation);
else
    config = defaults;
end

if isfield(request, 'metric')
    metric = char(request.metric);
else
    metric = 'IAE_final';
end
if ~isfield(request, 'parameters') || isempty(request.parameters)
    error('FOCTwin:NoParameters', 'At least one tuning parameter is required.');
end
parameters = request.parameters;
if ~all(isfield(parameters, {'path', 'lower', 'upper'}))
    error('FOCTwin:InvalidParameters', 'Each parameter needs path, lower and upper fields.');
end

optimizer.max_evaluations = 60;
optimizer.min_surrogate_points = 15;
optimizer.use_parallel = false;
optimizer.resume = false;
optimizer.top_count = 5;
[requestDirectory, ~, ~] = fileparts(requestPath);
if isempty(requestDirectory)
    requestDirectory = pwd;
end
optimizer.checkpoint_file = fullfile(requestDirectory, 'surrogate-checkpoint.mat');
if isfield(request, 'optimizer')
    optimizer = foctwin_merge_structs(optimizer, request.optimizer);
end
optimizer.checkpoint_file = char(optimizer.checkpoint_file);
end

function [lowerBounds, upperBounds] = parameterBounds(parameters)
count = numel(parameters);
lowerBounds = zeros(1, count);
upperBounds = zeros(1, count);
for index = 1:count
    lowerBounds(index) = double(parameters(index).lower);
    upperBounds(index) = double(parameters(index).upper);
    if ~isfinite(lowerBounds(index)) || ~isfinite(upperBounds(index)) || ...
            lowerBounds(index) >= upperBounds(index)
        error('FOCTwin:InvalidBounds', 'Invalid bounds for %s.', parameters(index).path);
    end
end
end

function cost = evaluateCandidate(x, baseConfig, parameters, metric)
penalty = 1e12;
try
    config = baseConfig;
    for index = 1:numel(parameters)
        config = foctwin_set_path(config, parameters(index).path, x(index));
    end
    simulation = foctwin_run_simulation(config);
    if ~simulation.success || ~isfield(simulation.metrics, metric)
        cost = penalty;
        return;
    end
    cost = double(simulation.metrics.(metric));
    if ~isscalar(cost) || ~isfinite(cost)
        cost = penalty;
    end
catch exception
    warning('FOCTwin:TuningCandidateFailed', '%s', ...
        getReport(exception, 'basic', 'hyperlinks', 'off'));
    cost = penalty;
end
end

function values = namedValues(parameters, x)
values = repmat(struct('path', '', 'value', 0), 1, numel(parameters));
for index = 1:numel(parameters)
    values(index).path = char(parameters(index).path);
    values(index).value = double(x(index));
end
end

function candidates = topCandidates(trials, parameters, count)
[xValues, fValues] = trialValues(trials);
if isempty(fValues)
    candidates = struct('rank', {}, 'value', {}, 'parameters', {});
    return;
end

valid = isfinite(fValues);
xValues = xValues(valid, :);
fValues = fValues(valid);
[sortedValues, order] = sort(fValues, 'ascend');
limit = min(double(count), numel(order));
candidates = repmat(struct('rank', 0, 'value', 0, 'parameters', []), 1, limit);
for index = 1:limit
    candidates(index).rank = index;
    candidates(index).value = double(sortedValues(index));
    candidates(index).parameters = namedValues(parameters, xValues(order(index), :));
end
end

function [xValues, fValues] = trialValues(trials)
xValues = [];
fValues = [];
if istable(trials)
    names = trials.Properties.VariableNames;
    if any(strcmp(names, 'X')) && any(strcmp(names, 'Fval'))
        xValues = trials.X;
        fValues = trials.Fval;
    end
elseif isstruct(trials) && isfield(trials, 'X') && isfield(trials, 'Fval')
    xValues = trials.X;
    fValues = trials.Fval;
end
fValues = double(fValues(:));
xValues = double(xValues);
end

function value = outputField(output, name, fallback)
if isstruct(output) && isfield(output, name)
    value = output.(name);
else
    value = fallback;
end
end
