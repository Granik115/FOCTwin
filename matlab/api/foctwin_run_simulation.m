function result = foctwin_run_simulation(config)
%FOCTWIN_RUN_SIMULATION Run one model without mutating MATLAB base workspace.

if strcmpi(config.mode, 'current')
    modelName = 'AutoPID_FOC_Model_Current';
elseif strcmpi(config.mode, 'voltage')
    modelName = 'AutoPID_FOC_Model_Voltage';
else
    error('FOCTwin:InvalidMode', 'Unsupported simulation mode: %s', config.mode);
end

modelDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'models');
modelPath = fullfile(modelDir, [modelName '.slx']);
if ~isfile(modelPath)
    error('FOCTwin:ModelMissing', 'Model is missing: %s', modelPath);
end

load_system(modelPath);
cleanup = onCleanup(@() close_system(modelName, 0));
variables = foctwin_model_variables(config);
simulationInput = Simulink.SimulationInput(modelName);
names = fieldnames(variables);
for index = 1:numel(names)
    simulationInput = simulationInput.setVariable(names{index}, variables.(names{index}));
end
simulationInput = simulationInput.setModelParameter('StopTime', num2str(config.stop_time_s));
simulationOutput = sim(simulationInput);

result.success = true;
result.model = modelName;
result.stop_time_s = config.stop_time_s;
result.metrics = collectMetrics(simulationOutput);
result.signals = collectSignals(simulationOutput);
clear cleanup;
end

function metrics = collectMetrics(output)
metrics = struct();
names = {'IAE_final', 'ITAE_final', 'IT2AE_final', 'settling_time'};
available = output.who;
for index = 1:numel(names)
    name = names{index};
    if any(strcmp(available, name))
        metrics.(name) = finalValue(output.get(name));
    end
end
end

function signals = collectSignals(output)
signals = struct();
names = {'des_angle', 'cur_angle', 'cur_speed', 'cur_Iq', 'glb_angle', 'glb_speed', 'glb_Iq'};
available = output.who;
for index = 1:numel(names)
    name = names{index};
    if any(strcmp(available, name))
        signals.(name) = serialiseSignal(output.get(name));
    end
end
end

function value = finalValue(input)
if isa(input, 'timeseries')
    value = double(input.Data(end));
elseif isnumeric(input)
    value = double(input(end));
elseif isstruct(input) && isfield(input, 'signals')
    value = double(input.signals.values(end));
else
    value = NaN;
end
end

function value = serialiseSignal(input)
if isa(input, 'timeseries')
    value.time = double(input.Time(:)');
    value.data = double(input.Data(:)');
elseif isstruct(input) && isfield(input, 'time') && isfield(input, 'signals')
    value.time = double(input.time(:)');
    value.data = double(input.signals.values(:)');
elseif isnumeric(input)
    value.time = [];
    value.data = double(input(:)');
else
    value.time = [];
    value.data = [];
end
end
