function foctwin_simulate(requestPath, resultPath)
%FOCTWIN_SIMULATE Stable JSON-file entry point for Python and compiled builds.

startedAt = datetime('now', 'TimeZone', 'UTC');
try
    request = jsondecode(fileread(requestPath));
    config = foctwin_merge_structs(foctwin_default_config(), request);
    result = foctwin_run_simulation(config);
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
