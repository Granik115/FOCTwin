function foctwin_write_json(resultPath, value)
%FOCTWIN_WRITE_JSON Write UTF-8 JSON through an atomic same-directory move.

[directory, name, extension] = fileparts(resultPath);
if isempty(directory)
    directory = pwd;
end
if ~isfolder(directory)
    mkdir(directory);
end

temporary = fullfile(directory, [name extension '.tmp']);
fileId = fopen(temporary, 'w', 'n', 'UTF-8');
if fileId < 0
    error('FOCTwin:WriteFailed', 'Cannot open result file: %s', temporary);
end

cleanup = onCleanup(@() fclose(fileId));
fwrite(fileId, jsonencode(value), 'char');
fwrite(fileId, newline, 'char');
clear cleanup;
movefile(temporary, resultPath, 'f');
end
