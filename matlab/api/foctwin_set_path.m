function value = foctwin_set_path(value, path, replacement)
%FOCTWIN_SET_PATH Set a dot-separated field in a nested scalar structure.

parts = strsplit(char(path), '.');
if isempty(parts) || any(cellfun(@isempty, parts))
    error('FOCTwin:InvalidParameterPath', 'Invalid parameter path: %s', path);
end
value = setRecursive(value, parts, replacement, char(path));
end

function value = setRecursive(value, parts, replacement, originalPath)
field = parts{1};
if ~isstruct(value) || ~isfield(value, field)
    error('FOCTwin:UnknownParameter', 'Unknown parameter path: %s', originalPath);
end
if numel(parts) == 1
    value.(field) = replacement;
else
    value.(field) = setRecursive(value.(field), parts(2:end), replacement, originalPath);
end
end
