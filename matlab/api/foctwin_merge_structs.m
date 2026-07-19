function merged = foctwin_merge_structs(defaults, override)
%FOCTWIN_MERGE_STRUCTS Recursively merge a JSON request over canonical defaults.

merged = defaults;
if isempty(override)
    return;
end
names = fieldnames(override);
for index = 1:numel(names)
    name = names{index};
    if isfield(merged, name) && isstruct(merged.(name)) && isstruct(override.(name))
        merged.(name) = foctwin_merge_structs(merged.(name), override.(name));
    else
        merged.(name) = override.(name);
    end
end
end

