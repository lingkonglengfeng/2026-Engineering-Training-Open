function spec = load_mapspec(path)
%LOAD_MAPSPEC Read the field geometry JSON emitted by tools/check_map.py.
%
%   spec = LOAD_MAPSPEC()            % uses the repo default location
%   spec = LOAD_MAPSPEC(path)        % explicit path
%
% The JSON is the single source of truth shared by:
%   tools/check_map.py   (exporter + reference implementation, Python)
%   Core/Inc/grid_nav.h  (firmware, C)
%   sim/matlab/*         (this simulator)
%
% Regenerate it with:
%   C:\ProgramData\miniconda3\python.exe tools\check_map.py --mapspec sim\mapspec\field_mapspec.json
%
% No MATLAB toolbox is required (jsondecode is core MATLAB since R2016b).

if nargin < 1 || isempty(path)
    path = default_mapspec_path();
end

if ~isfile(path)
    error('load_mapspec:missing', ...
        ['mapspec not found: %s\n' ...
         'Generate it first:\n' ...
         '  C:\\ProgramData\\miniconda3\\python.exe tools\\check_map.py --mapspec sim\\mapspec\\field_mapspec.json'], ...
        path);
end

raw = fileread(path);
spec = jsondecode(raw);

if ~strcmp(spec.format, 'field-mapspec')
    error('load_mapspec:format', 'unexpected format field: %s', spec.format);
end
if spec.version ~= 1
    error('load_mapspec:version', ...
        'mapspec version %d is not supported by this simulator (expected 1)', spec.version);
end

spec.source_path = path;
end

function p = default_mapspec_path()
% Walk up from this file to the repo root, then use sim/mapspec/.
here = fileparts(mfilename('fullpath'));      % <repo>/sim/matlab
root = fileparts(fileparts(here));            % <repo>
p = fullfile(root, 'sim', 'mapspec', 'field_mapspec.json');
end
