function varargout = map_coords(op, varargin)
%MAP_COORDS The ONLY rule-book-mm <-> internal-cell conversion point.
%
%   [cx, cy] = MAP_COORDS('mm2cell', spec, x, y)
%   [cx, cy] = MAP_COORDS('mm2cell_clamped', spec, x, y)
%   [x, y]   = MAP_COORDS('cell2mm', spec, cx, cy)     % cell CENTRE in mm
%
% Two frames exist in this project, and mixing them is the documented cause of
% "the car drives the wrong way / off the mat" (docs/hardware/场地地图与栅格说明.md):
%
%   MAP frame  (rule book, human facing): origin = field lower-left inner
%              corner, x right, y UP, unit mm. Everything user-facing uses it.
%   CELL frame (internal): cx right, cy DOWN, so cy = size-1 - floor(y/cell).
%              Downward rows keep the firmware's field-verified "dy>0 = forward"
%              direction chain untouched.
%
% Stands in for the firmware's GridNav_MmToCellY() / GridNav_CellCenterMmY()
% and for check_map.py's mm_to_cell() / cell_to_mm().
%
% Inputs accept arrays (element-wise). Returned cell indices keep the
% firmware/Python 0-BASED cx, so a MATLAB array access needs +1 - every caller
% goes through GRID_INDEX() for that offset.
%
% NOTE: the first argument is named `op` and the spec arrives inside `varargin`
% on purpose. Naming a parameter `spec` here would collide with this file's own
% remaining parameters and MATLAB would resolve `spec` as a function call.

if nargin < 1
    error('map_coords:args', 'usage: map_coords(op, spec, ...)');
end
if numel(varargin) < 1
    error('map_coords:args', 'missing mapspec argument for op "%s"', op);
end
sp = varargin{1};
rest = varargin(2:end);

switch lower(op)
    case 'mm2cell'
        [varargout{1}, varargout{2}] = mm2cell(sp, rest{:});
    case 'mm2cell_clamped'
        [varargout{1}, varargout{2}] = mm2cell_clamped(sp, rest{:});
    case 'cell2mm'
        [varargout{1}, varargout{2}] = cell2mm(sp, rest{:});
    otherwise
        error('map_coords:op', 'unknown op "%s"', op);
end
end

function [cx, cy] = mm2cell(sp, x, y)
cx = floor(x ./ sp.cell_mm);
cy = (sp.size - 1) - floor(y ./ sp.cell_mm);
end

function [cx, cy] = mm2cell_clamped(sp, x, y)
% Mirror of the python/firmware clamp: out-of-field points snap to the border
% cell instead of returning an out-of-range index.
cx = min(sp.size - 1, floor(x ./ sp.cell_mm));
cy = (sp.size - 1) - min(sp.size - 1, floor(y ./ sp.cell_mm));
end

function [x, y] = cell2mm(sp, cx, cy)
x = cx .* sp.cell_mm + sp.cell_mm / 2;
y = (sp.size - 1 - cy) .* sp.cell_mm + sp.cell_mm / 2;
end
