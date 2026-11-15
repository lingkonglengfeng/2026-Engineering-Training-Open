function [names, codes] = cell_types()
%CELL_TYPES Cell type names and their numeric codes, in firmware enum order.
%
% Mirrors `grid_cell_type_t` in Core/Inc/grid_nav.h and the ROAD/RING/... tuple
% in tools/check_map.py. Derived facts (never hand-copied):
%   walkable for the BODY : everything except BLOCKED and PAD
%   ground is drivable    : everything except BLOCKED
%
% The distinction matters: `WORK` is only a functional-area semantic overlay
% (it contains the free stop space the car may stand in) and MUST stay
% passable, while `PAD` is a real table top the body must never touch.

names = {'ROAD', 'RING', 'BLOCKED', 'START', 'WORK', 'PAD'};
codes = struct('ROAD', 0, 'RING', 1, 'BLOCKED', 2, 'START', 3, 'WORK', 4, 'PAD', 5);
end
