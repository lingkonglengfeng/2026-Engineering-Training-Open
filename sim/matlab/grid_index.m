function idx = grid_index(spec, cx, cy)
%GRID_INDEX Linear index into an (size x size) grid from 0-based (cx, cy).
%
% The mapspec / firmware / check_map.py all speak 0-based cell coordinates,
% while MATLAB arrays are 1-based. Doing the +1 by hand at every call site is
% exactly how the row/column order gets swapped, so it happens here only.
%
%   idx = GRID_INDEX(spec, cx, cy)      % arrays welcome, result is linear index
%
% Grids are stored as G(cy+1, cx+1)  -- row = cy, column = cx.
% MATLAB is column-major, so the linear index advances along cy first.

idx = (cx + 1) + cy * spec.size;
end
