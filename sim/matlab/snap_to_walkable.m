function [cell_out, snapped, dist] = snap_to_walkable(map, cell_in, max_ring)
%SNAP_TO_WALKABLE Nearest passable cell by ring search around a target cell.
%
%   [cell_out, snapped, dist] = SNAP_TO_WALKABLE(map, cell_in)
%
% Mirror of the firmware's GridNav_SnapToWalkable() and check_map.py snap().
%
% Why this exists (documented real bug): rule-book "round mm" stop points such
% as (1200, 2250) land exactly on a CELL BOUNDARY. The nearest cell centre is
% then 25 mm inside a forbidden strip (the pad / the zone edge), so planning
% straight from the raw point returns NO PATH for every target. Planning snaps
% the endpoint to the nearest passable cell centre, while EXECUTION still drives
% to the exact mm - hence the `snapped` flag, which is worth reporting.
%
% NOTE: the arguments are deliberately NOT named `cell`. Inside this function a
% parameter called `cell` shadows MATLAB's built-in cell() constructor, so
% `cell(1)` silently becomes a function call returning an empty cell instead of
% indexing the argument - which then fails as "too many output arguments".

if nargin < 3 || isempty(max_ring)
    max_ring = 8;         % same search radius as the firmware
end

n = map.spec.size;
cell_out = cell_in;
snapped = false;
dist = 0;

if cell_in(1) >= 0 && cell_in(1) < n && cell_in(2) >= 0 && cell_in(2) < n && ...
   map.walkable(cell_in(2) + 1, cell_in(1) + 1)
    return
end
snapped = true;

best = [];
bestd = inf;
for r = 1:max_ring
    for dy = -r:r
        for dx = -r:r
            nx = cell_in(1) + dx;
            ny = cell_in(2) + dy;
            if nx < 0 || nx >= n || ny < 0 || ny >= n
                continue
            end
            if ~map.walkable(ny + 1, nx + 1)
                continue
            end
            d = dx * dx + dy * dy;
            if d < bestd
                bestd = d;
                best = [nx ny];
            end
        end
    end
    if ~isempty(best)
        dist = bestd;
        cell_out = best;
        return
    end
end
% nothing passable within max_ring: leave the cell as-is (caller will fail)
end
