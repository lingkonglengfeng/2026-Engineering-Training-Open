function [path, info] = astar_grid(map, start_cell, goal_cell)
%ASTAR_GRID 4-connected A* over the walkable grid, with turn penalty + lane bias.
%
%   [path, info] = ASTAR_GRID(map, start_cell, goal_cell)
%     map         from BUILD_GRID
%     start_cell  [cx cy] 0-based (MAP mm converted via MAP_COORDS('mm2cell_clamped'))
%     goal_cell   [cx cy] 0-based
%
%   path  Nx2 0-based [cx cy] cells, start..goal, or [] when unreachable
%   info  .cells      number of steps (numel(path)-1)
%   info  .snapped    true if an endpoint had to be snapped
%   info  .start/.goal  the snapped endpoints
%
% Cost model, identical to check_map.py astar():
%   step         1 per cell
%   line change  +120  (GRID_NAV_TURN_COST) -> 12x a step: fewer corners beats
%                fewer cells, because every corner costs a real in-place turn
%   lane bias    +3 per cell away from the nearest lane centre line
%                (GRID_NAV_LANE_COST_PER_CELL) -> stay near 275/1200/2125
%
% Note (deliberate replica): the start cell is expanded with turn = 0 because it
% has no incoming direction. That is what check_map.py does, so the two
% implementations produce comparable route lengths. Do not "fix" this on one
% side only - test_build_grid.m compares leg counts against that baseline.

n    = map.spec.size;
turn_cost = map.spec.astar.turn_cost;
lane_cost = map.spec.astar.lane_cost_per_cell;

[start_cell, s_snap] = snap_to_walkable(map, start_cell);
[goal_cell,  g_snap] = snap_to_walkable(map, goal_cell);

path = [];
info = struct('cells', 0, 'snapped', s_snap || g_snap, ...
              'start', start_cell, 'goal', goal_cell, 'cost', inf);

if ~map.walkable(start_cell(2) + 1, start_cell(1) + 1) || ...
   ~map.walkable(goal_cell(2) + 1, goal_cell(1) + 1)
    return
end

total    = n * n;
g        = inf(total, 1);
came     = zeros(total, 1);
came_dir = zeros(total, 1);      % 1..4 = U D R L, 0 = none
closed   = false(total, 1);

si = grid_index(map.spec, start_cell(1), start_cell(2));
gi = grid_index(map.spec, goal_cell(1),  goal_cell(2));

g(si) = 0;
open = zeros(0, 2);              % [priority, linear index]
open = heap_push(open, 0, si);

dirs = [0 -1; 0 1; 1 0; -1 0];   % U D R L in (dx, dy) with dy down = D

while ~isempty(open)
    [open, ~, cur] = heap_pop(open);
    if closed(cur)
        continue
    end
    closed(cur) = true;

    if cur == gi
        path = reconstruct(came, cur, map.spec, n);
        info.cells = size(path, 1) - 1;
        info.cost  = g(cur);
        return
    end

    cx = mod(cur - 1, n);        % 0-based
    cy = floor((cur - 1) / n);   % 0-based

    for k = 1:4
        nx = cx + dirs(k, 1);
        ny = cy + dirs(k, 2);
        if nx < 0 || nx >= n || ny < 0 || ny >= n
            continue
        end
        if ~map.walkable(ny + 1, nx + 1)
            continue
        end
        ni = grid_index(map.spec, nx, ny);
        if closed(ni)
            continue
        end

        if came_dir(cur) == 0
            turn = 0;                       % start cell: no incoming direction
        elseif came_dir(cur) == k
            turn = 0;
        else
            turn = turn_cost;
        end

        gg = g(cur) + 1 + turn + lane_cost * lane_deviation_cells(map, nx, ny);
        if gg < g(ni)
            g(ni)        = gg;
            came(ni)     = cur;
            came_dir(ni) = k;
            h = abs(nx - goal_cell(1)) + abs(ny - goal_cell(2));
            open = heap_push(open, gg + h, ni);
        end
    end
end
% exhausted the open set: no route
end

% ----------------------------------------------------------------------
function p = reconstruct(came, cur, spec, n)
p = zeros(0, 2);
while cur ~= 0
    p = [mod(cur - 1, n), floor((cur - 1) / n); p];   %#ok<AGROW>
    cur = came(cur);
end
end

function dev = lane_deviation_cells(map, cx, cy)
%Mirror of GridNav_LaneDeviationCells(): how many 50mm cells this cell centre
%is away from the nearest lane centre line (275 / 1200 / 2125).
[x, y] = map_coords('cell2mm', map.spec, cx, cy);
dx = min(abs(x - map.lane_mm));
dy = min(abs(y - map.lane_mm));
dev = floor(min(dx, dy) / map.spec.cell_mm);
end

% ---------------------------- tiny binary min-heap ---------------------
function h = heap_push(h, prio, idx)
h = [h; prio, idx];
i = size(h, 1);
while i > 1
    p = floor(i / 2);
    if h(p, 1) <= h(i, 1)
        break
    end
    h([p i], :) = h([i p], :);
    i = p;
end
end

function [h, prio, idx] = heap_pop(h)
prio = h(1, 1);
idx  = h(1, 2);
h(1, :) = h(end, :);
h(end, :) = [];
i = 1;
m = size(h, 1);
while true
    l = 2 * i; r = l + 1; s = i;
    if l <= m && h(l, 1) < h(s, 1), s = l; end
    if r <= m && h(r, 1) < h(s, 1), s = r; end
    if s == i, break, end
    h([i s], :) = h([s i], :);
    i = s;
end
end
