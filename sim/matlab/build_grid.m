function map = build_grid(spec, opts)
%BUILD_GRID Build the 48x48 field grid, its car-body inflation and its A* data.
%
%   map = BUILD_GRID(spec)
%   map = BUILD_GRID(spec, struct('obstacles', [cx cy; ...], 'lock_zone', 2))
%
% This is the MATLAB implementation of the firmware's GridNav_BuildFieldMap()
% plus GridNav_CellPassable(), and of check_map.py's build_kind() +
% build_walkable(). All three must agree; test_build_grid.m proves it.
%
% Inputs
%   spec.obstacles  Nx2 0-based (cx, cy) runtime obstacles (`&obs`), optional
%   spec.lock_zone  1 or 2: after `&leave`/`&cross` that start zone becomes
%                   no-contact for the body; empty/0 = no lock, optional
%
% Output map fields
%   .kind        {size x size} cell array of type name strings
%   .walkable    logical(size): car CENTRE may stand here (body fits, clear)
%   .drivable    logical(size): ground is road/ring/start/work (no body check)
%   .pads        4x4 rects [x0 x1 y0 y1] mm, no-contact table tops
%   .lane_mm     lane centre lines the A* prefers
%   .carside     car footprint mm
%   .locked      logical(size): cells the body may not touch due to lock_zone
%
% Cell row/column convention: G(cy+1, cx+1), cy increasing DOWNWARD.

if nargin < 2 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'obstacles') || isempty(opts.obstacles)
    obstacles = zeros(0, 2);
else
    obstacles = opts.obstacles;
end
if ~isfield(opts, 'lock_zone') || isempty(opts.lock_zone)
    lock_zone = 0;
else
    lock_zone = opts.lock_zone;
end

n    = spec.size;
cell = spec.cell_mm;
fk   = spec.field_kind;

% ---- cell centres in mm, laid out so that G(cy+1, cx+1) ----------------
xrow = ((0:n-1) * cell + cell / 2);           % 1 x n
ycol = ((n-1:-1:0) * cell + cell / 2).';      % n x 1, y falls as cy grows
X = repmat(xrow, n, 1);
Y = repmat(ycol, 1, n);
x = X(:);
y = Y(:);

% ---- static cell types (mirror of build_kind / GridNav_BuildFieldMap) --
% Parenthesise the two axis tests: `A & B & C | D & E` parses as
% `(A & B & C) | (D & E)` in MATLAB (not as `A & (B|D) & ...`), which silently
% turned every y-in-cross-band cell into ROAD and wiped out RING entirely.
cross_x = (x >= fk.mid_mm - fk.cross_half_mm) & (x < fk.mid_mm + fk.cross_half_mm);
cross_y = (y >= fk.mid_mm - fk.cross_half_mm) & (y < fk.mid_mm + fk.cross_half_mm);
is_cross = cross_x | cross_y;
is_ring  = (x < fk.block_lo_mm) | (x >= fk.block_hi_mm) | ...
           (y < fk.block_lo_mm) | (y >= fk.block_hi_mm);

kind = repmat({'BLOCKED'}, n * n, 1);
% cross corridor first, then the outer ring band fills the rest of the drivable
% ground; whatever is neither stays BLOCKED (the four 450x450 yellow zones).
kind(is_cross) = {'ROAD'};
if ~isfield(spec.field_kind, 'ring_drivable') || spec.field_kind.ring_drivable
    kind(is_ring & ~is_cross) = {'RING'};
else
    % STRICT READING: the rules name only the 400mm gray cross lane as a driving
    % lane, and the band is not a different colour on the rule book map, so it
    % has no basis as a lane. Everything outside the cross stays out of bounds.
    % This is a rules-interpretation switch, not a geometry correction - see
    % sim/docs/设计说明.md and test_ring_scope.m for the consequences of each way.
    kind(is_ring) = {'BLOCKED'};
end

m_start = in_any_rect(x, y, spec.area_types.start);
m_work  = in_any_rect(x, y, spec.area_types.work);
m_pad   = in_any_rect(x, y, spec.pads);

kind(m_start) = {'START'};
sel = m_work & ~m_start & ~strcmp(kind, 'BLOCKED');
kind(sel) = {'WORK'};
kind(m_pad) = {'PAD'};

% ---- runtime obstacles (`&obs`) ----------------------------------------
if ~isempty(obstacles)
    obs_mask = false(n * n, 1);
    for k = 1:size(obstacles, 1)
        obs_mask(grid_index(spec, obstacles(k, 1), obstacles(k, 2))) = true;
    end
    % an obstacle blocks the ground it sits on (the body test below then does
    % the inflation, exactly like the python reference)
    m_pad = m_pad | obs_mask;
end

% ---- ground drivable?  (BLOCKED and PAD are not) -----------------------
drivable = ~(strcmp(kind, 'BLOCKED') | strcmp(kind, 'PAD'));

% ---- car-body inflation: 6x6 cells, 3 cells each side ------------------
half   = spec.car_half_cells;
xc     = x - spec.car_mm / 2;      % car's low corner for each candidate cell
yc     = y - spec.car_mm / 2;

in_field = (xc >= 0) & (xc + spec.car_mm <= spec.field_mm) & ...
           (yc >= 0) & (yc + spec.car_mm <= spec.field_mm);

% pad grid to size+2 so the 7x7 neighbourhood window never runs off the edge;
% the pad ring itself is marked as blocked, which reproduces the python
% reference's exact `range(y0//CELL, y1//CELL + 1)` window.
wall = true(n + 2);
wall(2:n + 1, 2:n + 1) = reshape(~drivable, n, n);
wall = reshape(wall, (n + 2) * (n + 2), 1);
clear_of_walls = false(n * n, 1);
% Build the FULL 7x7 = 49 linear indices of the touched window.
% WARNING: `(r+2) + (b+1)*(n+2)` with two 1x7 row vectors does NOT broadcast in
% MATLAB - it returns 7 element-wise sums, so the loop below would test 7 cells
% instead of 49 and quietly declare cells that overhang a table top "passable"
% (that bug produced 953 walkable cells instead of 734). The explicit outer sum
% is what makes this a real 6x6 inflation test.
for k = 1:(n * n)
    if ~in_field(k)
        continue
    end
    b = floor(xc(k) / cell) + (0:2 * half);      % 7 columns touched
    r = floor(yc(k) / cell) + (0:2 * half);      % 7 rows touched
    lin = (r + 2).' + (b + 1) * (n + 2);         % 7x7 = 49 window cells
    % `if wall(lin(:))` would be WRONG here: a 49-element logical vector used as
    % a scalar condition means "ALL are true" in MATLAB. The inflation test is
    % "ANY forbidden cell inside the footprint", so it must be any(...).
    if any(wall(lin(:)))
        continue
    end
    clear_of_walls(k) = true;
end

walkable = in_field & clear_of_walls;

% ---- start-zone NO-TOUCH lock (map frame, strict overlap) --------------
locked = false(n * n, 1);
if lock_zone == 1 || lock_zone == 2
    zx0 = 2100; zx1 = spec.field_mm;
    if lock_zone == 1
        zy0 = 0; zy1 = 300;
    else
        zy0 = spec.field_mm - 300; zy1 = spec.field_mm;
    end
    % the candidate rect is in the INTERNAL frame (rows grow downward) while
    % the zone lives in rule-book mm, so flip y first. Getting this wrong
    % swaps which zone ends up locked.
    my0  = spec.field_mm - (yc + spec.car_mm);
    my1  = spec.field_mm - yc;
    overlaps = (xc < zx1) & (xc + spec.car_mm > zx0) & (my0 < zy1) & (my1 > zy0);
    locked   = overlaps;         % python returns True for any x0<FIELD,x1>2100
    walkable = walkable & ~locked;
end

map = struct();
map.spec     = spec;
map.kind     = reshape(kind, n, n);
map.drivable = reshape(drivable, n, n);
map.walkable = reshape(walkable, n, n);
map.locked   = reshape(locked, n, n);
map.pads     = spec.pads;
map.lane_mm  = [spec.lanes_mm.near, spec.lanes_mm.mid, spec.lanes_mm.far];
map.carside  = spec.car_mm;
map.obstacles = obstacles;
map.lock_zone = lock_zone;
end

% ----------------------------------------------------------------------
function m = in_any_rect(x, y, rects)
%IN_ANY_RECT True where (x, y) falls in any [x0 x1 y0 y1] half-open rect.
% Half-open (x0 <= x < x1) to match check_map.py's in_range().
m = false(size(x));
if isempty(rects)
    return
end
for k = 1:size(rects, 1)
    m = m | (x >= rects(k, 1) & x < rects(k, 2) & ...
             y >= rects(k, 3) & y < rects(k, 4));
end
end
