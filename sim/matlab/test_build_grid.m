function test_build_grid()
%TEST_BUILD_GRID Cross-validate the MATLAB map against the Python baseline.
%
% Run:  matlab -batch "cd('E:\Engineering_Training_Project\sim\matlab'); test_build_grid"
%
% The mapspec deliberately exports only field semantics, so this test is the
% proof that MATLAB rebuilt the inflation, the A* and the executor decision the
% same way the reference (tools/check_map.py) did. Every EXPECTED number below
% is copied from that script's current output.
%
% If a number here fails, the mapspec or the firmware geometry changed - decide
% which side is right BEFORE editing the expectation.

spec = load_mapspec();
map  = build_grid(spec);

fails = 0;

section('FIELD SIZE');
expect(spec.field_mm == 2400 && spec.size == 48 && spec.cell_mm == 50, ...
    'field 2400mm / 48 cells / 50mm', sprintf('%g / %d / %g', ...
    spec.field_mm, spec.size, spec.cell_mm));

section('WALKABLE CELLS (car-body inflation)');
walk_count = nnz(map.walkable);
fails = fails + expect(walk_count == 734, '734 / 2304 walkable (32%)', ...
    sprintf('%d / %d', walk_count, numel(map.walkable)));

h1 = snap_to_walkable(map, mmcell(2400 - 150, 150));   % 2250, 150 -> cell 45,44
reachmask = reachable_from(map, h1);
% NOTE: reachable_from returns an n x n mask; comparing the MASK to a scalar
% would compare every element and produce nonsense, so count the cells.
n_reach = nnz(reachmask);
fails = fails + expect(n_reach == walk_count, ...
    'ALL CONNECTED from H1', sprintf('%d reachable (have %d walkable)', ...
    n_reach, walk_count));

section('NAMED WAYPOINTS  (mm -> cell -> walkable / reachable)');
W = spec.waypoints_mm;
expected_cells = { ...
    'center',   [24 23], true; ...
    'raw',      [24  6], true; ...
    'rough',    [24 40], true; ...
    'rough1',   [21 40], true; ...
    'rough2',   [24 40], true; ...
    'rough3',   [27 40], true; ...
    'temp',     [ 7 23], true; ...
    'temp1',    [ 7 20], true; ...
    'temp2',    [ 7 23], true; ...
    'temp3',    [ 7 26], true; ...
    'qr',       [42 23], true; ...
    'h1',       [45 44], false; ...   % zone centre itself is NOT a legal car centre
    'h2',       [45  2], false; ...
    'exit1',    [42 38], true; ...
    'exit2',    [42  8], true };

for i = 1:size(expected_cells, 1)
    name = expected_cells{i, 1};
    want = expected_cells{i, 2};
    wall = expected_cells{i, 3};
    pt = W.(name);
    cell = mmcell(pt(1), pt(2));
    got_wall = map.walkable(cell(2) + 1, cell(1) + 1);
    got_reach = reachmask(cell(2) + 1, cell(1) + 1);
    ok = isequal(cell, want) && got_wall == wall && got_reach == wall;
    fails = fails + expect(ok, sprintf('%-8s (%4d,%4d) -> (%2d,%2d) walk=%d', ...
        name, pt(1), pt(2), want(1), want(2), wall), ...
        sprintf('(%2d,%2d) walk=%d reach=%d', cell(1), cell(2), ...
        got_wall, got_reach));
end

section('TASK LEGS  (cells / segments / turns / strafes)');
legs = { ...
    'h1',     'exit1',   8, 2, 1, 1; ...
    'exit1',  'raw',    50, 2, 2, 0; ...
    'raw',    'rough2', 34, 1, 1, 0; ...
    'rough2', 'temp2',  34, 2, 2, 0; ...
    'temp2',  'qr',     35, 1, 1, 0; ...
    'qr',     'h2',     22, 2, 1, 1; ...
    'h2',     'exit2',   7, 2, 1, 1; ...
    'exit2',  'rough2', 50, 2, 2, 0; ...
    'h2',     'h1',     43, 4, 1, 2 };

for i = 1:size(legs, 1)
    a = legs{i, 1}; b = legs{i, 2};
    want = [legs{i, 3} legs{i, 4} legs{i, 5} legs{i, 6}];
    [path, info] = astar_grid(map, mmcell(W.(a)(1), W.(a)(2)), ...
                                   mmcell(W.(b)(1), W.(b)(2)));
    if isempty(path)
        fails = fails + expect(false, sprintf('%-6s -> %-6s route exists', a, b), 'NO PATH');
        continue
    end
    segs = path_segments(path, spec);
    p = plan_maneuvers(segs, spec);
    got = [info.cells, size(segs, 1), p.turns, p.strafes];
    fails = fails + expect(isequal(got, want), ...
        sprintf('%-6s -> %-6s %2d cells, %d legs, %d turns, %d strafes', ...
                a, b, want(1), want(2), want(3), want(4)), ...
        sprintf('%d cells, %d legs, %d turns, %d strafes | %s', ...
                got(1), got(2), got(3), got(4), strjoin(p.actions, ' | ')));
end

section('START-ZONE NO-TOUCH LOCK');
for zone = 1:2
    locked_map = build_grid(spec, struct('lock_zone', zone));
    cnt = nnz(locked_map.walkable);
    % the python baseline measures reachability from the SAME zone's exit point
    ex = W.(sprintf('exit%d', zone));
    st = snap_to_walkable(locked_map, mmcell(ex(1), ex(2)));
    r = nnz(reachable_from(locked_map, st));
    fails = fails + expect(cnt == 699 && r == 699, ...
        sprintf('zone %d locked: 699 walkable, 699 reachable', zone), ...
        sprintf('%d walkable, %d reachable', cnt, r));
end

section('A* COST MODEL');
fprintf('   move=%.1f turn=%.1f lane=%.1f  (mapspec / check_map.py semantics)\n', ...
    spec.astar.move_cost, spec.astar.turn_cost, spec.astar.lane_cost_per_cell);
fprintf('   firmware NOTE: grid_nav.c uses GRID_NAV_MOVE_COST=10 with the same\n');
fprintf('   TURN_COST=120, so the firmware weights a turn 12x a cell while this\n');
fprintf('   model weights it 120x. Route choice can therefore differ; sim_scan.m\n');
fprintf('   re-plans with the firmware weights and reports any divergence.\n');

fprintf('\n%s\n', repmat('=', 1, 66));
if fails == 0
    fprintf('RESULT: PASS - MATLAB map matches the Python baseline on every item\n');
else
    fprintf('RESULT: %d FAILURE(S) - fix the model or the expectation, do not ignore\n', fails);
end
fprintf('%s\n', repmat('=', 1, 66));
if fails > 0
    error('test_build_grid:failed', '%d check(s) failed', fails);
end
end

% ----------------------------------------------------------------------
function cell = mmcell(x, y)
%local convenience: rule-book mm -> 0-based cell, clamped like the firmware
cell = [min(47, floor(x / 50)), 47 - min(47, floor(y / 50))];
end

function reach = reachable_from(map, cell)
n = map.spec.size;
reach = false(n, n);
if ~map.walkable(cell(2) + 1, cell(1) + 1)
    return
end
q = zeros(n * n, 2);
head = 1; tail = 1;
q(1, :) = cell;
reach(cell(2) + 1, cell(1) + 1) = true;
d = [0 -1; 0 1; 1 0; -1 0];
while head <= tail
    cur = q(head, :); head = head + 1;
    for k = 1:4
        nx = cur(1) + d(k, 1);
        ny = cur(2) + d(k, 2);
        if nx < 0 || nx >= n || ny < 0 || ny >= n
            continue
        end
        if map.walkable(ny + 1, nx + 1) && ~reach(ny + 1, nx + 1)
            reach(ny + 1, nx + 1) = true;
            tail = tail + 1;
            q(tail, :) = [nx ny];
        end
    end
end
end

function fails = expect(ok, label, got)
if ok
    fprintf('  [ ok ] %s\n', label);
    fails = 0;
else
    fprintf('  [FAIL] %s\n         got: %s\n', label, got);
    fails = 1;
end
end

function section(title)
fprintf('\n--- %s ---\n', title);
end
