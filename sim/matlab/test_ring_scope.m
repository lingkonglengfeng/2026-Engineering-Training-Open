function test_ring_scope()
%TEST_RING_SCOPE What does the rules interpretation of the outer band COST?
%
% The rules (docs/electrical/rules.pdf p3) name exactly ONE driving lane:
%   "行车道为灰色，中间十字灰色行车道宽度为 400mm" ... "其余区域为亚光白色等浅色底色"
% and "离开启停区后只能在灰色车道上行驶，进入其它颜色区域（除启停区）结束比赛".
%
% There is NO stated "outer ring band". 550 mm is DERIVED as (2400-400-2*450)/2,
% and on the rule book map the band is the SAME grey as the rest of the floor, so
% it is not even visually separable as a lane.
%
% The firmware treats the band as drivable - that is what makes the map connected
% and what the car has been driving on. But under the strict reading, everything
% the car does outside the cross is an instant loss. So this decides whether the
% navigation approach is valid at all, not a detail.
%
% This script does not argue: it builds both interpretations and reports numbers,
% so the call can be made on evidence.

spec = load_mapspec();

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('OUTER-BAND SCOPE TEST  (is the 550 mm ring band a legal driving lane?)\n');
fprintf('%s\n', repmat('=', 1, 74));

cases = {true,  'A) band drivable   (current firmware behaviour)'; ...
         false, 'B) strict reading   (only the 400mm cross + start zones)'};

for c = 1:size(cases, 1)
    ring_ok = cases{c, 1};
    sp = spec;
    sp.field_kind.ring_drivable = ring_ok;
    map = build_grid(sp);

    walk = nnz(map.walkable);
    fprintf('\n%s\n', cases{c, 2});
    fprintf('   walkable cells : %4d / %d\n', walk, numel(map.walkable));

    ex = sp.zones.x2.exit_mm;                       % the team's launch-zone exit
    c0 = snap_cell(map, mm2cx(sp, ex(1)), mm2cy(sp, ex(2)));
    reach = reachable(map, c0);
    re = nnz(reach);
    fprintf('   reachable      : %4d  -> %s\n', re, ...
        ternary(re == walk, 'ALL CONNECTED', 'ISOLATED ISLANDS'));

    names = fieldnames(sp.waypoints_mm);
    bad = {};
    for i = 1:numel(names)
        p = sp.waypoints_mm.(names{i});
        sc = snap_cell(map, mm2cx(sp, p(1)), mm2cy(sp, p(2)));
        if ~reach(sc(2) + 1, sc(1) + 1)
            bad{end + 1} = names{i};   %#ok<AGROW>
        end
    end
    if isempty(bad)
        fprintf('   waypoints      : all %d reachable\n', numel(names));
    else
        fprintf('   waypoints      : UNREACHABLE -> %s\n', strjoin(bad, ', '));
    end

    legs = {'h1','exit1'; 'exit1','raw'; 'raw','rough2'; 'rough2','temp2'; ...
            'temp2','qr'; 'qr','h2'; 'h2','exit2'; 'exit2','rough2'; 'h2','h1'};
    n_bad = 0;
    bad_legs = {};
    for i = 1:size(legs, 1)
        pa = sp.waypoints_mm.(legs{i, 1});
        pb = sp.waypoints_mm.(legs{i, 2});
        sa = snap_cell(map, mm2cx(sp, pa(1)), mm2cy(sp, pa(2)));
        sb = snap_cell(map, mm2cx(sp, pb(1)), mm2cy(sp, pb(2)));
        if isequal(sa, sb)
            continue
        end
        [p, info] = astar_grid(map, sa, sb);
        if isempty(p) || isinf(info.cost)
            n_bad = n_bad + 1;
            bad_legs{end + 1} = sprintf('%s->%s', legs{i, 1}, legs{i, 2});   %#ok<AGROW>
        end
    end
    fprintf('   task legs      : %d / %d drivable\n', size(legs, 1) - n_bad, size(legs, 1));
    if n_bad > 0
        fprintf('                    NO PATH -> %s\n', strjoin(bad_legs, ', '));
    end

    if ring_ok
        fprintf('   VERDICT        : navigation works\n');
    else
        fprintf('   VERDICT        : %s\n', ...
            ternary(re == walk && n_bad == 0, ...
            'still connected - the band may not be needed', ...
            'THE MAP BREAKS - the mission cannot be driven this way'));
    end
end

fprintf('\n%s\n', repmat('-', 1, 74));
fprintf('WHAT THIS MEANS\n');
fprintf('  If A holds: current navigation stands; the only exposure is a strict\n');
fprintf('    judge calling the band "other colour".\n');
fprintf('  If B holds: every leg using the band loses the run instantly and the\n');
fprintf('    mission route must be redesigned.\n');
fprintf('  The band is NOT visually separable from the floor (same grey), so the\n');
fprintf('  rule book map cannot settle it - it needs the real mat or a judge.\n');
fprintf('  The simulator runs A (firmware behaviour) and records here what B costs.\n');
fprintf('%s\n\n', repmat('=', 1, 74));
end

% ----------------------------------------------------------------------
function cx = mm2cx(sp, x)
% Inline coordinate conversion. Deliberately NOT map_coords(): calling a
% two-output helper with a single output name silently keeps only cx, which
% reads as a scalar and then fails far away from its cause.
cx = min(sp.size - 1, floor(x / sp.cell_mm));
end

function cy = mm2cy(sp, y)
cy = (sp.size - 1) - min(sp.size - 1, floor(y / sp.cell_mm));
end

function sc = snap_cell(map, cx, cy)
% Local ring-search snap, so this script does not depend on a shared helper.
n = map.spec.size;
if cx >= 0 && cx < n && cy >= 0 && cy < n && map.walkable(cy + 1, cx + 1)
    sc = [cx cy];
    return
end
for r = 1:8
    best = []; bestd = inf;
    for dy = -r:r
        for dx = -r:r
            nx = cx + dx; ny = cy + dy;
            if nx < 0 || nx >= n || ny < 0 || ny >= n
                continue
            end
            if ~map.walkable(ny + 1, nx + 1)
                continue
            end
            d = dx * dx + dy * dy;
            if d < bestd
                bestd = d; best = [nx ny];
            end
        end
    end
    if ~isempty(best)
        sc = best;
        return
    end
end
sc = [cx cy];
end

function reach = reachable(map, c0)
n = map.spec.size;
reach = false(n, n);
if ~map.walkable(c0(2) + 1, c0(1) + 1)
    return
end
q = zeros(n * n, 2);
head = 1; tail = 1;
q(1, :) = c0;
reach(c0(2) + 1, c0(1) + 1) = true;
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

function s = ternary(c, a, b)
if c
    s = a;
else
    s = b;
end
end
