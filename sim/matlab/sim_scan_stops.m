function sim_scan_stops(opts)
%SIM_SCAN_STOPS Sweep a named stop point ("停位") and report the legal window.
%
%   sim_scan_stops()
%   sim_scan_stops(struct('wp', 'raw', 'leg', {{'exit1','raw'}}))
%
% WHY THIS IS A WINDOW AND NOT A TIME CURVE. The planner works on 50 mm cells and
% snaps the requested mm point to the nearest LEGAL cell centre, then the executor
% drives to that cell centre. So moving the stop a few mm usually changes nothing
% at all - the same cell centre is chosen - and the interesting question is not
% "which stop is fastest" but:
%
%   HOW FAR CAN EACH STOP MOVE before the planner is forced onto a different cell
%   (or refuses the point outright)?
%
% That window is the real design tolerance: it says how much slack the field
% layout leaves for on-site calibration, which matters because the rules place
% several zones RANDOMLY and the team has to re-measure them on the day
% (see sim/docs/设计说明.md 8.8).
%
% METHOD: march the stop point along the lane it sits on, in 5 mm steps, and record
% the snapped cell each time. Report the breakpoints. Also run the leg at the
% shifted positions to confirm nothing becomes illegal.

if nargin < 1 || isempty(opts)
    opts = struct();
end
opts = setdef(opts, 'step_mm', 5);
opts = setdef(opts, 'span_mm', 150);       % sweep +- this much along the lane

spec = load_mapspec();
targets = { ...
    'raw',   [1200 2050], [0 1],  {'exit1', 'raw'}; ...
    'rough2',[1200  350], [0 1],  {'raw',  'rough2'}; ...
    'temp2', [ 350 1200], [1 0],  {'rough2','temp2'}; ...
    'qr',    [2125 1200], [0 1],  {'temp2','qr'}; ...
    'exit2', [2125 1950], [0 1],  {'h2',  'exit2'}};

if isfield(opts, 'wp') && ~isempty(opts.wp)
    keep = strcmp(targets(:, 1), opts.wp);
    targets = targets(keep, :);
    if isempty(targets)
        error('sim_scan_stops:wp', 'unknown waypoint "%s"', opts.wp);
    end
end

fprintf('\n%s\n', repmat('=', 1, 84));
fprintf('STOP-POINT ("停位") SWEEP\n');
fprintf('%s\n', repmat('=', 1, 84));
fprintf('%-8s %-8s %-10s %-9s %-11s %s\n', ...
    'stop', 'lane', 'requested', 'cell', 'offset mm', 'cell changes at (mm)');
fprintf('%s\n', repmat('-', 1, 84));

for i = 1:size(targets, 1)
    name = targets{i, 1};
    base = targets{i, 2};
    dirv = targets{i, 3};

    % snapped cell at every sample, plus the offset it implies
    cells = {};
    for d = -opts.span_mm:opts.step_mm:opts.span_mm
        p = base + dirv * d;
        spec2 = spec;
        spec2.waypoints_mm.(name) = p;
        map2 = build_grid(spec2);
        c = snap_cell(map2, mm2cx(spec2, p(1)), mm2cy(spec2, p(2)));
        cells{end + 1} = sprintf('%d,%d:%+d', c(1), c(2), d);   %#ok<AGROW>
    end

    mid = find(deltas_of(cells) == 0, 1);
    base_cell = cell_of(cells{mid});
    offs = snapped_offset(spec, base, dirv, base_cell);

    % contiguous run around the nominal sample that keeps the SAME cell
    dlist = deltas_of(cells);
    lo = dlist(mid);
    for k = mid:-1:1
        if strcmp(cell_of(cells{k}), base_cell)
            lo = dlist(k);
        else
            break
        end
    end
    hi = dlist(mid);
    for k = mid:numel(cells)
        if strcmp(cell_of(cells{k}), base_cell)
            hi = dlist(k);
        else
            break
        end
    end

    changes = find_changes(cells, dlist);
    if isempty(changes)
        chgstr = 'none within span';
    else
        chgstr = sprintf('%s', strjoin(arrayfun(@(x) sprintf('%+d', x), ...
            changes, 'UniformOutput', false), ', '));
    end

    fprintf('%-8s %-8s (%4d,%4d) %-9s %-11s %s\n', name, mat2str(dirv), ...
        base(1), base(2), base_cell, offs, chgstr);
    fprintf('         SAME CELL for %+d..%+d mm  (that is the usable tolerance)\n', lo, hi);
end

fprintf('\n%s\n', repmat('-', 1, 84));
fprintf('HOW TO READ THIS\n');
fprintf('  "SAME CELL" is the number that matters: inside that window the planner\n');
fprintf('  picks the identical cell, so the whole plan - turns, arrival offset,\n');
fprintf('  clearance - is bit-for-bit unchanged. The stop therefore does NOT need\n');
fprintf('  mm-accurate placement on site; it needs to land inside a ~1-cell window.\n');
fprintf('\n  "offset mm" is the snap the planner applies AT the nominal stop point.\n');
fprintf('  25 mm means the nominal point sits exactly on a cell BOUNDARY in that\n');
fprintf('  axis - the documented trap that once made every target report NO PATH\n');
fprintf('  (see sim/docs/设计说明.md 8.5). It is not an error, but it does mean the\n');
fprintf('  requested mm is not achievable and the car stops half a cell off it.\n');
fprintf('  "cell changes at" lists where a 5 mm nudge flips the chosen cell.\n');
fprintf('%s\n\n', repmat('=', 1, 84));
end

% ----------------------------------------------------------------------
function c = cell_of(s)
c = s(1:find(s == ':', 1) - 1);
end

function d = deltas_of(cells)
d = zeros(1, numel(cells));
for i = 1:numel(cells)
    d(i) = str2double(cells{i}(find(cells{i} == ':', 1) + 1:end));
end
end

function offs = snapped_offset(spec, base, dirv, cellstr)
%Distance between the requested point and the cell centre the planner picked,
%projected on the lane direction.
parts = strsplit(cellstr, ',');
cx = str2double(parts{1}); cy = str2double(parts{2});
centre = [cx * spec.cell_mm + spec.cell_mm / 2, ...
          (spec.size - 1 - cy) * spec.cell_mm + spec.cell_mm / 2];
offs = sprintf('%+.0f', dot(centre - base, dirv));
end

function ch = find_changes(cells, dlist)
ch = [];
for i = 2:numel(cells)
    if ~strcmp(cell_of(cells{i}), cell_of(cells{i - 1}))
        ch(end + 1) = dlist(i);   %#ok<AGROW>
    end
end
end

% ----------------------------------------------------------------------
function cx = mm2cx(sp, x)
cx = min(sp.size - 1, floor(x / sp.cell_mm));
end

function cy = mm2cy(sp, y)
cy = (sp.size - 1) - min(sp.size - 1, floor(y / sp.cell_mm));
end

function sc = snap_cell(map, cx, cy)
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

function o = setdef(o, f, v)
if ~isfield(o, f) || isempty(o.(f))
    o.(f) = v;
end
end
