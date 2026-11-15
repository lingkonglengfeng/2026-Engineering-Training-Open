function r = sim_report_exec(opts)
%SIM_REPORT_EXEC Run the 9 task legs and return metrics. No printing, no plots.
%
%   r = SIM_REPORT_EXEC(struct('turn_cost', 240, 'lane_cost', 0))
%
% Shared engine behind SIM_REPORT (prints) and SIM_SCAN (sweeps). Both must go
% through here, otherwise a swept number and a reported number can silently
% differ - the kind of drift that makes a tuning table worthless.
%
% opts
%   .speed_scale      firmware `spd`
%   .time_1000mm_ms   firmware `t1000`
%   .counts_per_rev   firmware `cnt`
%   .turn_cost        A* corner penalty   (default: mapspec value)
%   .lane_cost        A* lane-centre bias (default: mapspec value)
%   .overrides        extra SIM_PARAMS overrides
%   .legs             cell Nx2 of waypoint names (default: the 9 mission legs)
%
% r
%   .seconds .turns .strafes .legs_ok .n_legs .n_nopath
%   .worst_dev_mm     worst |actual stop - planned cell centre|
%   .min_clearance_mm tightest body-to-forbidden-rect distance
%   .min_band_mm      tightest body-to-field-edge margin
%   .legal            no body overlap with a forbidden rect / off-mat / locked zone
%   .legs             per-leg detail (actions, samples, verdict)

if nargin < 1 || isempty(opts)
    opts = struct();
end

% Reject unknown option keys. Silently ignoring one turns a sweep into a
% no-op that still reports plausible numbers - exactly how a `t1000` field
% named differently from the one read here made it look like t1000 had no
% effect on anything. Cheap check, expensive bug.
known = {'speed_scale', 'time_1000mm_ms', 'counts_per_rev', 'position_mode', ...
         'turn_cost', 'lane_cost', 'overrides', 'legs', 'wp_override'};
fn = fieldnames(opts);
bad = setdiff(fn, known);
if ~isempty(bad)
    error('sim_report_exec:unknownOpt', ...
        'unknown option(s): %s. Known: %s', strjoin(bad, ', '), strjoin(known, ', '));
end

% Move a named waypoint (the "停位" sweep). Applied to the spec BEFORE the map is
% built so planning, execution and legality all see the moved stop point.
if isfield(opts, 'wp_override') && ~isempty(opts.wp_override)
    names = fieldnames(opts.wp_override);
    for i = 1:numel(names)
        if ~isfield(spec.waypoints_mm, names{i})
            error('sim_report_exec:wp', 'unknown waypoint "%s"', names{i});
        end
        spec.waypoints_mm.(names{i}) = opts.wp_override.(names{i});
    end
end

spec = load_mapspec();

% Cost model must be applied BEFORE the map is built: astar_grid reads the cost
% from map.spec, so mutating `spec` afterwards has no effect on planning at all.
% (That bug made every turn_cost / lane_cost sweep return identical numbers -
% the sweep looked like "these knobs do nothing" when it was simply not wired.)
if isfield(opts, 'turn_cost') && ~isempty(opts.turn_cost)
    spec.astar.turn_cost = opts.turn_cost;
end
if isfield(opts, 'lane_cost') && ~isempty(opts.lane_cost)
    spec.astar.lane_cost_per_cell = opts.lane_cost;
end
map = build_grid(spec);

% parameter overrides: only real SIM_PARAMS keys may reach SIM_PARAMS
over = struct();
for f = {'speed_scale', 'time_1000mm_ms', 'counts_per_rev', 'position_mode'}
    if isfield(opts, f{1})
        over.(f{1}) = opts.(f{1});
    end
end
if isfield(opts, 'overrides') && ~isempty(opts.overrides)
    of = fieldnames(opts.overrides);
    for i = 1:numel(of)
        over.(of{i}) = opts.overrides.(of{i});
    end
end
P = sim_params(over);

if isfield(opts, 'legs') && ~isempty(opts.legs)
    legs = opts.legs;
else
    legs = {'h1','exit1'; 'exit1','raw'; 'raw','rough2'; 'rough2','temp2'; ...
            'temp2','qr'; 'qr','h2'; 'h2','exit2'; 'exit2','rough2'; 'h2','h1'};
end

rows = struct([]);
n_nopath = 0;
for i = 1:size(legs, 1)
    a = legs{i, 1};
    b = legs{i, 2};
    pa = spec.waypoints_mm.(a);
    pb = spec.waypoints_mm.(b);

    % Start pose: a leg beginning in a start zone starts at the zone centre with
    % the zone's nose; anything else starts at its own waypoint. A leg is a
    % RELATIVE action sequence, so the starting pose is part of the experiment.
    if a(1) == 'h'
        zone = a(2);
        nose = spec.zones.(sprintf('x%s', zone)).nose_map;
    else
        nose = [0 1];
    end
    car = car_init(pa(1), pa(2), nose);

    log = sim_log(struct('enabled', false));
    out = run_leg(map, car, a, b, P, struct('log', log));

    v = out.verdict;
    % body-to-field-edge margin
    br = v.body_rect;
    band = min([br(1), br(2), spec.field_mm - br(3), spec.field_mm - br(4)]);

    row = struct('from', a, 'to', b, ...
                 'cells', max(size(out.path, 1) - 1, 0), ...
                 'segs', out.n_segments, 'turns', out.turns, 'strafes', out.strafes, ...
                 'seconds', out.seconds, 'dev_mm', out.err_mm, ...
                 'clearance_mm', v.min_clearance_mm, 'band_mm', band, ...
                 'legal', v.ok, 'no_path', isempty(out.path), ...
                 'actions', {out.actions});
    if isempty(rows)
        rows = row;
    else
        rows(end + 1) = row;   %#ok<AGROW>
    end
    n_nopath = n_nopath + double(isempty(out.path));
end

r = struct();
r.legs = rows;
r.n_legs = numel(rows);
r.n_nopath = n_nopath;
r.legs_ok = sum([rows.dev_mm] < 60 & ~[rows.no_path]);
r.seconds = sum([rows.seconds]);
r.turns = sum([rows.turns]);
r.strafes = sum([rows.strafes]);
r.worst_dev_mm = max([rows.dev_mm]);
r.min_clearance_mm = min([rows.clearance_mm]);
r.min_band_mm = min([rows.band_mm]);
r.legal = all([rows.legal]);
r.speed_scale = P.speed_scale;
r.t1000 = P.time_1000mm_ms;
r.turn_cost = spec.astar.turn_cost;
r.lane_cost = spec.astar.lane_cost_per_cell;
r.P = P;
r.spec = spec;
end
