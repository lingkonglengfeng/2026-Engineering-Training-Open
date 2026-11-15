function out = run_leg(map, car, start_mm, goal_mm, P, opts)
%RUN_LEG Plan and execute one navigation leg in simulation.
%
%   out = RUN_LEG(map, car, 'raw', 'qr', P)
%   out = RUN_LEG(map, car, [1200 2050], [2125 1200], P, struct('log', log))
%
%   map, car   from BUILD_GRID / CAR_INIT   (car is advanced in place)
%   start_mm   where the car is now: a waypoint NAME or [x y] mm
%   goal_mm    destination:          a waypoint NAME or [x y] mm
%   P          from SIM_PARAMS
%   opts.log   a SIM_LOG to append to (optional)
%   opts.dt    integration step, default P.cmd_period_ms/1000
%   opts.err   physics-error hook for WHEEL_FORWARD (phase 2; ideal by default)
%
% Executor model, replicated from ZDT_Nav_Service() in Core/Src/zdt_motor.c:
%
%   for each A* segment:
%     * SAME direction as the nose            -> drive straight
%     * PERPENDICULAR, <= 3 cells (150 mm)    -> STRAFE, nose unchanged
%     * otherwise (corner, reversal, long jog)-> turn in place, then drive
%
%   In-place turns use CHASSIS_TURN_FAST_WHEEL_RPM (120) and finish in ~0.8 s
%   for 90 deg. Strafing keeps the nose fixed, so it costs no turn AND adds no
%   alignment residual - which is why the firmware prefers it for short jogs.
%
% out fields
%   .actions        {1xM} the executed action strings ("turn+go U1600", ...)
%   .segments       Mx3 [letter cells mm]
%   .turns .strafes counts
%   .n_segments     number of A* segments
%   .seconds        total simulated time for the leg
%   .path           Nx2 cell path (0-based)
%   .start_mm .goal_mm .end_mm  actual final pose
%   .start_cell .goal_cell .end_cell
%   .snapped        true if an endpoint had to be snapped to a legal cell
%   .samples        struct array of the time series (t, x, y, h, rpm, ...)
%   .log            the log after appending

if nargin < 6 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'dt') || isempty(opts.dt)
    dt = P.cmd_period_ms / 1000;
else
    dt = opts.dt;
end
if ~isfield(opts, 'err') || isempty(opts.err)
    err = struct();
else
    err = opts.err;
end
if ~isfield(opts, 'log') || isempty(opts.log)
    log = sim_log();
else
    log = opts.log;
end

s_mm = resolve_point(map.spec, start_mm);
g_mm = resolve_point(map.spec, goal_mm);

% NOTE: map_coords returns TWO outputs. Assigning it to a single name silently
% keeps only cx, so every later cell(1)/cell(2) read fails or reads nonsense -
% the surface symptom was "too many output arguments" inside snap_to_walkable.
[s_cx, s_cy] = map_coords('mm2cell_clamped', map.spec, s_mm(1), s_mm(2));
[g_cx, g_cy] = map_coords('mm2cell_clamped', map.spec, g_mm(1), g_mm(2));
s_cell = [s_cx, s_cy];
g_cell = [g_cx, g_cy];

[path, info] = astar_grid(map, s_cell, g_cell);
log = sim_log_add(log, '[NAV] EXEC start.');

out = struct('actions', {{}}, 'segments', zeros(0, 3), 'turns', 0, 'strafes', 0, ...
             'n_segments', 0, 'seconds', 0, 'path', path, ...
             'start_mm', s_mm, 'goal_mm', g_mm, 'end_mm', s_mm, ...
             'start_cell', s_cell, 'goal_cell', g_cell, 'end_cell', s_cell, ...
             'snapped', info.snapped, 'samples', struct([]), 'log', log, ...
             'arrived', false, 'car', car, 'verdict', struct(), 'err_mm', NaN);

if isempty(path)
    log = sim_log_add(log, '[NAV] no executable path.');
    out.log = log;
    out.verdict = check_legality(map, car);
    return
end
if numel(path) == 1
    log = sim_log_add(log, '[NAV] ARRIVED (already at target).');
    out.arrived = true;
    out.log = log;
    out.verdict = check_legality(map, car);
    out.err_mm = norm([car.x, car.y] - g_mm);
    return
end

segs = path_segments(path, map.spec);
plan = plan_maneuvers(segs, map.spec);
out.segments = segs;
out.turns = plan.turns;
out.strafes = plan.strafes;
out.actions = plan.actions;
out.n_segments = size(segs, 1);

% Is this leg arc-turnable at all? A leg's arc feasibility is a property of the
% PLAN, not of the run, so it is computed here from the corner geometry rather
% than inferred from whether the arc run happened to end somewhere legal. The
% end pose is a weak proxy: an arc can finish legally after sweeping through a
% yellow zone. See SIM_ARC_VS_PIVOT for the geometric search behind this.
out.arc_possible = arc_turns_possible(map, path, segs);

samples = struct([]);
nose = car.h;                       % current heading angle, map frame

for k = 1:size(segs, 1)
    L     = char(segs(k, 1));
    cells = segs(k, 2);
    mm    = segs(k, 3);

    bearing = letter_to_heading(L);          % map-frame direction to travel
    seg_vec = [cos(bearing), sin(bearing)];

    % body-frame components of this segment under the CURRENT nose
    F = [cos(nose), sin(nose)];
    R = [-F(2), F(1)];             % firmware literal form; see car_step.m for why R = (-F.y, F.x)
    fwd_comp   = dot(seg_vec, F);
    right_comp = dot(seg_vec, R);

    is_strafe = abs(fwd_comp) < 1e-9 && cells <= plan_maneuvers_strafe_max(map.spec);
    use_arc = isfield(opts, 'arc_turns') && opts.arc_turns;

    if is_strafe
        sign_y = sign(right_comp);
        if sign_y == 0
            sign_y = 1;
        end
        [car, samples, log] = do_strafe(car, mm, sign_y, dt, P, err, log, samples, k);
    else
        % align to the travel direction unless already aligned
        target_h = bearing;
        diff = wrap_pi(target_h - nose);
        if use_arc && abs(diff) > deg2rad(P.align_tolerance_deg)
            % ARC: drive the segment WHILE turning onto its bearing, instead of
            % stopping, turning in place, then driving. The firmware cannot do
            % this today (it has no arc executor); see sim_arc_vs_pivot.m.
            [car, samples, log] = do_drive_arc(car, mm, diff, dt, P, err, log, samples, k);
            nose = car.h;
        else
            if abs(diff) > deg2rad(P.align_tolerance_deg)
                [car, samples, log] = do_turn(car, diff, dt, P, log, samples, k);
                nose = car.h;
            end
            F = [cos(nose), sin(nose)];
            sgn = sign(dot(seg_vec, F));
            if sgn == 0
                sgn = 1;
            end
            [car, samples, log] = do_drive(car, mm, sgn, dt, P, err, log, samples, k);
        end
    end

    % ---- segment done: identical bookkeeping/logging to the firmware ------
    if is_strafe
        log = sim_log_add(log, '[NAV] SEG%u done (strafe %.0fmm, no turn)', k, mm);
    else
        revs = mm / P.wheel_circumference_mm;
        log = sim_log_add(log, '[NAV] SEG%u done: %.3f/%.3frev (hold peak %.2f deg)', ...
            k, revs, revs, 0.00);
    end
    [e_cx, e_cy] = map_coords('mm2cell_clamped', map.spec, car.x, car.y);
    e_cell = [e_cx, e_cy];
    log = sim_log_add(log, '[NAV] pose now (%umm,%umm) cell=(%u,%u) after SEG%u', ...
        round(car.x), round(car.y), e_cell(1), e_cell(2), k);

    out.end_cell = e_cell;
end

log = sim_log_add(log, '[NAV] ARRIVED.');

% final micro-correction: drive the last few mm to the exact target mm, which is
% the firmware's `correct` pass after the segment loop.
%
% Only do this when the requested goal was itself a LEGAL car centre. Start-zone
% centres are deliberately NOT legal (the body would hang off the mat), so the
% planner snaps them; driving on to the raw point then puts the car off the field
% - that produced "end err 2100 mm, off-field" on the h2 legs. When the goal was
% snapped, stop at the snapped cell centre: it is the closest legal pose.
if ~info.snapped
    % Keep everything a ROW vector: g_mm / s_mm are 1x2, so mixing in a column
    % would silently make `final` 2x2 and blow up the dot() calls below.
    final = [g_mm(1) - car.x, g_mm(2) - car.y];
    if norm(final) > 1.0
        F = [cos(car.h), sin(car.h)];
        R = [-F(2), F(1)];             % firmware literal form; see car_step.m
        d_fwd = dot(final, F);
        d_rgt = dot(final, R);
        [car, samples] = do_drive(car, d_fwd, sign(d_fwd), dt, P, err, log, samples, 0, false);
        [car, samples] = do_strafe(car, abs(d_rgt), sign(d_rgt), dt, P, err, log, samples, 0);
    end
end

out.end_mm = [car.x, car.y];
out.seconds = car.t;
out.samples = samples;
out.arrived = true;
out.log = log;
out.car = car;
% Arrival error. Report the two numbers separately, because they mean different
% things and one of them is NOT an error at all:
%   err_mm        distance from where the car stopped to the PLANNED endpoint
%                 (the snapped cell centre when the raw goal was illegal)
%   err_vs_goal_mm distance to the raw requested mm point
%   snapped_off_mm how far the raw goal had to be moved to become legal - this
%                 is a property of the FIELD (start-zone centres are not legal
%                 car centres), NOT a driving error. Folding it into a single
%                 "err" number made a 2 mm arrival look like an 850 mm failure.
planned = cell_centre_mm(map.spec, info.goal);
out.err_mm = hypot(car.x - planned(1), car.y - planned(2));
out.err_vs_goal_mm = hypot(car.x - g_mm(1), car.y - g_mm(2));
out.snapped_off_mm = hypot(planned(1) - g_mm(1), planned(2) - g_mm(2));
% final legality verdicts (the claims a report must make explicitly)
out.verdict = check_legality(map, car);
end

function xy = cell_centre_mm(spec, cellc)
xy = [cellc(1) * spec.cell_mm + spec.cell_mm / 2, ...
      (spec.size - 1 - cellc(2)) * spec.cell_mm + spec.cell_mm / 2];
end

% ----------------------------------------------------------------------
function p = resolve_point(spec, pt)
%Accept a waypoint name or an [x y] mm pair.
if ischar(pt) || isstring(pt)
    name = char(pt);
    if ~isfield(spec.waypoints_mm, name)
        error('run_leg:waypoint', 'unknown waypoint "%s"', name);
    end
    p = spec.waypoints_mm.(name);
else
    p = pt(:).';
    if numel(p) ~= 2
        error('run_leg:point', 'point must be [x y] mm or a waypoint name');
    end
end
end

function h = letter_to_heading(L)
%Segment letter -> map-frame bearing. CAUTION: cell row cy grows DOWNWARD, so
%'D' means map -y. Swapping these is the documented "D/U reversed in the log" bug.
switch L
    case 'U', h =  pi / 2;      % map +y
    case 'D', h = -pi / 2;      % map -y
    case 'R', h =  0;           % map +x
    case 'L', h =  pi;          % map -x
    otherwise
        error('run_leg:letter', 'bad segment letter "%s"', L);
end
end

function m = plan_maneuvers_strafe_max(spec)
m = spec.executor.strafe_max_cells;
end

function tf = arc_turns_possible(map, path, segs)
%True when EVERY corner of the plan can be swept as an arc instead of a pivot.
%Delegates to the same feasibility search SIM_ARC_VS_PIVOT uses, so the two
%cannot disagree. A leg with no corners is trivially arc-able.
tf = true;
if size(path, 1) < 2
    return
end
for k = 2:size(path, 1)
    dx0 = path(k - 1, 1) - path(max(k - 2, 1), 1);
    dy0 = path(k - 1, 2) - path(max(k - 2, 1), 2);
    dx1 = path(k, 1) - path(k - 1, 1);
    dy1 = path(k, 2) - path(k - 1, 2);
    if k == 2
        continue            % no incoming direction yet at the first cell
    end
    if (dx0 == dx1) && (dy0 == dy1)
        continue            % not a corner
    end
    a = seg_letter_vec(dx0, dy0);
    b = seg_letter_vec(dx1, dy1);
    if abs(dot(a, b) + 1) < 1e-9
        tf = false;         % reversal: never arc-able
        return
    end
    corner = cell_xy_mm(map.spec, path(k - 1, 1), path(k - 1, 2));
    if isnan(arc_turn_min_radius(map, corner, a, b))
        tf = false;
        return
    end
end
end

function v = seg_letter_vec(dx, dy)
%Cell step -> unit vector in the MAP frame. Row cy grows DOWNWARD, so dy>0 is -y.
v = [sign(dx), -sign(dy)];
v = v / norm(v);
end

function xy = cell_xy_mm(spec, cx, cy)
xy = [cx * spec.cell_mm + spec.cell_mm / 2, ...
      (spec.size - 1 - cy) * spec.cell_mm + spec.cell_mm / 2];
end

function R = arc_turn_min_radius(map, corner, in_dir, out_dir)
%Thin wrapper so RUN_LEG and SIM_ARC_VS_PIVOT share one implementation.
R = arc_min(map, corner, in_dir, out_dir);
end

function [Rbest, Cbest] = arc_min(map, corner, in_dir, out_dir)
in_dir  = in_dir(:).'  / norm(in_dir);
out_dir = out_dir(:).' / norm(out_dir);
cr = in_dir(1) * out_dir(2) - in_dir(2) * out_dir(1);
if abs(cr) < 1e-9
    Rbest = NaN; Cbest = []; return
end
turn = atan2(cr, dot(in_dir, out_dir));
Rbest = NaN; Cbest = [];
for R = 150:25:600
    for ox = -R:10:R
        for oy = -R:10:R
            if hypot(ox, oy) > R + 1
                continue
            end
            C = corner + [ox oy];
            if arc_sweep_ok(map, C, R, turn, in_dir, corner)
                Rbest = R; Cbest = C;
                return
            end
        end
    end
end
end

function ok = arc_sweep_ok(map, C, R, turn, in_dir, corner)
h0 = atan2(in_dir(2), in_dir(1));
rad0 = h0 - pi / 2;
nsteps = max(24, ceil(abs(turn) * R / 3));
min_d = inf;
ok = true;
for i = 0:nsteps
    th = turn * (i / nsteps);
    ra = rad0 + th;
    px = C(1) + R * cos(ra);
    py = C(2) + R * sin(ra);
    if ~arc_pose_ok(map, px, py, h0 + th)
        ok = false;
        return
    end
    min_d = min(min_d, hypot(px - corner(1), py - corner(2)));
end
if min_d > 40
    ok = false;
end
end

function ok = arc_pose_ok(map, x, y, h)
spec = map.spec;
half = spec.car_mm / 2;
F = [cos(h), sin(h)];
Rt = [-F(2), F(1)];
ex = half * (abs(F(1)) + abs(Rt(1)));
ey = half * (abs(F(2)) + abs(Rt(2)));
x0 = x - ex; x1 = x + ex; y0 = y - ey; y1 = y + ey;
if x0 < 0 || y0 < 0 || x1 > spec.field_mm || y1 > spec.field_mm
    ok = false; return
end
rr = blocked_rects(spec);
for i = 1:size(rr, 1)
    r = rr(i, :);
    if x0 < r(2) && x1 > r(1) && y0 < r(4) && y1 > r(3)
        ok = false; return
    end
end
ok = true;
end

function a = wrap_pi(a)
a = mod(a + pi, 2 * pi) - pi;
end
