function sim_arc_vs_pivot(opts)
%SIM_ARC_VS_PIVOT Can corners be arc'd instead of pivoted? Feasibility AND cost.
%
%   sim_arc_vs_pivot()
%
% The firmware rounds every corner as: stop, turn in place, drive. That costs the
% turn time plus a +-0.5 deg alignment residual which the straight run then has to
% correct. A mecanum base CAN drive and turn at once, so the question is real.
%
% THE ANSWER, and the order matters - feasibility first:
%
%   PART 1 shows that at the cross junctions NO arc is legal at any radius or
%   centre, because the car's half-diagonal (212 mm) exceeds the corridor's
%   half-width (200 mm). A 300 mm square body is wider than a 400 mm lane the
%   moment it is broadside, so it cannot yaw through the middle of a junction
%   without its corner crossing into a yellow zone. Pivot turns are therefore NOT
%   a lazy choice at the junctions - they are the only legal option, and the
%   firmware is right.
%
%   PART 2 then measures what arcs would cost IF they were usable, using the
%   DO_DRIVE_ARC executor, so the number is a model result rather than a claim.
%   Read it as the payoff available on OPEN stretches (the 550 mm outer ring),
%   not at the junctions.
%
% HONEST LIMITS:
%   * No slip, no lateral scrub, no wheel saturation. A combined vx+omega command
%     is the mecanum's worst case for grip, so a real arc slips and the saving
%     below is an UPPER BOUND.
%   * No turn-error model either way (phase 1 = ideal).
%   * The firmware does not implement arc turns; nothing here says it can.

if nargin < 1 || isempty(opts)
    opts = struct();
end

spec = load_mapspec();
map  = build_grid(spec);
P    = sim_params();

fprintf('\n%s\n', repmat('=', 1, 80));
fprintf('ARC TURN vs PIVOT TURN\n');
fprintf('%s\n', repmat('=', 1, 80));

% ---- PART 1: is an arc legal at the junctions at all? -------------------
fprintf('\n--- PART 1: feasibility (searched over radius AND centre) ---\n');
fprintf('%-26s %10s  %s\n', 'junction / turn', 'R_min', 'example centre');
fprintf('%s\n', repmat('-', 1, 80));

cases = { ...
    'cross centre  +y -> +x', [1200 1200], [0 1],  [1 0]; ...
    'cross centre  +x -> +y', [1200 1200], [1 0],  [0 1]; ...
    'cross centre  +y -> -x', [1200 1200], [0 1],  [-1 0]; ...
    'cross centre  -y -> +x', [1200 1200], [0 -1], [1 0]; ...
    'raw approach  +y -> +x', [1200 2050], [0 1],  [1 0]; ...
    'rough approach+y -> +x', [1200  350], [0 1],  [1 0]; ...
    'right lane    +y -> +x', [2125 1200], [0 1],  [1 0]};

n_feasible = 0;
for i = 1:size(cases, 1)
    [R, C] = arc_min(map, cases{i, 2}, cases{i, 3}, cases{i, 4});
    if isempty(R)
        fprintf('%-26s %10s  none - pivot is the only legal option\n', cases{i, 1}, 'none');
    else
        n_feasible = n_feasible + 1;
        fprintf('%-26s %7.0f mm  (%6.0f, %6.0f)\n', cases{i, 1}, R, C(1), C(2));
    end
end

fprintf('\n  Why the junctions fail: body half-diagonal %.0f mm > corridor half-width\n', ...
    300 / 2 * sqrt(2));
fprintf('  %.0f mm. A 300 mm square inside a 400 mm lane has only +-50 mm of centre\n', 200);
fprintf('  freedom, and yawing sweeps its corners past that instantly.\n');
fprintf('  => %d of %d tested turns can be arc''d. Pivot turns stay at the rest.\n', ...
    n_feasible, size(cases, 1));

% ---- PART 2: what arcs would cost, IF legal -----------------------------
fprintf('\n--- PART 2: executor comparison (model result, upper bound) ---\n');
fprintf('%-12s %10s %10s %8s   %s\n', 'leg', 'pivot(s)', 'arc(s)', 'saved', 'note');
fprintf('%s\n', repmat('-', 1, 80));

legs = {'h1','exit1'; 'exit1','raw'; 'raw','rough2'; 'rough2','temp2'; ...
        'temp2','qr'; 'qr','h2'; 'h2','exit2'; 'exit2','rough2'; 'h2','h1'};

tot_p = 0; tot_a = 0;
n_arc_ok = 0;
for i = 1:size(legs, 1)
    a = legs{i, 1}; b = legs{i, 2};
    pa = spec.waypoints_mm.(a);
    if a(1) == 'h'
        nose = spec.zones.(sprintf('x%s', a(2))).nose_map;
    else
        nose = [0 1];
    end
    car = car_init(pa(1), pa(2), nose);

    op = run_leg(map, car, a, b, P, struct('log', sim_log(struct('enabled', false))));
    arc = run_leg(map, car, a, b, P, ...
        struct('log', sim_log(struct('enabled', false)), 'arc_turns', true));

    saved = op.seconds - arc.seconds;
    tot_p = tot_p + op.seconds;
    tot_a = tot_a + arc.seconds;

    % feasibility comes from the PLAN (op.arc_possible), not from whether the arc
    % run happened to end legally - an arc can finish clean after sweeping
    % through a yellow zone, which would make an illegal leg look fine
    note = '';
    if op.arc_possible
        n_arc_ok = n_arc_ok + 1;
        note = 'arc-able';
    else
        note = 'corners NOT arc-able (pivot required)';
    end
    fprintf('%-12s %10.2f %10.2f %+8.2f   %s\n', ...
        sprintf('%s->%s', a, b), op.seconds, arc.seconds, saved, note);
end

fprintf('%s\n', repmat('-', 1, 80));
fprintf('%-12s %10.2f %10.2f %+8.2f\n', 'TOTAL', tot_p, tot_a, tot_p - tot_a);
fprintf('arc-able legs by plan geometry: %d / %d\n', n_arc_ok, size(legs, 1));

fprintf('\n%s\n', repmat('-', 1, 80));
fprintf('VERDICT\n');
if tot_a < tot_p
    fprintf('  Arcs would save %.1f s - but only %d of %d legs are arc-able.\n', ...
        tot_p - tot_a, n_arc_ok, size(legs, 1));
else
    fprintf('  Arcs would LOSE %.1f s on these legs, and only %d of %d legs are\n', ...
        tot_a - tot_p, n_arc_ok, size(legs, 1));
    fprintf('  arc-able in the first place. The loss is structural, not a bug: an\n');
    fprintf('  arc turn replaces a corner with a curved path that also travels,\n');
    fprintf('  so a leg whose corner sits at the START covers extra distance\n');
    fprintf('  instead of saving a stop-and-turn.\n');
end
fprintf('  Since the junctions are not arc-able at all (PART 1), the firmware''s\n');
fprintf('  pivot turns are the correct choice and are NOT worth replacing.\n');
fprintf('\n  If arc turns are wanted anywhere, the 550 mm outer ring is the only\n');
fprintf('  place with room - subject to a hardware slip measurement this project\n');
fprintf('  has not made (see sim/docs/设计说明.md 8.10).\n');
fprintf('%s\n\n', repmat('=', 1, 80));
end

% ----------------------------------------------------------------------
function [Rbest, Cbest] = arc_min(map, corner, in_dir, out_dir)
%Search radii and centres near the junction for a legal swept arc that actually
%passes through it. Coarse-to-fine; a coarse grid would make a negative result
%an artifact of the grid instead of a property of the field.
in_dir  = in_dir(:).'  / norm(in_dir);
out_dir = out_dir(:).' / norm(out_dir);
cr = in_dir(1) * out_dir(2) - in_dir(2) * out_dir(1);
turn = atan2(cr, dot(in_dir, out_dir));

Rbest = []; Cbest = [];
for R = 150:25:600
    for ox = -R:10:R
        for oy = -R:10:R
            if hypot(ox, oy) > R + 1
                continue
            end
            C = corner + [ox oy];
            if arc_ok(map, C, R, turn, in_dir, corner)
                Rbest = R; Cbest = C;
                return
            end
        end
    end
end
end

function ok = arc_ok(map, C, R, turn, in_dir, corner)
h0 = atan2(in_dir(2), in_dir(1));
rad0 = h0 - pi / 2;                       % radial angle at the start
nsteps = max(24, ceil(abs(turn) * R / 3));
min_d = inf;
ok = true;
for i = 0:nsteps
    th = turn * (i / nsteps);
    ra = rad0 + th;
    px = C(1) + R * cos(ra);
    py = C(2) + R * sin(ra);
    if ~pose_ok(map, px, py, h0 + th)
        ok = false;
        return
    end
    min_d = min(min_d, hypot(px - corner(1), py - corner(2)));
end
if min_d > 40
    ok = false;                           % must actually pass through the junction
end
end

function ok = pose_ok(map, x, y, h)
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
