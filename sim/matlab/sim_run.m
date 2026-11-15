function [car, res, log, map] = sim_run(cmd, opts)
%SIM_RUN Run ONE navigation command in simulation. The single-command entry point.
%
%   sim_run('goto raw')
%   sim_run('goto qr', struct('speed_scale', 0.75))
%   [car, res, log, map] = sim_run('h2->qr')
%
% Supported commands (mirroring the firmware's `&` commands):
%   'goto <name>'      named waypoint, e.g. goto raw / rough2 / temp3 / qr / center
%   'goto <x> <y>'     explicit MAP mm
%   'h2->qr'           convenience: run a leg between two waypoints
%   'startpose 1|2'    place the car in start zone 1 (bottom-right) / 2 (top-right)
%   'leave'            zone exit: forward 300 mm then strafe 125 mm
%   'home'             return to the start zone: strafe 125 mm then back 300 mm
%   'cross'            drive from the current start zone to the other one
%   'nudge u|d|l|r [mm]'  heading self-check along a MAP direction
%   'anim'             (option) play the animation; also 'anim' in `opts`
%
% opts (all optional)
%   .speed_scale .time_1000mm_ms .counts_per_rev   firmware `spd`/`t1000`/`cnt`
%   .anim      true to animate            .axes  target axes for the drawing
%   .quiet     true to suppress the log echo
%   .overrides struct passed straight to SIM_PARAMS
%
% Returns
%   car   final car state
%   res   the RUN_LEG result (see run_leg.m), or a script result for the
%         zone commands
%   log   the SIM_LOG struct (use sim_run_print(log))
%   map   the built grid

if nargin < 1 || isempty(cmd)
    error('sim_run:cmd', 'usage: sim_run(''goto raw'')');
end
if nargin < 2 || isempty(opts)
    opts = struct();
end

over = struct();
for f = {'speed_scale', 'time_1000mm_ms', 'counts_per_rev'}
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

spec = load_mapspec();
map = build_grid(spec);

log = sim_log();

tokens = split(strtrim(cmd));
verb = lower(tokens{1});

anim = isfield(opts, 'anim') && opts.anim;
ax = [];
if isfield(opts, 'axes') && ~isempty(opts.axes)
    ax = opts.axes;
end

switch verb
    case {'goto', 'go'}
        [start_mm, zone] = default_start(spec);
        car = car_init(start_mm(1), start_mm(2), zone_nose(spec, zone));
        if numel(tokens) < 2
            error('sim_run:usage', 'usage: goto <name|x y>');
        end
        if numel(tokens) == 2
            goal = tokens{2};
        else
            goal = [str2double(tokens{2}), str2double(tokens{3})];
        end
        res = run_leg(map, car, car_start_name(zone), goal, P, struct('log', log));

    case 'cross'
        car = car_init(2250, 2250, zone_nose(spec, 2));
        res = run_script(map, car, 'cross', P, log);

    case 'leave'
        car = car_init(2250, 2250, zone_nose(spec, 2));
        res = run_script(map, car, 'leave', P, log);

    case 'home'
        car = car_init(str2double(tokens{2}), str2double(tokens{3}), [0 -1]);
        res = run_script(map, car, 'home', P, log);

    case 'startpose'
        if numel(tokens) < 2
            error('sim_run:usage', 'usage: startpose 1|2');
        end
        zone = str2double(tokens{2});
        start_mm = spec.zones.(sprintf('%d', zone)).center_mm;
        car = car_init(start_mm(1), start_mm(2), zone_nose(spec, zone));
        res = struct('actions', {{}}, 'seconds', 0, 'zone', zone, ...
                     'start_mm', start_mm, 'end_mm', start_mm);

    case 'nudge'
        car = car_init(2250, 2250, zone_nose(spec, 2));
        if numel(tokens) < 2
            error('sim_run:usage', 'usage: nudge u|d|l|r [mm]');
        end
        mm = 100;
        if numel(tokens) >= 3
            mm = str2double(tokens{3});
        end
        res = run_nudge(map, car, lower(tokens{2}), mm, P, log);

    otherwise
        % allow the 'a->b' shorthand
        if contains(cmd, '->')
            parts = split(cmd, '->');
            a = strtrim(parts{1}); b = strtrim(parts{2});
            car = car_init(0, 0, [0 1]);
            p = spec.waypoints_mm.(char(a));
            car = car_init(p(1), p(2), [0 1]);
            res = run_leg(map, car, char(a), char(b), P, struct('log', log));
        else
            error('sim_run:cmd', 'unknown command "%s"', cmd);
        end
end

log = res.log;
car = res.car;
res.P = P;

if anim
    h = car_draw(map, car, struct('axes', ax));
    animate(h, map, car, res);
end

if ~isfield(opts, 'quiet') || ~opts.quiet
    sim_run_print(log);
end
end

% ----------------------------------------------------------------------
function [start_mm, zone] = default_start(spec)
%The team's launch zone. Start zone 2 (top-right) is the firmware default
%(ZDT_START_ZONE_DEFAULT); start zone 1 is the bottom-right one.
zone = 2;
start_mm = spec.zones.(sprintf('%d', zone)).center_mm;
end

function name = car_start_name(zone)
name = sprintf('h%d', zone);
end

function nose = zone_nose(spec, zone)
nose = spec.zones.(sprintf('%d', zone)).nose_map;
end

function res = run_script(map, car, which, P, log)
%The zone exit/return/cross scripts, as step tables (see docs, section 4).
%These are RELATIVE actions, so getting the entry zone wrong leaves the car half
%off the mat while the log still looks perfect - hence the pose gate.
spec = map.spec;
LEAVE = spec.zones.leave_mm;
STRAFE = spec.zones.strafe_mm;

switch which
    case 'leave'
        steps = {'FWD', LEAVE, 'L'; 'STRAFE', STRAFE, 'L'};
    case 'home'
        steps = {'STRAFE', STRAFE, 'R'; 'BACK', LEAVE, 'R'};
    case 'cross'
        steps = {'FWD', LEAVE, 'L'; 'STRAFE', STRAFE, 'L'; ...
                 'FWD', 1500, 'L'; 'TURN', 180, 'L'; ...
                 'STRAFE', STRAFE, 'R'; 'BACK', LEAVE, 'R'};
    otherwise
        error('run_script:which', 'unknown script "%s"', which);
end

log = sim_log_add(log, '[NAV] script: %s', which);
actions = {};
samples = struct([]);

for i = 1:size(steps, 1)
    kind = steps{i, 1};
    amount = steps{i, 2};
    dir = steps{i, 3};
    switch kind
        case 'FWD'
            [car, samples, log] = do_drive(car, amount, 1, P.cmd_period_ms / 1000, P, ...
                struct(), log, samples, i);
        case 'BACK'
            [car, samples, log] = do_drive(car, amount, -1, P.cmd_period_ms / 1000, P, ...
                struct(), log, samples, i);
        case 'STRAFE'
            s = 1; if dir == 'L', s = -1; end
            [car, samples, log] = do_strafe(car, amount, s, P.cmd_period_ms / 1000, P, ...
                struct(), log, samples, i);
        case 'TURN'
            [car, samples, log] = do_turn(car, deg2rad(amount), P.cmd_period_ms / 1000, ...
                P, log, samples, i);
    end
    actions{end + 1} = sprintf('%s %g', kind, amount);   %#ok<AGROW>
    log = sim_log_add(log, '[NAV] script %u/%u: %s %gmm', i, size(steps, 1), kind, amount);
end

log = sim_log_add(log, '[NAV] script done: pose=(%umm,%umm)', ...
    round(car.x), round(car.y));

res = struct();
res.car = car;
res.actions = actions;
res.seconds = car.t;
res.end_mm = [car.x, car.y];
res.samples = samples;
res.log = log;
res.verdict = check_legality(map, car);
res.path = zeros(0, 2);
res.turns = sum(strcmp(steps(:, 1), 'TURN'));
res.strafes = sum(strcmp(steps(:, 1), 'STRAFE'));
res.n_segments = size(steps, 1);
res.snapped = false;
res.arrived = true;
end

function res = run_nudge(map, car, dir, mm, P, log)
%`&nudge`: drive a short distance along a MAP direction, to prove the nose
%binding. Deliberately uses the same map->body mapping as navigation, so a
%wrong placement shows up as motion in the wrong MAP direction.
switch dir
    case 'u', want = [0 1];
    case 'd', want = [0 -1];
    case 'l', want = [-1 0];
    case 'r', want = [1 0];
    otherwise
        error('run_nudge:dir', 'usage: nudge u|d|l|r [mm]');
end
F = [cos(car.h), sin(car.h)];
if abs(dot(want, F)) > 0.9
    start_h = car.h;
    car = car_init(car.x, car.y, want);
    [car, samples, log] = do_drive(car, mm, 1, P.cmd_period_ms / 1000, P, ...
        struct(), log, struct([]), 1);
else
    start_h = car.h;
    car = car_init(car.x, car.y, want);
    [car, samples, log] = do_drive(car, mm, 1, P.cmd_period_ms / 1000, P, ...
        struct(), log, struct([]), 1);
end
log = sim_log_add(log, '[NAV] nudge %c %umm: expect the car to move toward map %s', ...
    dir, mm, mat2str(want));

res = struct('car', car, 'actions', {{sprintf('nudge %c %d', dir, mm)}}, ...
             'seconds', car.t, 'end_mm', [car.x car.y], 'samples', samples, ...
             'log', log, 'verdict', check_legality(map, car), ...
             'path', zeros(0, 2), 'turns', 0, 'strafes', 0, ...
             'n_segments', 1, 'snapped', false, 'arrived', true);
res.start_h = start_h;
end

function animate(h, map, car, res)
%Replay the recorded samples so the animation shows exactly what was simulated.
if isempty(res.samples)
    return
end
s = res.samples;
for i = 1:numel(s)
    car.x = s(i).x; car.y = s(i).y; car.h = s(i).h;
    car.wheel_rpm = s(i).rpm;
    car_draw_update(h, car);
    drawnow limitrate;
end
end
