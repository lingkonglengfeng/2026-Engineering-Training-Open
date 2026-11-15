function test_zone_scripts()
%TEST_ZONE_SCRIPTS Ground-truth check of the nose binding and strafe direction.
%
% Run:  matlab -batch "cd('E:\Engineering_Training_Project\sim\matlab'); test_zone_scripts"
%
% These are the strongest available end-to-end checks, because every expected
% number is an INDEPENDENTLY RECORDED fact rather than a re-derivation of the
% same formulas:
%
%   * tools/check_map.py end-position self-check (`&cross`)
%   * docs/hardware/底盘方向与麦轮排布说明.md §7.3: from start zone 2 (nose map -y)
%     map +x lies on the car's RIGHT, so a 125 mm right strafe must move map -x.
%
% A mirrored right vector passes every pure-body test and fails exactly here.

spec = load_mapspec();
map  = build_grid(spec);
P    = sim_params();
W    = spec.waypoints_mm;

fails = 0;

% --- zone 2 (the team's launch zone), nose into the field -----------------
z2 = spec.zones.x2;
fprintf('\n--- zone 2 exit script (nose map [%d %d]) ---\n', z2.nose_map(1), z2.nose_map(2));
car = car_init(z2.center_mm(1), z2.center_mm(2), z2.nose_map);res = run_script_local(map, car, 'leave', P);
fprintf('   start (%d,%d) -> end (%d,%d)\n', ...
    round(z2.center_mm(1)), round(z2.center_mm(2)), round(res.car.x), round(res.car.y));
fails = fails + expect(abs(res.car.x - z2.exit_mm(1)) < 2 && ...
                       abs(res.car.y - z2.exit_mm(2)) < 2, ...
    sprintf('&leave ends at exit2 (%d,%d)', z2.exit_mm(1), z2.exit_mm(2)), ...
    sprintf('(%d,%d)', round(res.car.x), round(res.car.y)));

fprintf('   x moved %+d mm (must NEGATIVE: right strafe = map -x for nose -y)\n', ...
    round(res.car.x - z2.center_mm(1)));
fails = fails + expect(res.car.x < z2.center_mm(1), ...
    'right strafe moves map -x (nose map -y)', sprintf('dx = %d', ...
    round(res.car.x - z2.center_mm(1))));

% --- zone 1, nose map +y: map +x is on the LEFT, so the exit strafes LEFT --
z1 = spec.zones.x1;
fprintf('\n--- zone 1 exit script (nose map [%d %d]) ---\n', z1.nose_map(1), z1.nose_map(2));
car = car_init(z1.center_mm(1), z1.center_mm(2), z1.nose_map);
res = run_script_local(map, car, 'leave', P);
fprintf('   start (%d,%d) -> end (%d,%d)\n', ...
    round(z1.center_mm(1)), round(z1.center_mm(2)), round(res.car.x), round(res.car.y));
fails = fails + expect(abs(res.car.x - z1.exit_mm(1)) < 2 && ...
                       abs(res.car.y - z1.exit_mm(2)) < 2, ...
    sprintf('&leave ends at exit1 (%d,%d)', z1.exit_mm(1), z1.exit_mm(2)), ...
    sprintf('(%d,%d)', round(res.car.x), round(res.car.y)));

% --- &cross: zone 2 -> zone 1, straight 1500 mm down the right lane -------
fprintf('\n--- &cross zone 2 -> zone 1 ---\n');
car = car_init(z2.center_mm(1), z2.center_mm(2), z2.nose_map);
res = run_script_local(map, car, 'cross', P);
fprintf('   end (%d,%d)  expected (%d,%d)\n', round(res.car.x), round(res.car.y), ...
    z1.center_mm(1), z1.center_mm(2));
fails = fails + expect(abs(res.car.x - z1.center_mm(1)) < 2 && ...
                       abs(res.car.y - z1.center_mm(2)) < 2, ...
    '&cross ends in the other start-zone centre', ...
    sprintf('(%d,%d)', round(res.car.x), round(res.car.y)));
% the car must never leave the mat or drop into a yellow zone on the way
fails = fails + expect(res.verdict.ok, '&cross stays legal the whole way', ...
    sprintf('blocked=%d pad=%d off=%d', res.verdict.hit_blocked, ...
    res.verdict.hit_pad, res.verdict.off_field));

% --- a plain A* leg through run_leg (the executed path, not just the plan) --
% This is the check that separates "the plan is right" from "the car got there".
fprintf('\n--- run_leg executed legs (plan vs actual arrival) ---\n');
legs = {'h2', 'exit2'; 'exit1', 'raw'; 'raw', 'rough2'; ...
        'rough2', 'temp2'; 'temp2', 'qr'; 'qr', 'h2'};
for i = 1:size(legs, 1)
    a = legs{i, 1}; b = legs{i, 2};
    pa = W.(a); pb = W.(b);
    if a(1) == 'h'
        car = car_init(pa(1), pa(2), spec.zones.(sprintf('x%s', a(2))).nose_map);
    else
        car = car_init(pa(1), pa(2), [0 1]);
    end
    out = run_leg(map, car, a, b, P, struct('log', sim_log()));
    fprintf('   %-7s -> %-7s end=(%7.1f,%7.1f) want=(%7.1f,%7.1f) err=%7.1f  %s\n', ...
        a, b, out.end_mm(1), out.end_mm(2), pb(1), pb(2), out.err_mm, ...
        strjoin(out.actions, ' | '));
end

fprintf('\n%s\n', repmat('=', 1, 66));
if fails == 0
    fprintf('RESULT: PASS - nose binding and strafe direction match the field record\n');
else
    fprintf('RESULT: %d FAILURE(S)\n', fails);
end
fprintf('%s\n', repmat('=', 1, 66));
if fails > 0
    error('test_zone_scripts:failed', '%d check(s) failed', fails);
end
end

% ----------------------------------------------------------------------
function res = run_script_local(map, car, which, P)
%Same step tables as sim_run's private run_script, kept here so the test does
%not depend on sim_run's command parsing.
spec = map.spec;
LEAVE  = spec.zones.leave_mm;
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
        error('run_script_local:which', 'unknown script "%s"', which);
end

dt = P.cmd_period_ms / 1000;
log = sim_log();
samples = struct([]);
for i = 1:size(steps, 1)
    switch steps{i, 1}
        case 'FWD'
            [car, samples, log] = do_drive(car, steps{i, 2}, 1, dt, P, struct(), log, samples, i, false);
        case 'BACK'
            [car, samples, log] = do_drive(car, steps{i, 2}, -1, dt, P, struct(), log, samples, i, false);
        case 'STRAFE'
            s = 1; if steps{i, 3} == 'L', s = -1; end
            [car, samples, log] = do_strafe(car, steps{i, 2}, s, dt, P, struct(), log, samples, i);
        case 'TURN'
            [car, samples, log] = do_turn(car, deg2rad(steps{i, 2}), dt, P, log, samples, i);
    end
end

res = struct('car', car, 'samples', samples, 'log', log, ...
             'verdict', check_legality(map, car));
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
