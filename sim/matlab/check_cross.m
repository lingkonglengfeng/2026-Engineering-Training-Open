function check_cross()
%CHECK_CROSS Test the recorded start-zone geometry for self-consistency.
%
% Two recorded facts involve start zone 1 (h1 = (2250,150), exit1 = (2125,450)):
%   A) &leave  from the zone centre must end at exit1
%   B) &cross  from zone 2 must end at the zone centre
% For a nose of +y, (A) needs a strafe toward map -x while (B) needs one toward
% map +x. Since a strafe's map direction flips with the nose, at most one of them
% can hold for any single nose - unless the zone's recorded nose is wrong.
% This script measures both, for both candidate noses, instead of arguing.

spec = load_mapspec();
map  = build_grid(spec);
P    = sim_params();

fid = fopen('tmp_check_cross.txt', 'w', 'n', 'UTF-8');
noses = {[0 1], 'nose +y (map up, into the field)'; [0 -1], 'nose -y (map down)'};

for n = 1:size(noses, 1)
    nose = noses{n, 1};
    fprintf(fid, '\n=========== zone 1 with %s ===========\n', noses{n, 2});

    % (A) &leave from the zone centre
    car = car_init(2250, 150, nose);
    res = script(map, car, 'leave', P);
    fprintf(fid, 'A) &leave   : (2250,150) -> (%.1f,%.1f)   recorded exit1 = (2125,450)  %s\n', ...
        res.car.x, res.car.y, okstr(abs(res.car.x-2125) < 2 && abs(res.car.y-450) < 2));

    % (B) &cross from zone 2 into zone 1, with a per-step trace
    car = car_init(2250, 2250, [0 -1]);
    res = script(map, car, 'cross', P);
    fprintf(fid, 'B) &cross   : (2250,2250) -> (%.1f,%.1f)   recorded h1 centre = (2250,150) %s\n', ...
        res.car.x, res.car.y, okstr(abs(res.car.x-2250) < 2 && abs(res.car.y-150) < 2));
    for k = 1:numel(res.trace)
        fprintf(fid, '      %s\n', res.trace{k});
    end
end
fclose(fid);
end

function res = script(map, car, which, P)
spec = map.spec;
LEAVE  = spec.zones.leave_mm;
STRAFE = spec.zones.strafe_mm;
switch which
    case 'leave'
        steps = {'FWD', LEAVE, 'S'; 'STRAFE', STRAFE, 'L'};
    case 'cross'
        steps = {'FWD', LEAVE, 'S'; 'STRAFE', STRAFE, 'L'; ...
                 'FWD', 1500, 'S'; 'TURN', 180, 'S'; ...
                 'STRAFE', STRAFE, 'R'; 'BACK', LEAVE, 'S'};
end
dt = P.cmd_period_ms / 1000;
log = sim_log();
samples = struct([]);
trace = {};
for i = 1:size(steps, 1)
    x0 = car.x; y0 = car.y; h0 = car.h;
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
    trace{end + 1} = sprintf('%-7s %-5s from h=%+7.2f deg at (%7.1f,%7.1f) -> (%7.1f,%7.1f)  d=(%+7.1f,%+7.1f)', ...
        steps{i, 1}, steps{i, 3}, h0 * 180 / pi, x0, y0, car.x, car.y, ...
        car.x - x0, car.y - y0);
end
res = struct('car', car, 'trace', {trace});
end

function s = okstr(ok)
if ok, s = '<-- MATCHES'; else, s = '<-- does NOT match'; end
end
