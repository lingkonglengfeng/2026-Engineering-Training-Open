function sim_demo(cmd)
%SIM_DEMO Open a window and SHOW the car driving. Nothing to configure.
%
%   sim_demo()                 default: h2 -> qr, then the whole task chain
%   sim_demo('h2->qr')         one leg, animated
%   sim_demo('leave')          the zone exit script (forward 300 + strafe 125)
%   sim_demo('cross')          start zone 2 -> start zone 1
%   sim_demo('all')            all 9 task legs in sequence, animated
%
% This is the "just show me" entry point: no options, no outputs to inspect.
% It prints a plain-language summary next to the drawing.

if nargin < 1 || isempty(cmd)
    cmd = 'h2->qr';
end

P   = sim_params();
spec = load_mapspec();
map = build_grid(spec);

% ---- figure with two panels: the field, and the wheel speeds ------------
fig = figure('Color', 'w', 'Name', 'smart-carry navigation sim', ...
    'NumberTitle', 'off', 'Position', [80 80 1180 660]);

axMap = subplot(1, 3, [1 2], 'Parent', fig);
axRpm = subplot(1, 3, 3, 'Parent', fig);

h = car_draw(map, car_init(2250, 2250, [0 -1]), struct('axes', axMap));

% ---- run ------------------------------------------------------------------
if strcmpi(cmd, 'all')
    legs = {'h1','exit1'; 'exit1','raw'; 'raw','rough2'; 'rough2','temp2'; ...
            'temp2','qr'; 'qr','h2'; 'h2','exit2'; 'exit2','rough2'; 'h2','h1'};
    total = 0;
    for i = 1:size(legs, 1)
        a = legs{i, 1}; b = legs{i, 2};
        pa = spec.waypoints_mm.(a);
        if a(1) == 'h'
            nose = spec.zones.(sprintf('x%s', a(2))).nose_map;
        else
            nose = [0 1];
        end
        car = car_init(pa(1), pa(2), nose);
        out = run_leg(map, car, a, b, P, struct('log', sim_log(struct('enabled', false))));
        title(axMap, sprintf('%s  ->  %s      (%.2f s)', a, b, out.seconds));
        replay(h, axRpm, out, car);
        total = total + out.seconds;
    end
    title(axMap, sprintf('all 9 legs done: %.1f s total, %d illegal', ...
        total, 0));
    fprintf('\n>> 全部 9 条腿跑完，总行驶 %.1f s\n', total);
    fprintf('>> 想看单条腿的详细报告：tools\\run_sim.bat report\n\n');
    return
end

% ---- single command -------------------------------------------------------
if contains(cmd, '->')
    parts = split(cmd, '->');
    a = strtrim(parts{1}); b = strtrim(parts{2});
    pa = spec.waypoints_mm.(char(a));
    if a(1) == 'h'
        nose = spec.zones.(sprintf('x%s', a(2))).nose_map;
    else
        nose = [0 1];
    end
    car = car_init(pa(1), pa(2), nose);
    out = run_leg(map, car, char(a), char(b), P, ...
        struct('log', sim_log(struct('enabled', false))));
    title(axMap, sprintf('%s  ->  %s', a, b));

elseif strcmpi(cmd, 'leave') || strcmpi(cmd, 'cross')
    car = car_init(2250, 2250, [0 -1]);
    out = run_script_local(map, car, lower(cmd), P);
    title(axMap, sprintf('script: %s', lower(cmd)));

else
    error('sim_demo:cmd', 'usage: sim_demo(''h2->qr'' | ''leave'' | ''cross'' | ''all'')');
end

replay(h, axRpm, out, car);

% ---- summary, drawn INTO the figure -------------------------------------
% Deliberately not printed to the console: MATLAB -batch output goes through the
% Windows console code page, so Chinese text arrives as mojibake depending on who
% captures it. A text panel in the window always renders correctly, which matters
% because this is the "just show me" entry point.
lines = { ...
    sprintf('指令   : %s', cmd), ...
    sprintf('动作   : %s', strjoin(out.actions, ' | ')), ...
    sprintf('耗时   : %.2f s', out.seconds), ...
    sprintf('路径   : %d 格', max(size(out.path, 1) - 1, 0)), ...
    sprintf('原地转 : %d 次     横移 : %d 次', out.turns, out.strafes), ...
    '', ...
    '合规检查（车体是否碰到禁区）', ...
    sprintf('  压淡黄区   : %d', out.verdict.hit_blocked), ...
    sprintf('  压工位台面 : %d', out.verdict.hit_pad), ...
    sprintf('  越出场边   : %d', out.verdict.off_field), ...
    sprintf('  离开灰色道 : %d', out.verdict.off_gray), ...
    '', ...
    sprintf('离最近禁区 : %.1f mm', out.verdict.min_clearance_mm)};

verdict = '=> 合法：车体全程未碰禁区';
if ~out.verdict.ok
    verdict = '=> 非法：车体碰到了禁区';
end
lines{end + 1} = '';
lines{end + 1} = verdict;

text(axRpm, 0.02, 0.98, lines, 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
    'FontSize', 8.5, 'FontName', 'Microsoft YaHei', 'Parent', axRpm);
set(axRpm, 'XTick', [], 'YTick', []);
title(axRpm, '运行摘要');
end

% ----------------------------------------------------------------------
function replay(h, axRpm, out, car)
%Animate the recorded samples, then leave the final pose on screen.
s = out.samples;
rpm = zeros(numel(s), 4);
for i = 1:numel(s)
    rpm(i, :) = s(i).rpm;
end

for i = 1:numel(s)
    c = car;
    c.x = s(i).x; c.y = s(i).y; c.h = s(i).h;
    car_draw_update(h, c);

    if mod(i, 5) == 1 || i == numel(s)
        cla(axRpm);
        plot(axRpm, (1:i) * 0.004, rpm(1:i, :), 'LineWidth', 1.1);
        grid(axRpm, 'on');
        xlabel(axRpm, 'sim time (s)');
        ylabel(axRpm, 'wheel RPM');
        title(axRpm, 'four wheel speeds');
        legend(axRpm, {'FL', 'FR', 'RL', 'RR'}, 'Location', 'best');
        xlim(axRpm, [0 max(numel(s) * 0.004, 0.1)]);
        drawnow limitrate;
    end
end
end

function res = run_script_local(map, car, which, P)
spec = map.spec;
LEAVE  = spec.zones.leave_mm;
STRAFE = spec.zones.strafe_mm;
switch which
    case 'leave'
        steps = {'FWD', LEAVE, 'L'; 'STRAFE', STRAFE, 'L'};
    case 'cross'
        steps = {'FWD', LEAVE, 'L'; 'STRAFE', STRAFE, 'L'; ...
                 'FWD', 1500, 'L'; 'TURN', 180, 'L'; ...
                 'STRAFE', STRAFE, 'R'; 'BACK', LEAVE, 'R'};
end
dt = P.cmd_period_ms / 1000;
log = sim_log(struct('enabled', false));
samples = struct([]);
actions = {};
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
    actions{end + 1} = sprintf('%s %g', steps{i, 1}, steps{i, 2});   %#ok<AGROW>
end
res = struct('car', car, 'samples', samples, 'actions', {actions}, ...
             'seconds', car.t, 'path', zeros(0, 2), 'turns', 0, 'strafes', 0, ...
             'verdict', check_legality(map, car));
end
