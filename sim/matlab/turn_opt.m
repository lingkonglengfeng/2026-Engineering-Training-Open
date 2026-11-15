function turn_opt()
%TURN_OPT Maximise swept-body clearance with independent control-point offsets.
%
% A single rigid offset of the whole path made things WORSE (-30.4 mm vs -17.1 mm
% for the un-offset path), which says the obstruction is local: the path has to
% bulge around one specific corner, not shift as a whole. So this version gives
% each interior control point its own offset along its own normal and hill-climbs
% to maximise the minimum clearance.
%
% This is the last and most general attempt at the question "is a 90 deg turn
% available at all". If a legal path exists, hill-climbing from several starts
% should find it, because the objective is smooth and the feasible set (if any) is
% broad. If nothing legal turns up, the conclusion is about the BODY, not the path.

spec = load_mapspec();
map  = build_grid(spec);

fprintf('\n%s\n', repmat('=', 1, 84));
fprintf('TURN: MAXIMISE CLEARANCE OVER INDEPENDENT CONTROL-POINT OFFSETS\n');
fprintf('%s\n', repmat('=', 1, 84));

cases = { ...
    'cross centre   +y -> +x', [1200 1200], [0 1],  [1 0]; ...
    'cross centre   +x -> +y', [1200 1200], [1 0],  [0 1]; ...
    'cross centre   +y -> -x', [1200 1200], [0 1],  [-1 0]; ...
    'cross centre   -y -> +x', [1200 1200], [0 -1], [1 0]};

fprintf('\n%-26s %-14s %-16s %s\n', 'junction / turn', 'best clearance', 'binding rect', 'legal?');
fprintf('%s\n', repmat('-', 1, 84));

for i = 1:size(cases, 1)
    r = optimise(map, cases{i, 2}, cases{i, 3}, cases{i, 4});
    fprintf('%-26s %-14s %-16s %s\n', cases{i, 1}, sprintf('%+.1f mm', r.clr), ...
        r.binds, tern(r.ok));
end

fprintf('\n%s\n', repmat('-', 1, 84));
fprintf('BOTTOM LINE\n');
fprintf('  Compare with the un-optimised tangent path, which was -17.1 mm. If\n');
fprintf('  optimising only recovers a few mm, the turn is decided by the body\n');
fprintf('  rotating inside the pocket and no path can fix it. If it recovers past\n');
fprintf('  zero, a legal turn exists and the earlier conclusion was wrong.\n');
fprintf('%s\n\n', repmat('=', 1, 84));
end

% ----------------------------------------------------------------------
function r = optimise(map, corner, in_dir, out_dir)
in_dir  = in_dir(:).'  / norm(in_dir);
out_dir = out_dir(:).' / norm(out_dir);
h0 = atan2(in_dir(2), in_dir(1));
h3 = atan2(out_dir(2), out_dir(1));

nin  = [-in_dir(2),  in_dir(1)];
nout = [-out_dir(2), out_dir(1)];
L = 220;

% base control points: two along each heading, plus one on each side of the middle
base = [ corner - in_dir * L; ...
         corner - in_dir * L * 0.45; ...
         corner - in_dir * L * 0.10; ...
         corner + out_dir * L * 0.10; ...
         corner + out_dir * L * 0.45; ...
         corner + out_dir * L ];

% each interior point gets its own offset along the average inward normal
best = struct('clr', -inf, 'binds', '-', 'off', zeros(1, 4));
for start = {[0 0 0 0], [100 100 100 100], [-100 -100 -100 -100], ...
             [200 0 0 200], [0 200 200 0]}
    off = start{1};
    step = 80;
    for iter = 1:60
        improved = false;
        for k = 1:4
            for d = [step -step]
                trial = off;
                trial(k) = trial(k) + d;
                [clr, binds] = evaluate(map, base, nin, nout, trial, h0, h3);
                if clr > best.clr
                    best.clr = clr; best.binds = binds; best.off = trial;
                end
                if clr > evaluate(map, base, nin, nout, off, h0, h3)
                    off = trial;
                    improved = true;
                end
            end
        end
        if ~improved
            step = step / 2;
            if step < 2
                break
            end
        end
    end
end
r = best;
r.ok = r.clr > 0;
end

function [clr, binds] = evaluate(map, base, nin, nout, off, h0, h3)
Pc = base;
% interior points 2..5 get their offsets; ends stay pinned to the poses
for k = 1:4
    idx = k + 1;
    if k <= 2
        n = nin;
    else
        n = nout;
    end
    Pc(idx, :) = Pc(idx, :) + n * off(k);
end
[clr, binds] = sweep_quintic(map, Pc, h0, h3);
end

function [minclr, binds] = sweep_quintic(map, Pc, h0, h3)
n = 220;
minclr = inf; binds = '-';
d = mod(h3 - h0 + pi, 2 * pi) - pi;
for i = 0:n
    t = i / n;
    b = [ (1-t)^5, 5*(1-t)^4*t, 10*(1-t)^3*t^2, 10*(1-t)^2*t^3, 5*(1-t)*t^4, t^5 ];
    px = b * Pc(:, 1);
    py = b * Pc(:, 2);
    u = t * t * (3 - 2 * t);
    h = h0 + d * u;
    [clr, name] = pose_clr(map, px, py, h);
    if clr < minclr
        minclr = clr; binds = name;
    end
end
end

function [clr, name] = pose_clr(map, x, y, h)
spec = map.spec;
half = spec.car_mm / 2;
F = [cos(h), sin(h)];
Rt = [-F(2), F(1)];
ex = half * (abs(F(1)) + abs(Rt(1)));
ey = half * (abs(F(2)) + abs(Rt(2)));
x0 = x - ex; x1 = x + ex;
y0 = y - ey; y1 = y + ey;

clr = inf; name = '-';
m = min([x0, y0, spec.field_mm - x1, spec.field_mm - y1]);
if m < clr
    clr = m; name = 'mat edge';
end
rr = blocked_rects(spec);
ids = {'yellow-NW', 'yellow-NE', 'yellow-SW', 'yellow-SE', ...
       'pad-raw', 'pad-rough', 'pad-temp', 'pad-qr'};
for i = 1:size(rr, 1)
    q = rr(i, :);
    dx = max([q(1) - x1, x0 - q(2)]);
    dy = max([q(3) - y1, y0 - q(4)]);
    if dx > 0 || dy > 0
        sep = hypot(max(dx, 0), max(dy, 0));
    else
        sep = max(dx, dy);
    end
    if sep < clr
        clr = sep; name = ids{i};
    end
end
end

function s = tern(v)
if v, s = 'YES'; else, s = 'no'; end
end
