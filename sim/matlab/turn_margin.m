function turn_margin()
%TURN_MARGIN How much is a turning path short, and by WHICH rectangle?
%
% The lane-width argument (424 mm body vs 400 mm lane) concluded "impossible by
% 24 mm". The swept-body test says "no path", but its numbers came out around
% 0 mm, and a sanity check showed the model itself is a few mm stricter than the
% real mat. So the useful output is not yes/no - it is:
%
%   HOW MANY MILLIMETRES SHORT IS THE BEST PATH, and WHICH RECTANGLE BINDS?
%
% A few millimetres short is a completely different answer from 24 mm short: the
% first is decided by real body/zone dimensions and is worth measuring, the second
% is structural and is not.

spec = load_mapspec();
map  = build_grid(spec);

fprintf('\n%s\n', repmat('=', 1, 82));
fprintf('HOW MUCH IS A 90 DEG TURN SHORT, AND WHAT BLOCKS IT?\n');
fprintf('%s\n', repmat('=', 1, 82));
fprintf('\n  car %d mm square   half-diagonal %.1f mm\n', ...
    spec.car_mm, spec.car_mm * sqrt(2) / 2);

cases = { ...
    'cross centre   +y -> +x', [1200 1200], [0 1],  [1 0]; ...
    'cross centre   +x -> +y', [1200 1200], [1 0],  [0 1]; ...
    'cross centre   +y -> -x', [1200 1200], [0 1],  [-1 0]; ...
    'cross centre   -y -> +x', [1200 1200], [0 -1], [1 0]; ...
    'raw approach   +y -> +x', [1200 2050], [0 1],  [1 0]; ...
    'rough approach +y -> +x', [1200  350], [0 1],  [1 0]};

fprintf('\n%-26s %-12s %-16s %-10s %s\n', ...
    'junction / turn', 'best short', 'binding rectangle', 'at t', 'legal?');
fprintf('%s\n', repmat('-', 1, 82));

for i = 1:size(cases, 1)
    r = best_path(map, cases{i, 2}, cases{i, 3}, cases{i, 4});
    fprintf('%-26s %-12s %-16s %-10.2f %s\n', cases{i, 1}, ...
        sprintf('%+.1f mm', r.short), r.binds, r.t_at, tern(r.ok));
end

fprintf('\n%s\n', repmat('-', 1, 82));
fprintf('READING THIS\n');
fprintf('  "best short" = the least-bad intrusion over all tried paths. Positive\n');
fprintf('  means a legal path exists. Values within a few mm of zero mean the corner\n');
fprintf('  is decided by REAL dimensions (body size, where the yellow paint actually\n');
fprintf('  is), not by a structural geometry limit - and this model is already known\n');
fprintf('  to be a few mm stricter than the mat.\n');
fprintf('\n  "binding rectangle" names what is in the way. If it is a yellow zone at a\n');
fprintf('  corner, that is the junction pinch; if it is a pad, the stop point is the\n');
fprintf('  problem instead.\n');
fprintf('%s\n\n', repmat('=', 1, 82));
end

% ----------------------------------------------------------------------
function r = best_path(map, corner, in_dir, out_dir)
in_dir  = in_dir(:).'  / norm(in_dir);
out_dir = out_dir(:).' / norm(out_dir);
L = 200;

P0 = corner - in_dir * L;
P3 = corner + out_dir * L;
h0 = atan2(in_dir(2), in_dir(1));
h3 = atan2(out_dir(2), out_dir(1));

r = struct('short', -inf, 'binds', '-', 't_at', NaN, 'ok', false);
for s = [0.15 0.2 0.25 0.3 0.4 0.5 0.6 0.7 0.85 1.0 1.2 1.4 1.7 2.0 2.5 3.0] * L
    P1 = P0 + in_dir * s;
    P2 = P3 - out_dir * s;
    [clr, binds, t_at] = sweep_bezier(map, P0, P1, P2, P3, h0, h3);
    if clr > r.short
        r.short = clr;
        r.binds = binds;
        r.t_at = t_at;
        r.ok = clr > 0;
    end
    if r.ok
        return
    end
end
end

function [minclr, binds, t_at] = sweep_bezier(map, P0, P1, P2, P3, h0, h3)
n = 240;
minclr = inf;
binds = '-';
t_at = NaN;
d = mod(h3 - h0 + pi, 2 * pi) - pi;
for i = 0:n
    t = i / n;
    B = (1 - t)^3 * P0 + 3 * (1 - t)^2 * t * P1 + 3 * (1 - t) * t^2 * P2 + t^3 * P3;
    u = t * t * (3 - 2 * t);
    h = h0 + d * u;
    [clr, name] = pose_clr(map, B(1), B(2), h);
    if clr < minclr
        minclr = clr;
        binds = name;
        t_at = t;
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
    r = rr(i, :);
    dx = max([r(1) - x1, x0 - r(2)]);
    dy = max([r(3) - y1, y0 - r(4)]);
    if dx > 0 || dy > 0
        sep = hypot(max(dx, 0), max(dy, 0));
    else
        sep = max(dx, dy);
    end
    if sep < clr
        clr = sep;
        name = ids{i};
    end
end
end

function s = tern(v)
if v, s = 'YES'; else, s = 'no'; end
end
