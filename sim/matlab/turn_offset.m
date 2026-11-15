function turn_offset()
%TURN_OFFSET Can the turn be made legal by OFFSETTING the path into the pocket?
%
% Previous results:
%   lane-width argument : "impossible by 24 mm"  <- WRONG, it assumed the centre
%                         must stay inside the 400 mm lane band.
%   tangent-Bezier search: -17.1 mm, binding on a yellow zone at mid-turn.
%                         This pinned both control points ON the entry and exit
%                         headings, which forces the centre through the junction
%                         point - i.e. it never tried to steer AROUND the corner.
%
% This version lets the control points leave those tangent lines, so the centre is
% free to swing into the open pocket BETWEEN the facing yellow zones. That is the
% manoeuvre the lane argument wrongly ruled out, and it is the one a driver would
% actually take: swing wide, turn, straighten.
%
% Path: a quintic Bezier (6 control points). The first two are spaced along the
% entry heading, the last two along the exit heading reversed, and ALL of them are
% displaced inward by one signed scalar `rho` along the inward normal. So the whole
% family is one parameter: rho = 0 is the straight-through turn, larger rho swings
% the centre further from the corner.
%
% Heading follows a smoothstep, so the car rotates while it translates.

spec = load_mapspec();
map  = build_grid(spec);

fprintf('\n%s\n', repmat('=', 1, 84));
fprintf('TURN WITH THE PATH OFFSET INTO THE POCKET (one parameter)\n');
fprintf('%s\n', repmat('=', 1, 84));

cases = { ...
    'cross centre   +y -> +x', [1200 1200], [0 1],  [1 0]; ...
    'cross centre   +x -> +y', [1200 1200], [1 0],  [0 1]; ...
    'cross centre   +y -> -x', [1200 1200], [0 1],  [-1 0]; ...
    'cross centre   -y -> +x', [1200 1200], [0 -1], [1 0]};

fprintf('\n%-26s %-14s %-12s %-16s %s\n', ...
    'junction / turn', 'best rho(mm)', 'clearance', 'binding rect', 'legal?');
fprintf('%s\n', repmat('-', 1, 84));

for i = 1:size(cases, 1)
    r = best_offset(map, cases{i, 2}, cases{i, 3}, cases{i, 4});
    fprintf('%-26s %-14s %-12s %-16s %s\n', cases{i, 1}, ...
        sprintf('%+.0f', r.rho), sprintf('%+.1f mm', r.clr), r.binds, tern(r.ok));
end

fprintf('\n%s\n', repmat('-', 1, 84));
fprintf('WHAT THIS IS TESTING\n');
fprintf('  rho > 0 swings the centre toward the INSIDE of the turn, i.e. away from\n');
fprintf('  the corner the body was clipping. If some rho makes the sweep legal, then\n');
fprintf('  a 90 deg turn IS available and the earlier "impossible" was an artifact of\n');
fprintf('  constraining the path, not a property of the car.\n');
fprintf('\n  If every rho still fails, the obstruction is the body rotating at 45 deg\n');
fprintf('  within %.1f mm of a zone corner - and the cure is a bigger pocket or a\n', ...
    200);
fprintf('  smaller body, not a cleverer path.\n');
fprintf('%s\n\n', repmat('=', 1, 84));
end

% ----------------------------------------------------------------------
function r = best_offset(map, corner, in_dir, out_dir)
in_dir  = in_dir(:).'  / norm(in_dir);
out_dir = out_dir(:).' / norm(out_dir);

h0 = atan2(in_dir(2), in_dir(1));
h3 = atan2(out_dir(2), out_dir(1));
cr = in_dir(1) * out_dir(2) - in_dir(2) * out_dir(1);
sgn = 1; if cr < 0, sgn = -1; end

% inward normal at the entry (points to the inside of the turn)
nin = sgn * [-in_dir(2), in_dir(1)];

L = 220;                       % pose spacing from the junction
r = struct('rho', 0, 'clr', -inf, 'binds', '-', 'ok', false);
for rho = [0 20 40 60 80 100 120 150 180 220 260 300 -20 -40 -60 -80 -100]
    % six control points: two on each side, all shifted inward by rho
    Pc = [ corner - in_dir * L; ...
           corner - in_dir * L * 0.45; ...
           corner - in_dir * L * 0.10; ...
           corner + out_dir * L * 0.10; ...
           corner + out_dir * L * 0.45; ...
           corner + out_dir * L ];              % 6x2
    Pc = Pc + repmat(nin * rho, 6, 1);

    [clr, binds] = sweep_quintic(map, Pc, h0, h3);
    if clr > r.clr
        r.clr = clr; r.rho = rho; r.binds = binds; r.ok = clr > 0;
    end
    if r.ok
        return
    end
end
end

function [minclr, binds] = sweep_quintic(map, Pc, h0, h3)
n = 260;
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
