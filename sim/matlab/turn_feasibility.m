function turn_feasibility()
%TURN_FEASIBILITY Can the car turn 90 deg at a junction, on ANY smooth path?
%
% WHY THIS REPLACES THE EARLIER LANE-WIDTH ARGUMENT
% The first version of this analysis concluded "impossible" from the fact that
% the body spans 424 mm at 45 deg while the lane is 400 mm. That argument is WRONG
% because it assumes the car must keep its centre inside the 400 mm lane band.
% Nothing requires that. The real constraint is only:
%
%     the swept BODY must never overlap a yellow zone, a table, or leave the mat
%
% Near a junction the centre is free to move in the open pocket BETWEEN the two
% facing yellow zones, which is far wider than the lane. So the question has to be
% asked properly: is there ANY path from the entry pose to the exit pose, at every
% point of which the rotated body stays legal?
%
% METHOD: parameterise candidate paths as cubic Beziers. A cubic Bezier has four
% control points; the first and last are pinned by the entry/exit poses, and the
% two middle ones slide along the entry and exit headings (that is what fixes the
% tangent directions). So the whole search is over ONE scalar: how far the
% control points are placed. Large values give a broad sweep, small values a
% tight turn. Then sample the body along the path and test legality.
%
% This is a genuine generalisation of the arc search: every circular arc is not
% reachable by a Bezier with these tangents, but many non-circular turns are, and
% arcs are a poor fit for a corner anyway.

spec = load_mapspec();
map  = build_grid(spec);

fprintf('\n%s\n', repmat('=', 1, 80));
fprintf('CAN THE CAR TURN 90 DEG AT A JUNCTION?  (swept-body test, any path)\n');
fprintf('%s\n', repmat('=', 1, 80));

car = spec.car_mm;
fprintf('\n  car %d mm square   half-diagonal %.1f mm\n', car, car * sqrt(2) / 2);
fprintf('  constraint tested: swept body vs yellow zones / tables / mat edge ONLY.\n');
fprintf('  (the lane band is NOT a constraint - the centre may use the open pocket)\n');

% junctions to test: the cross centre, and the two work-approach points
cases = { ...
    'cross centre   +y -> +x', [1200 1200], [0 1],  [1 0]; ...
    'cross centre   +x -> +y', [1200 1200], [1 0],  [0 1]; ...
    'cross centre   +y -> -x', [1200 1200], [0 1],  [-1 0]; ...
    'cross centre   -y -> +x', [1200 1200], [0 -1], [1 0]; ...
    'raw approach   +y -> +x', [1200 2050], [0 1],  [1 0]; ...
    'rough approach +y -> +x', [1200  350], [0 1],  [1 0]};

fprintf('\n%-26s %-10s %-12s %s\n', 'junction / turn', 'result', 'best scale', 'clearance at best');
fprintf('%s\n', repmat('-', 1, 80));

for i = 1:size(cases, 1)
    [ok, best_s, best_min] = search_turn(map, cases{i, 2}, cases{i, 3}, cases{i, 4});
    if ok
        fprintf('%-26s %-10s %-12.2f %.1f mm\n', cases{i, 1}, 'POSSIBLE', best_s, best_min);
    else
        fprintf('%-26s %-10s %-12.2f %.1f mm\n', cases{i, 1}, 'no path', best_s, best_min);
    end
end

fprintf('\n%s\n', repmat('-', 1, 80));
fprintf('HOW TO READ IT\n');
fprintf('  "POSSIBLE" means some Bezier path exists along which the body never\n');
fprintf('  overlaps a forbidden rectangle. "no path" means even the best of the\n');
fprintf('  tried control-point scales failed, with the closest attempt shown.\n');
fprintf('\n  Note what is and is NOT being claimed: a "no path" here is a statement\n');
fprintf('  about SWEPT-BODY LEGALITY for smooth one-shot turns. It does not forbid\n');
fprintf('  rotate-then-drive, which is a different manoeuvre: rotating about a point\n');
fprintf('  with room around it, then translating on-axis.\n');
fprintf('%s\n\n', repmat('=', 1, 80));
end

% ----------------------------------------------------------------------
function [ok, best_s, best_min] = search_turn(map, corner, in_dir, out_dir)
%Entry pose: on the incoming centre line, a bit back from the corner.
%Exit pose : on the outgoing centre line, the same distance past it.
in_dir  = in_dir(:).'  / norm(in_dir);
out_dir = out_dir(:).' / norm(out_dir);
L = 200;                                  % how far back/forward the poses sit

P0 = corner - in_dir * L;
P3 = corner + out_dir * L;
h0 = atan2(in_dir(2), in_dir(1));
h3 = atan2(out_dir(2), out_dir(1));

ok = false;
best_s = NaN;
best_min = -inf;

for s = [0.2 0.3 0.4 0.5 0.6 0.8 1.0 1.3 1.6 2.0 2.5 3.0] * L
    P1 = P0 + in_dir * s;
    P2 = P3 - out_dir * s;
    [legal, minclr] = sweep_bezier(map, P0, P1, P2, P3, h0, h3);
    if minclr > best_min
        best_min = minclr;
        best_s = s / L;
    end
    if legal
        ok = true;
        best_s = s / L;
        best_min = minclr;
        return
    end
end
end

function [ok, minclr] = sweep_bezier(map, P0, P1, P2, P3, h0, h3)
%Sample a cubic Bezier; heading is interpolated with a smoothstep so it turns
%monotonically from h0 to h3 (the car IS rotating while it translates - that is
%the point the earlier lane argument ignored).
n = 200;
minclr = inf;
ok = true;
for i = 0:n
    t = i / n;
    B = (1 - t)^3 * P0 + 3 * (1 - t)^2 * t * P1 + 3 * (1 - t) * t^2 * P2 + t^3 * P3;
    % smoothstep for the heading, with wrap handled by the shortest signed diff
    d = mod(h3 - h0 + pi, 2 * pi) - pi;
    u = t * t * (3 - 2 * t);
    h = h0 + d * u;
    [legal, clr] = pose_clearance(map, B(1), B(2), h);
    if ~legal
        ok = false;
        minclr = min(minclr, clr);
        return
    end
    minclr = min(minclr, clr);
end
end

function [legal, clr] = pose_clearance(map, x, y, h)
%Signed clearance of the body rectangle: >0 legal, <=0 the depth of intrusion.
spec = map.spec;
half = spec.car_mm / 2;
F = [cos(h), sin(h)];
Rt = [-F(2), F(1)];
ex = half * (abs(F(1)) + abs(Rt(1)));
ey = half * (abs(F(2)) + abs(Rt(2)));
x0 = x - ex; x1 = x + ex;
y0 = y - ey; y1 = y + ey;

legal = true;
clr = inf;

% mat edge
m = min([x0, y0, spec.field_mm - x1, spec.field_mm - y1]);
if m < 0
    legal = false;
end
clr = min(clr, m);

rr = blocked_rects(spec);
for i = 1:size(rr, 1)
    r = rr(i, :);
    % separation along each axis: negative on both means overlap
    dx = max([r(1) - x1, x0 - r(2)]);
    dy = max([r(3) - y1, y0 - r(4)]);
    if dx > 0 || dy > 0
        sep = hypot(max(dx, 0), max(dy, 0));
    else
        sep = max(dx, dy);            % negative: intruding by this much
    end
    if sep <= 0
        legal = false;
    end
    clr = min(clr, sep);
end
end
