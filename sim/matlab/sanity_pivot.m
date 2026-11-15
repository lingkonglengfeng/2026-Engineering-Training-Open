function sanity_pivot()
%SANITY_PIVOT Validate the model against a manoeuvre the firmware ACTUALLY DOES.
%
% Before trusting any "no path" result about turning, the model has to pass a
% basic sanity check: the firmware rotates in place at the cross junction
% (&goto aligns every segment that way) and it works on the real mat. If this
% model calls that illegal, the MODEL is wrong, not the firmware.
%
% This prints the clearance of the body through a full in-place rotation at a few
% points, so the model's bias (if any) is measured rather than assumed.

spec = load_mapspec();
map  = build_grid(spec);

fprintf('\n%s\n', repmat('=', 1, 78));
fprintf('MODEL SANITY: rotate in place, which the firmware does and which works\n');
fprintf('%s\n', repmat('=', 1, 78));
fprintf('\n  car %d mm square, half-diagonal %.1f mm\n\n', ...
    spec.car_mm, spec.car_mm * sqrt(2) / 2);
fprintf('%-24s %-12s %-12s %s\n', 'rotation centre (mm)', 'min clearance', 'verdict', 'binding rect');
fprintf('%s\n', repmat('-', 1, 78));

pts = { ...
    'cross centre (1200,1200)', [1200 1200]; ...
    'near corner  (1175,1225)', [1175 1225]; ...
    'lane cell    (1175,1200)', [1175 1200]; ...
    'start zone   (2250,2250)', [2250 2250]; ...
    'zone exit    (2125,1950)', [2125 1950]; ...
    'ring centre  (2125,1200)', [2125 1200]};

for i = 1:size(pts, 1)
    [clr, which] = pivot_clearance(map, pts{i, 2}(1), pts{i, 2}(2));
    if clr > 0.05
        v = 'legal';
    elseif clr > -1
        v = 'ON THE EDGE';
    else
        v = 'ILLEGAL';
    end
    fprintf('%-24s %-12.1f %-12s %s\n', pts{i, 1}, clr, v, which);
end

fprintf('\n%s\n', repmat('-', 1, 78));
fprintf('WHY THIS MATTERS\n');
fprintf('  Rotating in place is not a "path" - the centre does not move - so the\n');
fprintf('  body sweeps a disc of radius (half-diagonal) = %.1f mm around the centre.\n', ...
    spec.car_mm * sqrt(2) / 2);
fprintf('  At the cross junction the nearest forbidden edge is a yellow-zone corner\n');
fprintf('  200 mm away, so an ideal 300 mm square overhangs by %.1f mm during the\n', ...
    spec.car_mm * sqrt(2) / 2 - 200);
fprintf('  middle of the rotation. That is a MODEL statement about a mathematically\n');
fprintf('  exact 300x300 square; the real body is not exact.\n');
fprintf('\n  So if "cross centre" shows ILLEGAL here, the model is stricter than the\n');
fprintf('  physical mat and the earlier "no arc path" results inherit that strictness.\n');
fprintf('  The honest reading is then: turns have roughly ZERO margin, arcs would need\n');
fprintf('  to gain %.1f mm, and whether an arc fits is decided by real dimensions, not\n', ...
    spec.car_mm * sqrt(2) / 2 - 200);
fprintf('  by this geometry.\n');
fprintf('%s\n\n', repmat('=', 1, 78));
end

% ----------------------------------------------------------------------
function [minclr, which] = pivot_clearance(map, x, y)
minclr = inf;
which = '';
for hd = 0:2:360
    [legal, clr, name] = pose_clr(map, x, y, deg2rad(hd));
    if clr < minclr
        minclr = clr;
        which = name;
    end
    if ~legal && clr <= -1
        return
    end
end
end

function [legal, clr, name] = pose_clr(map, x, y, h)
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
name = '';

m = min([x0, y0, spec.field_mm - x1, spec.field_mm - y1]);
if m < clr
    clr = m; name = 'mat edge';
end
if m < -1
    legal = false;
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
    if sep < -1
        legal = false;
    end
end
end
