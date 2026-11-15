function v = check_legality(map, car)
%CHECK_LEGALITY The safety verdicts a report must state explicitly.
%
%   v = CHECK_LEGALITY(map, car)
%
%   v.hit_blocked   body overlaps a 450x450 yellow zone        -> rule violation
%   v.hit_pad       body overlaps a work table / QR board      -> no-contact
%   v.off_field     body leaves the 2400x2400 mat              -> loss
%   v.off_gray      body centre is outside the gray driving lane
%   v.zone_locked   body overlaps a NO-TOUCH start zone
%   v.min_clearance_mm  smallest distance from the body to any forbidden rect
%   v.ok            true when nothing above is violated
%
% WHY off_gray EXISTS: the rules (docs/electrical/rules.pdf p3) say
% "搬运机器人全部离开启停区后只能在灰色车道上行驶，进入其它颜色区域（除启停区�?% 结束比赛" - leaving the gray lane ENDS THE RUN, it is not a soft penalty. That
% is stricter than "don't touch the yellow zones", and the firmware's map does
% treat the WORK lane strips as drivable (which it must, or the top ring band
% would be cut off). So this check exists to QUANTIFY the exposure rather than
% leave it implicit. It is reported separately from the forbidden-rect checks
% because it is a rule-interpretation risk, not a settled geometry violation.

spec = map.spec;
half = car.footprint_mm / 2;

% body corners in the map frame
F = [cos(car.h), sin(car.h)];
R = [-F(2), F(1)];             % firmware literal form; see car_step.m for why R = (-F.y, F.x)
body_x0 = car.x - half * (abs(F(1)) + abs(R(1)));
body_x1 = car.x + half * (abs(F(1)) + abs(R(1)));
body_y0 = car.y - half * (abs(F(2)) + abs(R(2)));
body_y1 = car.y + half * (abs(F(2)) + abs(R(2)));

v = struct();

% --- forbidden rectangles: four yellow zones + the pads -----------------
fk = spec.field_kind;
yellow = [ fk.block_lo_mm, fk.block_hi_mm, fk.block_lo_mm, fk.block_hi_mm;
           fk.block_hi_mm, spec.field_mm - fk.block_lo_mm, fk.block_lo_mm, fk.block_hi_mm;
           fk.block_lo_mm, fk.block_hi_mm, fk.block_hi_mm, spec.field_mm - fk.block_lo_mm;
           fk.block_hi_mm, spec.field_mm - fk.block_lo_mm, fk.block_hi_mm, spec.field_mm - fk.block_lo_mm ];

forbidden = [yellow; map.pads];
v.hit_blocked = overlaps_any(body_x0, body_y0, body_x1, body_y1, yellow);
v.hit_pad     = overlaps_any(body_x0, body_y0, body_x1, body_y1, map.pads);

[v.min_clearance_mm, ~] = min_rect_clearance(body_x0, body_y0, body_x1, body_y1, forbidden);

% --- field bounds -------------------------------------------------------
v.off_field = body_x0 < 0 || body_y0 < 0 || ...
              body_x1 > spec.field_mm || body_y1 > spec.field_mm;

% --- gray lane membership of the body CENTRE ---------------------------
v.off_gray = ~is_gray(spec, car.x, car.y);

% --- locked start zone -------------------------------------------------
v.zone_locked = false;
if map.lock_zone == 1 || map.lock_zone == 2
    if map.lock_zone == 1
        z = [2100, spec.field_mm, 0, 300];
    else
        z = [2100, spec.field_mm, spec.field_mm - 300, spec.field_mm];
    end
    v.zone_locked = overlaps_any(body_x0, body_y0, body_x1, body_y1, z);
end

v.body_rect = [body_x0 body_y0 body_x1 body_y1];
v.ok = ~(v.hit_blocked || v.hit_pad || v.off_field || v.zone_locked);
end

% ----------------------------------------------------------------------
function tf = is_gray(spec, x, y)
%Gray driving lane = the cross corridor (400 wide on the two axes) UNION the
%outer ring band. Mirrors the firmware's cross|ring test.
fk = spec.field_kind;
cross = (x >= fk.mid_mm - fk.cross_half_mm && x < fk.mid_mm + fk.cross_half_mm) || ...
        (y >= fk.mid_mm - fk.cross_half_mm && y < fk.mid_mm + fk.cross_half_mm);
ring  = x < fk.block_lo_mm || x >= fk.block_hi_mm || ...
        y < fk.block_lo_mm || y >= fk.block_hi_mm;
tf = cross || ring;
end

function tf = overlaps_any(x0, y0, x1, y1, rects)
if isempty(rects)
    tf = false;
    return
end
tf = false;
for i = 1:size(rects, 1)
    r = rects(i, :);
    if x0 < r(2) && x1 > r(1) && y0 < r(4) && y1 > r(3)
        tf = true;
        return
    end
end
end

function [dmin, which] = min_rect_clearance(x0, y0, x1, y1, rects)
%Smallest separation between the body rect and any forbidden rect (0 when
%overlapping - reported as a negative overlap depth so a report can show HOW FAR
%inside it went).
dmin = inf;
which = 0;
for i = 1:size(rects, 1)
    r = rects(i, :);
    dx = max([r(1) - x1, x0 - r(2), 0]);
    dy = max([r(3) - y1, y0 - r(4), 0]);
    d = hypot(dx, dy);
    if d < dmin
        dmin = d;
        which = i;
    end
end
if isinf(dmin)
    dmin = NaN;
end
end
