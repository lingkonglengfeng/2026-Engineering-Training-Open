function [car, samples, log] = do_drive_arc(car, mm, dtheta, dt, P, err, log, samples, seg_no)
%DO_DRIVE_ARC Drive `mm` forward WHILE turning `dtheta` onto the next bearing.
%
%   [car, samples, log] = DO_DRIVE_ARC(car, mm, dtheta, dt, P, err, log, samples, k)
%
% The mecanum mix supports vx and omega at the same time, so a corner does not
% have to be "stop, turn in place, drive". This executor blends the two: the
% segment is driven at the planned speed, and the heading is rotated onto the
% new bearing as the segment progresses.
%
% WHERE THE ARC IS CENTRED - and why that is a real choice, not a detail:
%   * dtheta = 0            : plain straight drive, identical to DO_DRIVE.
%   * |dtheta| up to 90 deg : the turn is spread over the whole segment, so the
%                             car cuts the corner. The path stays INSIDE the
%                             corridor (it is convex on the inside of the turn),
%                             which is what makes this fit a 400 mm lane.
%   * a reversal (~180 deg): a single arc cannot do this within the lane; the
%                             executor falls back to a pivot turn, because turning
%                             180 deg while translating would sweep the body
%                             across the corridor. Reported in the log so the
%                             fallback is never invisible.
%
% Speed is taken from MOTION_PLAN for the segment length, so the arc covers the
% same distance as the straight drive would; only the heading changes.
%
% NOT MODELLED (see sim_arc_vs_pivot.m): lateral scrub. A combined vx+omega
% command is the mecanum's worst case for grip, so on real hardware this arc
% slips. Phase 1 assumes the wheels deliver it exactly, which makes any time
% saving computed here an UPPER BOUND.

if abs(dtheta) > deg2rad(150)
    % reversal: pivot, do not arc (would sweep the body out of the corridor)
    [car, samples, log] = do_turn(car, dtheta, dt, P, log, samples, seg_no);
    [car, samples, log] = do_drive(car, mm, 1, dt, P, err, log, samples, seg_no, true);
    return
end
if mm <= 0
    return
end

r = motion_plan(mm, P.speed_mm_s, P);
speed_body = r.covered_mm / max(r.seconds, eps);

log = sim_log_add(log, ...
    '[NAV] drive+turn %.0fmm turn %+.1f deg (arc, sim only) spd=%.3f t=%lums', ...
    mm, dtheta * 180 / pi, P.speed_scale, round(r.drive_ms));

n = max(1, ceil(r.seconds / dt));
dstep = dtheta / n;

for i = 1:n
    F = [cos(car.h), sin(car.h)];
    dx = speed_body * dt * F(1);
    dy = speed_body * dt * F(2);
    car.omega = 0;
    [car, s] = car_step(car, [dx dy], dt, P, err);
    % phase 1: apply the commanded heading change exactly (no arc error model)
    car.h = wrap_pi(car.h + dstep);
    s.h = car.h;
    samples = append_sample(samples, s);
end
end

function samples = append_sample(samples, s)
if isempty(samples)
    samples = s;
else
    samples(end + 1) = s;
end
end

function a = wrap_pi(a)
a = mod(a + pi, 2 * pi) - pi;
end
