function [car, samples, log] = do_drive(car, mm, sgn, dt, P, err, log, samples, seg_no, log_plan)
%DO_DRIVE Straight-line move of `mm` along the nose (sgn = +1 forward, -1 back).
%
% Mirrors ZDT_Nav_BeginStraightDrive() (Core/Src/zdt_motor.c ~2014) plus the
% firmware's stop criterion: the motors run for the PLANNED TIME, and only then
% does the encoder check whether the distance actually came out right.
%
% That distinction matters. The firmware does the same trapezoidal plan, but it
% is the DURATION that is commanded, so any inconsistency between `t1000` and
% the speed model turns into under-/over-run. MOTION_PLAN() reports the
% resulting `overshoot_mm`, and the simulation drives exactly that time, so the
% effect is reproduced rather than assumed away.

if nargin < 10 || isempty(log_plan)
    log_plan = true;
end
if mm <= 0
    return
end

r = motion_plan(mm, P.speed_mm_s, P);

% actual body speed reached inside the planned duration (mm/s)
speed_body = r.covered_mm / max(r.seconds, eps);

if log_plan
    st = 'FWD'; if sgn < 0, st = 'BACK'; end
    log = sim_log_add(log, ...
        '[NAV] drive %.3frev (%.0fmm) %s spd=%.3f v=%.0fmm/s t=%lums', ...
        mm / P.wheel_circumference_mm, mm, st, P.speed_scale, ...
        P.speed_mm_s, round(r.drive_ms));
    log = sim_log_add(log, '[NAV] drive (sim) t1000=%lums overshoot=%+.1fmm', ...
        round(P.time_1000mm_ms), r.overshoot_mm);
end

n = max(1, ceil(r.seconds / dt));
for i = 1:n
    F = [cos(car.h), sin(car.h)];
    dx = sgn * speed_body * dt * F(1);
    dy = sgn * speed_body * dt * F(2);
    [car, s] = car_step(car, [dx dy], dt, P, err);
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
