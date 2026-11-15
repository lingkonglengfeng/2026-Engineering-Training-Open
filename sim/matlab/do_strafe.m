function [car, samples, log] = do_strafe(car, mm, sign_y, dt, P, err, log, samples, seg_no)
%DO_STRAFE Pure lateral move of `mm`; the nose does NOT move.
%
% Mirrors ZDT_Nav_BeginLateralDrive() (Core/Src/zdt_motor.c ~2085). The log line
% is the firmware's, including the "(no turn)" marker that makes this
% distinguishable from a turn in a capture.
%
% Why the firmware does this at all: for a jog of <= 3 cells (150 mm), turning
% twice costs about 2 s and stacks two +-0.5 deg alignment residuals onto the
% pose. Strafing leaves the heading untouched and the residual at zero, so it is
% strictly better whenever the geometry allows it. This is the one place where
% the mecanum base pays for itself, and the simulator makes its benefit
% measurable rather than assumed.

if mm <= 0
    return
end

r = motion_plan(mm, P.speed_mm_s, P);

speed_body = r.covered_mm / max(r.seconds, eps);

log = sim_log_add(log, '[NAV] strafe %.0fmm %s (no turn) spd=%.3f t=%lums', ...
    mm, strafe_name(sign_y), P.speed_scale, round(r.drive_ms));

n = max(1, ceil(r.seconds / dt));
for i = 1:n
    F = [cos(car.h), sin(car.h)];
    R = [-F(2), F(1)];        % firmware literal form; see car_step.m
    dx = sign_y * speed_body * dt * R(1);
    dy = sign_y * speed_body * dt * R(2);
    [car, s] = car_step(car, [dx dy], dt, P, err);
    samples = append_sample(samples, s);
end
end

function name = strafe_name(sign_y)
%Body-frame right component sign -> the word the firmware logs.
if sign_y > 0
    name = 'RIGHT';
else
    name = 'LEFT';
end
end

function samples = append_sample(samples, s)
if isempty(samples)
    samples = s;
else
    samples(end + 1) = s;
end
end
