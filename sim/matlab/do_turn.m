function [car, samples, log] = do_turn(car, diff_rad, dt, P, log, samples, seg_no)
%DO_TURN In-place rotation by diff_rad, driven through the wheel layer.
%
% Mirrors the firmware's ChassisMotor_MixTurn() + the turn PID outcome:
%   FL = -w  FR = +w  RL = -w  RR = +w      (pure yaw, no translation)
% at CHASSIS_TURN_FAST_WHEEL_RPM (120) for per-leg alignment, which the project's
% measurements put at roughly 0.8 s for a 90 deg turn (the precise manual
% `&L90`/`&R90` path uses 90 RPM and takes ~1.2 s).
%
% Phase 1 has no turn-error model, so this lands exactly on target. That is a
% known OPTIMISTIC ASSUMPTION, not a finding: a real turn leaves about +-0.5 deg
% of residual and a little overshoot. Phase 2 injects both through `err`.
%
% The yaw rate is produced the way the wheels produce it: by asking for a pure
% yaw twist, so the same wheel layer and the same saturation logic apply here as
% everywhere else, instead of a separately hand-computed duration.

if abs(diff_rad) < eps
    return
end

sc = sim_scale(P);
t_start = car.t;
target_deg = diff_rad * 180 / pi;
sgn = sign(diff_rad);

% full-scale yaw rate = one wheel pair at max_wheel_rpm across the lever arm
omega_full = sc.yaw;
% The turn must come out CCW for a POSITIVE diff_rad, which is the declared
% contract (`omega>0 = counter-clockwise`, Core/Src/chassis_motor.c:155). The
% wheel mix in WHEEL_INVERSE rotates the other way for a positive omega, so the
% yaw twist is negated here. Getting this wrong is not subtle once two moves are
% combined: a 180 deg turn then went the wrong way and the following strafe was
% applied along the wrong axis, putting the car at (3145,546) instead of the
% start zone at (2250,150) - i.e. 900 mm outside the mat.
omega_cmd = -sgn * (P.turn_fast_wheel_rpm / P.max_wheel_rpm) * omega_full;

log = sim_log_add(log, '[NAV] align turn %+.1f deg (target %+.0f, fast)', ...
    target_deg, target_deg);

w = wheel_inverse(0, 0, omega_cmd / sc.yaw, P);
log = sim_log_add(log, '[NAV] applied rpm FL%+d FR%+d RL%+d RR%+d', ...
    round(w.rpm(1)), round(w.rpm(2)), round(w.rpm(3)), round(w.rpm(4)));

duration = abs(diff_rad) / abs(omega_cmd);
n = max(1, ceil(duration / dt));
step = diff_rad / n;

for i = 1:n
    % car_step only integrates translation, and returns early for an empty
    % displacement, so the pure-yaw rotation and its time are advanced HERE.
    % (A previous version forgot car.t, which silently shortened every later
    % translation in the same script - &leave moved 250 mm instead of 300.)
    car.t = car.t + dt;
    car.h = wrap_pi(car.h + step);

    % route through the wheel layer so the turn is produced the same way as
    % everything else (and so phase-2 turn error has somewhere to attach)
    wv = wheel_inverse(0, 0, omega_cmd / sc.yaw, P);
    s = struct('t', car.t, 'x', car.x, 'y', car.y, 'h', car.h, ...
               'vx', 0, 'vy', 0, 'omega', omega_cmd, ...
               'rpm', wv.rpm, 'saturated', wv.saturated);
    samples = append_sample(samples, s);
end

elapsed_ms = (car.t - t_start) * 1000;
log = sim_log_add(log, ...
    '[TURN] done %lums target=%+.2f err=%+.3f overshoot=%.3f peak_o=%.2f', ...
    round(elapsed_ms), target_deg, 0.00, 0.000, 1.00);
end

function samples = append_sample(samples, s)
% Keep every sample the SAME struct shape, or `samples(end+1) = s` throws
% "subscripted assignment between dissimilar structures".
if isempty(samples)
    samples = s;
else
    samples(end + 1) = s;
end
end

function a = wrap_pi(a)
a = mod(a + pi, 2 * pi) - pi;
end
