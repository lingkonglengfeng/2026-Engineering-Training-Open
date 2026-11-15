function w = wheel_inverse(vx, vy, omega, P, opts)
%WHEEL_INVERSE Body twist -> 4 wheel speeds. This is the ONLY inverse solution.
%
%   w = WHEEL_INVERSE(vx, vy, omega, P)
%   w = WHEEL_INVERSE(vx, vy, omega, P, struct('scale', 0.5))
%
%   w.values   1x4 normalized wheel values BEFORE the wheel-direction signs
%   w.scale    speed scale actually used
%   w.travel_mm_per_s  1x4 wheel surface speeds (mm/s)
%   w.rpm      1x4 wheel RPM as sent to the drivers (signs applied)
%   w.saturated true if the per-wheel normalization had to reduce the command
%
% CONVENTION (Core/Src/chassis_motor.c:155 - the firmware's own statement):
%   vx > 0     forward
%   vy > 0     to the car's RIGHT
%   omega > 0  counter-clockwise seen from above (a LEFT turn)
%
% The mix below is transcribed verbatim from ChassisMotor_Mix()
% (Core/Src/chassis_motor.c:157-160):
%
%   FL = vx + vy - omega        FR = vx - vy + omega
%   RL = vx - vy - omega        RR = vx + vy + omega
%
% The +/-150 mm mounting offsets are already divided out, so the inputs are
% dimensionless: 1.0 means "one wheel at CHASSIS_MAX_WHEEL_RPM". Callers with
% physical units must convert first - see SIM_SCALE() and CAR_STEP().
%
% NOTE on omega's sign: reading this mix as an inverse (which WHEEL_FORWARD does)
% makes omega>0 a CLOCKWISE turn, so this sign set and the "omega>0 = CCW"
% comment cannot both hold. The mix is kept verbatim because it is the frozen
% hardware/wiring interface; test_zone_scripts.m pins the behaviour that actually
% matters (which way the car strafes) against the recorded field positions. If a
% real turn ever comes out mirrored, this sign set is the single place to flip.

values = [vx + vy - omega, ...
          vx - vy + omega, ...
          vx - vy - omega, ...
          vx + vy + omega];

if nargin < 4 || isempty(P)
    P = sim_params();
end
if nargin < 5 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'scale') || isempty(opts.scale)
    scale = P.speed_scale;
else
    scale = opts.scale;
end

values = [vx + vy - omega, ...
          vx - vy + omega, ...
          vx - vy - omega, ...
          vx + vy + omega];

% Per-wheel normalization: scale so the fastest wheel hits the limit instead of
% clipping individually. Clipping one wheel alone would silently bend the
% intended motion into a different one.
peak = max(abs(values));
saturated = false;
if peak > 1
    values = values / peak;
    saturated = true;
end

w.values          = values;
w.scale           = scale;
w.travel_mm_per_s = values * scale * P.max_wheel_rpm * P.mm_s_per_rpm;
w.rpm             = values * scale * P.max_wheel_rpm .* P.wheel_signs;
w.saturated       = saturated;
end
