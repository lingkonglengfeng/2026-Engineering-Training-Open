function r = motion_plan(L_mm, v_mm_s, P)
%MOTION_PLAN Firmware trapezoidal/triangular move planner + the real profile.
%
%   r = MOTION_PLAN(distance_mm, speed_mm_s, P)
%
%   r.seconds        the time the firmware will DRIVE (drive_ms / 1000)
%   r.drive_ms       same, in ms, after ZDT_NAV_DRIVE_MIN_MS clamping
%   r.covered_mm     distance actually covered in drive_ms under accel/decel
%   r.overshoot_mm   covered_mm - L_mm   (>0 = drives past, <0 = stops short)
%   r.ref_seconds    plan_seconds(1000mm, ref_speed): the `t1000` divisor
%   r.peak_mm_s      peak speed actually reached inside drive_ms
%   r.triangular     true when the profile never reaches v_mm_s
%
% Faithful replica of ZDT_Nav_PlanSeconds() + ZDT_Nav_PlanDriveMs() in
% Core/Src/zdt_motor.c (lines ~1850-1925):
%
%   accel_mm = v^2 / (2*a)          decel_mm = v^2 / (2*d)
%   L >= accel+decel :  t = v/a + v/d + (L-accel-decel)/v          (trapezoid)
%   else             :  peak = sqrt(2L / (1/a + 1/d));  t = peak/a + peak/d
%   drive_ms = plan_seconds(L, v) / plan_seconds(1000, v_ref) * t1000
%
% THE POINT OF THIS FILE: the firmware turns that time into "drive the motors
% for drive_ms", not into "drive until the encoder says L". So if `t1000` is not
% exactly consistent with the speed model, every segment stops short or runs
% over - which is what `t1000` calibration is really trading away. This function
% reports `overshoot_mm` so the calibration can be seen instead of guessed.

if L_mm < 0
    L_mm = -L_mm;
end
if v_mm_s < 1
    v_mm_s = 1;      % firmware guard against divide-by-zero
end

tri = plan_seconds(L_mm, v_mm_s, P);
ref = plan_seconds(1000.0, P.ref_speed_mm_s, P);

drive_ms = tri / ref * P.time_1000mm_ms;
drive_ms = max(drive_ms, P.drive_min_ms);

% what the body really does in those drive_ms
[covered, peak, triangular] = integrate_profile(drive_ms / 1000, v_mm_s, P);

r = struct();
r.L_mm         = L_mm;
r.v_mm_s       = v_mm_s;
r.tri_seconds  = tri;
r.ref_seconds  = ref;
r.drive_ms     = drive_ms;
r.seconds      = drive_ms / 1000;
r.covered_mm   = covered;
r.overshoot_mm = covered - L_mm;
r.peak_mm_s    = peak;
r.triangular   = triangular;
end

% ----------------------------------------------------------------------
function t = plan_seconds(L_mm, v_mm_s, P)
%ZDT_Nav_PlanSeconds() - kinematic time to cover L at v with the ramp limits.
a = P.accel_mm_s2;
d = P.decel_mm_s2;
if v_mm_s < 1
    v_mm_s = 1;
end
accel_mm = v_mm_s^2 / (2 * a);
decel_mm = v_mm_s^2 / (2 * d);
if L_mm >= accel_mm + decel_mm
    t = v_mm_s / a + v_mm_s / d + (L_mm - accel_mm - decel_mm) / v_mm_s;
else
    peak = sqrt(2 * L_mm / (1 / a + 1 / d));
    t = peak / a + peak / d;
end
end

function [covered, peak, triangular] = integrate_profile(T_s, v_max, P)
%Distance covered in T seconds by a symmetric trapezoid capped at v_max.
%accel-limited: t_peak = v_max/a; decel-limited: t_peak = v_max/d.
a = P.accel_mm_s2;
d = P.decel_mm_s2;
triangular = false;

if T_s <= 0
    covered = 0; peak = 0; triangular = true; return
end

t_a = v_max / a;
t_d = v_max / d;
t_cruise = T_s - t_a - t_d;

if t_cruise >= 0
    peak = v_max;
    covered = 0.5 * a * t_a^2 + v_max * t_cruise + v_max * t_d - 0.5 * d * t_d^2;
else
    triangular = true;
    % solve for the peak speed reachable inside T with both limits
    if T_s <= t_a
        peak = a * T_s;
        covered = 0.5 * a * T_s^2;
    else
        % accelerate to peak then decelerate to 0, total time T_s
        ta = (d / (a + d)) * T_s;
        td = T_s - ta;
        peak = a * ta;
        covered = 0.5 * a * ta^2 + peak * td - 0.5 * d * td^2;
    end
end
end
