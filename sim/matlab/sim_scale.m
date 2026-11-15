function s = sim_scale(P)
%SIM_SCALE Body-mm/s and yaw-rad/s -> the NORMALIZED units wheel_inverse wants.
%
%   s.body   full-scale linear speed (mm/s)  = 1.0 normalized
%   s.yaw    full-scale yaw rate (rad/s)     = 1.0 normalized
%   s.lever  the lever arm used for the yaw conversion (mm)
%
% WHY THIS EXISTS - a real bug this caught:
% wheel_inverse / ChassisMotor_Mix() take a DIMENSIONLESS twist where 1.0 means
% "one wheel at CHASSIS_MAX_WHEEL_RPM". Feeding it mm/s instead looks fine in a
% unit test (the algebra is linear) but every real command then reads as a huge
% overspeed: the per-wheel normalization divides the whole twist down, and a
% 300 mm move silently shrinks to under 1 mm. Exactly that happened here -
% do_drive passed 532 mm/s and the car moved 0.74 mm.
%
% The quantities are all proportional to (wheel surface speed / wheel radius):
%   vx, vy   -> wrist scale = v_max
%   omega    -> same scale, but divided by the effective lever arm
% so one scalar converts all three consistently.

s = struct();
s.body  = P.max_wheel_rpm * P.mm_s_per_rpm;                 % ~1065 mm/s
s.lever = (abs(P.wheel_x_mm(1)) + abs(P.wheel_y_mm(2))) / 2; % 150 mm
s.yaw   = s.body / s.lever;                                 % ~7.10 rad/s
end
