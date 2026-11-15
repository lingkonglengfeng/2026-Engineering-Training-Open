function [car, sample] = car_step(car, dv_map, dt, P, err)
%CAR_STEP Advance the car model by dt, driven through the wheel layer.
%
%   [car, sample] = CAR_STEP(car, displacement_map_mm, dt, P, err)
%
%   car   state struct, see CAR_INIT
%   dv_map  [dx dy] desired MAP-frame displacement for this step (mm, may be 0)
%   err   optional physics-error hook passed to WHEEL_FORWARD (phase 2)
%
% Pipeline, in this order and nowhere else:
%
%   map delta  --(nose binding)-->  body (fwd, right) velocities
%              --(WHEEL_INVERSE)--> 4 wheel speeds
%              --(WHEEL_FORWARD + err)--> body twist the body really gets
%              --> pose integration
%
% The nose binding is transcribed from ZDT_Nav_GetBodyVelocity()
% (Core/Src/zdt_motor.c:1616-1626), which is the firmware's single frame map:
%
%   fwd   = d . F          F = (cos h, sin h)      nose, map frame
%   right = d . R          R = (-F.y,  F.x)        what the firmware calls "vy"
%
% so  map = fwd*F + right*R.
%
% CAUTION - this R is a 90 deg COUNTER-clockwise rotation of F, i.e. it points to
% the car's LEFT in a right-handed (y-up) map frame, even though the firmware
% comment and the header doc call it "body right". The two disagree, and this
% file deliberately follows the CODE, because the code is what the recorded field
% behaviour matches:
%
%   start zone 2 (h2 = (2250,2250), nose map -y) -- `&leave`
%     = forward 300 then strafe "RIGHT" 125
%     -> recorded end position is exit2 = (2125,1950)
%     -> the x motion is -125, which needs R = (-1, 0); R = (F.y, -F.x) would
%        give (1,0) and move the car to x=2362, 237 mm away from the documented
%        exit point, i.e. off the outside of the mat.
%
% test_zone_scripts.m pins this against exit1/exit2/h1/h2 and is the check that
% catches a mirrored right vector. If the car is ever re-bound at the field so
% that "RIGHT" means the other way, this line and that test change together.
% See docs/hardware/底盘方向与麦轮排布说�?md §7.3 for the project's own notes on
% this ambiguity (its worked example and its stated formula disagree).

if nargin < 5
    err = struct();
end

sample = struct('t', car.t, 'x', car.x, 'y', car.y, 'h', car.h);

if isempty(dv_map)
    return
end

F = [cos(car.h), sin(car.h)];
R = [-F(2), F(1)];             % firmware literal form; see car_step.m for why R = (-F.y, F.x)

d_body_fwd   = dot(dv_map, F);
d_body_right = dot(dv_map, R);

if dt <= 0
    return
end

vx_mm_s = d_body_fwd   / dt;      % physical body velocity, mm/s
vy_mm_s = d_body_right / dt;
omega_rad_s = car.omega;          % physical yaw rate, rad/s (0 = no arc)

% Normalize ONCE, here. wheel_inverse expects a dimensionless twist where 1.0
% is "one wheel at CHASSIS_MAX_WHEEL_RPM"; handing it mm/s is the bug that made
% a 300 mm move travel 0.7 mm (see sim_scale.m).
sc = sim_scale(P);
w = wheel_inverse(vx_mm_s / sc.body, vy_mm_s / sc.body, omega_rad_s / sc.yaw, P);

% wheels -> body. `err` is the ONLY place phase-2 physics (slip, roller losses,
% drag) will be injected; with an ideal err this is the exact inverse above.
[vx_n, vy_n, omega_n, winfo] = wheel_forward(w.rpm, P, err);

% back to physical units for the pose integration
vx_b = vx_n * sc.body;
vy_b = vy_n * sc.body;
omega_b = omega_n * sc.yaw;

% ---- integrate the pose with the body twist actually achieved ----------
car.x = car.x + (vx_b * F(1) + vy_b * R(1)) * dt;
car.y = car.y + (vx_b * F(2) + vy_b * R(2)) * dt;
car.h = wrap_pi(car.h + omega_b * dt);
car.t = car.t + dt;

car.wheel_values = w.values;
car.wheel_rpm    = w.rpm;
car.wheel_saturated = w.saturated;
car.last_winfo   = winfo;
car.path         = [car.path; car.x, car.y];
car.arc_omega    = omega_rad_s;

sample = struct('t', car.t, 'x', car.x, 'y', car.y, 'h', car.h, ...
                'vx', vx_b, 'vy', vy_b, 'omega', omega_b, ...
                'rpm', w.rpm, 'saturated', w.saturated);
end

function a = wrap_pi(a)
a = mod(a + pi, 2 * pi) - pi;
end
