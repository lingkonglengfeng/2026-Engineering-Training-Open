function P = sim_params(overrides)
%SIM_PARAMS Robot + navigation parameters, sourced from the FIRMWARE headers.
%
%   P = SIM_PARAMS()
%   P = SIM_PARAMS(struct('speed_scale', 0.62))
%
% Every default below is transcribed from a firmware header, with the source on
% the same line. Nothing here is a guess, and nothing here is re-derived by
% hand - derived quantities are computed at the bottom of this function.
%
% The rule (sim/docs/设计说明.md §5): defaults come from the firmware; a value
% you want to TRY goes in `overrides` and is echoed in the report, so it stays
% visible which numbers are real and which are a simulation hypothesis.

if nargin < 1 || isempty(overrides)
    overrides = struct();
end

% ---- wheel / drivetrain (Core/Inc/zdt_motor.h, Core/Inc/chassis_motor.h) ----
P.wheel_diameter_mm    = 81.39;    % implies ZDT_MILEAGE_MM_PER_REV = 255.694226
P.wheel_circumference_mm = 255.694226;   % ZDT_MILEAGE_MM_PER_REV (authoritative)
P.max_wheel_rpm        = 250.0;    % CHASSIS_MAX_WHEEL_RPM -> 1.0 ~= 1.065 m/s
P.accel_rpm_per_s      = 600.0;    % CHASSIS_ACCEL_RPM_PER_S
P.decel_rpm_per_s      = 900.0;    % CHASSIS_DECEL_RPM_PER_S
P.turn_max_wheel_rpm   = 90.0;     % CHASSIS_TURN_MAX_WHEEL_RPM  (precise turns)
P.turn_fast_wheel_rpm  = 120.0;    % CHASSIS_TURN_FAST_WHEEL_RPM (per-leg align)
P.wheel_signs          = [-1 1 -1 1];   % chassis_motor.h: FL FR RL RR
% wheel mounting, from the standard X layout (see docs/hardware/底盘方向与麦轮排布说明.md)
P.wheel_x_mm           = [150 150 -150 -150];   % FL FR RL RR, +x = forward
P.wheel_y_mm           = [-150 150 -150 150];   % +y = right
P.roller_angle_deg     = 45.0;     % X-type mecanum rollers

% ---- chip / control periods (Core/Src/zdt_motor.c, chassis_motor.h) --------
P.tick_ms              = 1.0;      % ZDT_Motor_Task osDelay(1)
P.cmd_period_ms        = 4.0;      % chassis velocity push period (was 20)
P.heading_hold_ms      = 80.0;     % straight-line heading hold period
P.heading_hold_gain    = 0.030;    % omega = clamp(0.030 * err, +-0.20)
P.heading_hold_omega_max = 0.20;
P.heading_deadband_deg = 0.2;
P.step_gap_ms          = 2.0;      % ZDT_NAV_STEP_GAP_MS
P.drive_min_ms         = 40.0;     % ZDT_NAV_DRIVE_MIN_MS

% ---- navigation tunables (Core/Inc/nav_params.h) --------------------------
% These are the ONLY three navigation values adjustable on the real car, via
% `spd<v>` / `t1000<v>` / `cnt<v>` (+ `save`). A tuning result should therefore
% be reported as those commands, not as raw mm/s.
P.speed_scale          = 0.50;     % `spd` default
P.time_1000mm_ms       = 2050.7;   % `t1000`; theoretical value at spd=0.50:
                                   % plan_seconds(1000, 532.7mm/s)*1000 = 2050.7
                                   % (the old 1900 ms default was measured under a
                                   %  superseded speed model and under-runs by ~7%)
P.counts_per_rev       = 65536.0;  % `cnt` (vendor Emm spec; verify on hardware)
P.position_mode        = 0;        % `posmode`

% ---- executor thresholds (Core/Src/zdt_motor.c, Core/Inc/grid_nav.h) ------
P.align_tolerance_deg  = 3.0;      % ZDT_NAV_ALIGN_TOLERANCE_DEG
P.seg_end_tol_deg      = 20.0;     % segment-end heading drift cap
P.lane_snap_deadband_mm = 40.0;    % ZDT_NAV_LANE_SNAP_DEADBAND_MM
P.strafe_max_cells     = 3;        % = mapspec.executor.strafe_max_cells
P.turn_done_deg        = 90.0;     % nominal in-place turn for a corner

% ---- apply explicit overrides --------------------------------------------
names = fieldnames(overrides);
for i = 1:numel(names)
    if ~isfield(P, names{i})
        error('sim_params:unknown', ...
            'override "%s" is not a known parameter (typo?)', names{i});
    end
    P.(names{i}) = overrides.(names{i});
end
P.overrides = names;

% ---- derived (never hand-copied) ----------------------------------------
P.mm_s_per_rpm   = P.wheel_circumference_mm / 60.0;        % 4.2616 mm/s per RPM
P.accel_mm_s2    = P.accel_rpm_per_s * P.mm_s_per_rpm;     % ~2557 mm/s^2
P.decel_mm_s2    = P.decel_rpm_per_s * P.mm_s_per_rpm;     % ~3835 mm/s^2
P.speed_mm_s     = P.speed_scale * P.max_wheel_rpm * P.mm_s_per_rpm;
P.ref_speed_mm_s = 0.50 * P.max_wheel_rpm * P.mm_s_per_rpm;   % t1000 reference
P.turn_omega_dps = P.turn_fast_wheel_rpm * P.mm_s_per_rpm / ...
                   ((abs(P.wheel_x_mm(1)) + abs(P.wheel_y_mm(2))) / 2) ...
                   * 180 / pi;   % body yaw rate at the fast turn wheel speed
end
