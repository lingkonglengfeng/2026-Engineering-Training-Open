function [vx, vy, omega, info] = wheel_forward(rpm, P, err)
%WHEEL_FORWARD 4 wheel speeds -> body twist. THE error-injection point.
%
%   [vx, vy, omega, info] = WHEEL_FORWARD(rpm, P)
%   [vx, vy, omega, info] = WHEEL_FORWARD(rpm, P, err)
%
%   rpm   1x4 wheel RPM in WHEEL_INVERSE's channel order (FL FR RL RR), as sent,
%         i.e. still carrying P.wheel_signs
%   err   optional physics-error hook (phase 2: slip, roller losses, drag):
%           .per_wheel  1x4 multiplier on each wheel's effective speed
%           .common     scalar multiplier on all wheels
%           .omega_bias additive yaw-rate bias (normalized units)
%   info.reconstructed  true when err actually changed anything
%
% Why this function exists even though with an ideal `err` it is the exact
% inverse of WHEEL_INVERSE, and therefore numerically a no-op:
%   Real mecanum error - roller friction, a slipping wheel, uneven grip, a
%   dragging castor - happens physically at the WHEEL. Reconstructing the body
%   motion from wheel speeds is the only physically meaningful place to inject
%   it, which keeps phase 2 confined to this one function.

if nargin < 3 || isempty(err)
    err = struct();
end

raw   = rpm(:).' ./ P.wheel_signs;                    % undo mounting signs
ideal = raw ./ (P.speed_scale * P.max_wheel_rpm);     % back to normalized

eff = ideal;
if isfield(err, 'per_wheel') && ~isempty(err.per_wheel)
    eff = eff .* err.per_wheel(:).';
end
if isfield(err, 'common') && ~isempty(err.common)
    eff = eff * err.common;
end

% ---------------------------------------------------------------------------
% DERIVATION. WHEEL_INVERSE encodes the firmware mix
% (ChassisMotor_Mix, Core/Src/chassis_motor.c:157-160), with w1..w4 = FL..RR:
%
%   w1 = vx + vy - om        w3 = vx - vy - om
%   w2 = vx - vy + om        w4 = vx + vy + om
%
% Solve for the twist:
%   w1 + w2 + w3 + w4  = 4*vx        (all four add to 4vx)
%   w1 - w2 - w3 + w4  = 4*vy        (w1+w4 = 2vx+2vy, w2+w3 = 2vx-2vy)
%   -w1 + w2 - w3 + w4 = 4*omega     (w2-w1 = -2vy+2om, w4-w3 = +2vy+2om)
%
% Regression patterns - write these into any rewrite, because a sign error here
% does not fail loudly, it just makes every translation curve:
%   pure strafe [1, -1, -1, 1] -> (0, 1, 0)
%   pure yaw    [-1, 1, -1, 1] -> (0, 0, 1)
%
% The self-check below enforces the round trip, so a wrong sign raises instead
% of silently corrupting every trajectory.
% ---------------------------------------------------------------------------
w1 = eff(1); w2 = eff(2); w3 = eff(3); w4 = eff(4);

vx    =  ( w1 + w2 + w3 + w4) / 4;
vy    =  ( w1 - w2 - w3 + w4) / 4;
omega =  (-w1 + w2 - w3 + w4) / 4;

if ~isfield(err, 'per_wheel') && ~isfield(err, 'common') && ~isfield(err, 'omega_bias')
    chk = [vx + vy - omega, vx - vy + omega, vx - vy - omega, vx + vy + omega];
    if max(abs(chk - eff)) > 1e-9
        error('wheel_forward:inconsistent', ...
            'forward/inverse mismatch: rebuilt %s from %s', ...
            mat2str(chk, 6), mat2str(eff, 6));
    end
end

if isfield(err, 'omega_bias') && ~isempty(err.omega_bias)
    omega = omega + err.omega_bias;
end

info = struct();
info.effective_rpm = eff * P.speed_scale * P.max_wheel_rpm .* P.wheel_signs;
info.reconstructed = ~isequal(eff, ideal);
info.ideal = ideal;
end
