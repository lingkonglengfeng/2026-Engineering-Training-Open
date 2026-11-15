function car = car_init(x_mm, y_mm, heading_map)
%CAR_INIT Create the car state used by CAR_STEP / RUN_LEG.
%
%   car = CAR_INIT(2250, 2250, [0 -1])    % start zone 2: nose into the field
%
%   car.x, car.y    MAP-frame position of the car CENTRE (mm)
%   car.h           heading angle in the MAP frame (rad); F = (cos h, sin h)
%   car.t           simulated seconds
%   car.omega       yaw rate for the NEXT step (normalized); nonzero = arc
%   car.path        Nx2 trail of (x, y) for plotting
%
% The INVERSE of the nose binding: instead of trusting a hand-written
% "nose = +y", the heading vector is converted to an angle here, so a wrong
% placement shows up as a wrong angle rather than as a silent sign error.

if nargin < 1 || isempty(x_mm), x_mm = 2250; end
if nargin < 2 || isempty(y_mm), y_mm = 2250; end
if nargin < 3 || isempty(heading_map), heading_map = [0 -1]; end

n = norm(heading_map);
if n == 0
    error('car_init:heading', 'heading vector must be non-zero');
end
heading_map = heading_map / n;

car = struct();
car.x = x_mm;
car.y = y_mm;
car.h = atan2(heading_map(2), heading_map(1));
car.t = 0;
car.omega = 0;                 % yaw rate for the NEXT step, rad/s (0 = pure translation)
car.path = [x_mm, y_mm];
car.wheel_values = [0 0 0 0];
car.wheel_rpm = [0 0 0 0];
car.wheel_saturated = false;
car.arc_omega = 0;
car.last_winfo = struct();
car.footprint_mm = 300;
end
