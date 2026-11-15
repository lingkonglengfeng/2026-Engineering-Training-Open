function car_draw_update(h, car)
%CAR_DRAW_UPDATE Move the existing car handles to the current pose.
%
% Split out of CAR_DRAW so an animation loop can update the car every frame
% without redrawing the 2304-cell field.

half = car.footprint_mm / 2;
F = [cos(car.h), sin(car.h)];
R = [-F(2), F(1)];             % firmware literal form; see car_step.m for why R = (-F.y, F.x)

% body corners in map frame
c = [ car.x + half * (-F(1) - R(1)), car.y + half * (-F(2) - R(2));
      car.x + half * ( F(1) - R(1)), car.y + half * ( F(2) - R(2));
      car.x + half * ( F(1) + R(1)), car.y + half * ( F(2) + R(2));
      car.x + half * (-F(1) + R(1)), car.y + half * (-F(2) + R(2)) ];
set(h.carPatch, 'XData', c(:, 1), 'YData', c(:, 2));

% nose indicator
set(h.nose, 'XData', [car.x, car.x + half * F(1) * 1.35], ...
            'YData', [car.y, car.y + half * F(2) * 1.35]);

% wheels: 60mm wide, 30mm thick, mounted at +-150mm forward / +-150mm right
wx = [150 150 -150 -150];
wy = [-150 150 -150 150];
for i = 1:4
    cx = car.x + wx(i) * F(1) + wy(i) * R(1);
    cy = car.y + wx(i) * F(2) + wy(i) * R(2);
    along = [F(1), F(2)];
    across = [R(1), R(2)];
    w = 30; l = 60;
    p = [ cx - l/2*along(1) - w/2*across(1), cy - l/2*along(2) - w/2*across(2);
          cx + l/2*along(1) - w/2*across(1), cy + l/2*along(2) - w/2*across(2);
          cx + l/2*along(1) + w/2*across(1), cy + l/2*along(2) + w/2*across(2);
          cx - l/2*along(1) + w/2*across(1), cy - l/2*along(2) + w/2*across(2) ];
    set(h.wheelPatches(i), 'XData', p(:, 1), 'YData', p(:, 2));
end
end
