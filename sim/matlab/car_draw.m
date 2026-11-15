function h = car_draw(map, car, log_or_opts)
%CAR_DRAW Render the field, the car body and its four wheels, plus the trail.
%
%   h = CAR_DRAW(map, car)
%   h = CAR_DRAW(map, car, struct('axes', ax, 'title', 'goto raw'))
%
%   h.ax, h.carPatch, h.wheelPatches, h.trail
%
% Uses plain 2-D patches and lines only - no toolbox is required, which keeps
% the simulator runnable anywhere. Everything is drawn in the MAP frame (y UP,
% origin at the field's lower-left inner corner), the frame the rule book and
% every report uses; the internal cy-downward grid is never shown to a human.

if nargin < 3 || isempty(log_or_opts)
    opts = struct();
else
    opts = log_or_opts;
end

spec = map.spec;
F = spec.field_mm;

if isfield(opts, 'axes') && ~isempty(opts.axes)
    ax = opts.axes;
else
    fig = figure('Color', 'w', 'Name', 'field + car', 'NumberTitle', 'off');
    ax = axes('Parent', fig);
end
cla(ax); hold(ax, 'on');

% ---- ground: kind-coloured cells (walkable overlay shown by edges) ------
col = struct('ROAD', [0.82 0.82 0.82], 'RING', [0.80 0.80 0.80], ...
             'BLOCKED', [0.99 0.94 0.60], 'START', [0.60 0.75 0.95], ...
             'WORK', [0.97 0.97 0.97], 'PAD', [0.72 0.62 0.45]);

[kr, kc] = size(map.kind);
for r = 1:kr
    for c = 1:kc
        t = map.kind{r, c};
        x0 = (c - 1) * spec.cell_mm;
        y0 = F - r * spec.cell_mm;          % row r counts down from the top
        rectangle(ax, 'Position', [x0, y0, spec.cell_mm, spec.cell_mm], ...
            'FaceColor', col.(t), 'EdgeColor', 'none');
    end
end

% ---- walkable overlay: a dot where the CAR CENTRE may stand ------------
[wrr, wcc] = find(map.walkable);
xs = (wcc - 1) * spec.cell_mm + spec.cell_mm / 2;
ys = F - wrr * spec.cell_mm + spec.cell_mm / 2;
plot(ax, xs, ys, '.', 'Color', [0.30 0.62 0.34], 'MarkerSize', 3);

% ---- pad rectangles (no-contact table tops) ----------------------------
for i = 1:size(map.pads, 1)
    r = map.pads(i, :);
    rectangle(ax, 'Position', [r(1), r(3), r(2) - r(1), r(4) - r(3)], ...
        'FaceColor', 'none', 'EdgeColor', [0.45 0.28 0.10], 'LineWidth', 1.5, ...
        'LineStyle', '--');
end

% ---- waypoints ---------------------------------------------------------
names = fieldnames(spec.waypoints_mm);
for i = 1:numel(names)
    p = spec.waypoints_mm.(names{i});
    plot(ax, p(1), p(2), 'o', 'MarkerSize', 3, ...
        'MarkerEdgeColor', [0.35 0.35 0.35], 'MarkerFaceColor', [0.35 0.35 0.35]);
    text(ax, p(1) + 25, p(2) + 25, names{i}, 'FontSize', 6, 'Color', [0.3 0.3 0.3]);
end

% ---- trail -------------------------------------------------------------
h.trail = plot(ax, car.path(:, 1), car.path(:, 2), '-', ...
    'Color', [0.85 0.20 0.20], 'LineWidth', 1.2);

% ---- car body + 4 mecanum wheels ---------------------------------------
half = car.footprint_mm / 2;
h.carPatch = patch(ax, 'XData', 0, 'YData', 0, 'FaceColor', [0.15 0.35 0.75], ...
    'FaceAlpha', 0.75, 'EdgeColor', [0.05 0.15 0.45], 'LineWidth', 1.2);
h.wheelPatches = gobjects(1, 4);
for i = 1:4
    h.wheelPatches(i) = patch(ax, 'XData', 0, 'YData', 0, ...
        'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'none');
end
h.nose = plot(ax, 0, 0, '-', 'Color', [1 1 0.2], 'LineWidth', 2);
car_draw_update(h, car);

% ---- frame cosmetics ---------------------------------------------------
axis(ax, 'equal');
xlim(ax, [-60, F + 60]); ylim(ax, [-60, F + 60]);
xlabel(ax, 'x  (map mm, origin = field lower-left)');
ylabel(ax, 'y  (map mm, UP)');
grid(ax, 'on');
set(ax, 'GridAlpha', 0.15);
title(ax, 'smart-carry field: 48x48 @50mm, car 300mm; green dots = legal car centres');

% Colour legend, in Chinese and drawn on the axes: the same information in a
% console message would arrive as mojibake depending on the code page.
legend_txt = { ...
    '灰 = 十字灰色车道（400mm）', ...
    '淡黄 = 四块 450x450 禁区', ...
    '蓝 = 启停区（300x300）', ...
    '棕虚线 = 工位台面（不可接触）', ...
    '绿点 = 车心可站的格（734 个）', ...
    '红线 = 本车轨迹', ...
    '蓝方块 = 车体（300x300）', ...
    '黑块 = 四个麦轮'};
text(ax, F * 0.02, F * 0.30, legend_txt, 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
    'FontSize', 8, 'FontName', 'Microsoft YaHei', 'Color', [0.25 0.25 0.25], ...
    'BackgroundColor', [1 1 1], 'EdgeColor', [0.8 0.8 0.8]);

h.ax = ax;
hold(ax, 'off');
end
