function sim_png(opts)
%SIM_PNG Render the simulation to PNG files. No window needed.
%
%   sim_png()      writes sim\out\*.png
%
% WHY THIS EXISTS: MATLAB's windowed desktop does not survive in every
% environment (it starts, then exits), so "open the GUI and look" is not a
% reliable instruction. -batch always works, and -batch can still write image
% files, so the figures are exported instead of displayed.
%
% Output (sim\out\):
%   field.png              the field alone: yellow zones, start zones, pads,
%                          the legal car-centre cells, and the named stop points
%   leg_<from>_<to>.png    one leg: plan, start/end pose, executed trail, numbers
%   mission.png            all 9 legs on one field
%
% NOTE on variable naming: an earlier version of this file reused `f` for both a
% figure handle and a loop counter, which silently invalidated the graphics
% handles mid-render. Graphics handles and counters never share a name here.

if nargin < 1 || isempty(opts)
    opts = struct();
end
opts = setdef(opts, 'outdir', fullfile(repo_root(), 'sim', 'out'));
if ~exist(opts.outdir, 'dir')
    mkdir(opts.outdir);
end

P    = sim_params();
spec = load_mapspec();
map  = build_grid(spec);

% ---- 1. the field on its own --------------------------------------------
figField = figure('Color', 'w', 'Visible', 'off', 'Position', [0 0 900 900]);
axField = axes('Parent', figField);
car_draw(map, car_init(2250, 2250, [0 -1]), struct('axes', axField));
title(axField, sprintf('field: 48x48 @50mm, car 300mm, %d legal car centres', ...
    nnz(map.walkable)));
export_png(figField, fullfile(opts.outdir, 'field.png'));
close(figField);

% ---- 2. one image per leg, plus one combined ---------------------------
legs = {'h1','exit1'; 'exit1','raw'; 'raw','rough2'; 'rough2','temp2'; ...
        'temp2','qr'; 'qr','h2'; 'h2','exit2'; 'exit2','rough2'; 'h2','h1'};

figAll = figure('Color', 'w', 'Visible', 'off', 'Position', [0 0 900 900]);
axAll = axes('Parent', figAll);
car_draw(map, car_init(2250, 2250, [0 -1]), struct('axes', axAll));
hold(axAll, 'on');
cols = lines(size(legs, 1));

for k = 1:size(legs, 1)
    a = legs{k, 1}; b = legs{k, 2};
    pa = spec.waypoints_mm.(a);
    if a(1) == 'h'
        nose = spec.zones.(sprintf('x%s', a(2))).nose_map;
    else
        nose = [0 1];
    end
    car0 = car_init(pa(1), pa(2), nose);
    out = run_leg(map, car0, a, b, P, struct('log', sim_log(struct('enabled', false))));

    figLeg = figure('Color', 'w', 'Visible', 'off', 'Position', [0 0 900 900]);
    axLeg = axes('Parent', figLeg);
    hLeg = car_draw(map, car0, struct('axes', axLeg));
    hold(axLeg, 'on');

    % planned path (cell centres)
    if ~isempty(out.path)
        px = out.path(:, 1) * spec.cell_mm + spec.cell_mm / 2;
        py = (spec.size - 1 - out.path(:, 2)) * spec.cell_mm + spec.cell_mm / 2;
        plot(axLeg, px, py, '-', 'Color', [0.20 0.45 0.85], 'LineWidth', 1.0);
    end
    % executed trail
    if ~isempty(out.samples)
        sx = arrayfun(@(s) s.x, out.samples);
        sy = arrayfun(@(s) s.y, out.samples);
        plot(axLeg, sx, sy, '-', 'Color', [0.85 0.15 0.15], 'LineWidth', 1.6);
    end
    plot(axLeg, pa(1), pa(2), 'o', 'MarkerSize', 9, ...
        'MarkerFaceColor', [0.10 0.65 0.20], 'MarkerEdgeColor', 'k');
    car_draw_update(hLeg, out.car);

    text(axLeg, 60, 2330, sprintf(['%s -> %s\n%.2f s    %d turns   %d strafes\n' ...
        'drive dev %.1f mm   clearance %.0f mm\n%s'], ...
        a, b, out.seconds, out.turns, out.strafes, out.err_mm, ...
        out.verdict.min_clearance_mm, strjoin(out.actions, ' | ')), ...
        'FontSize', 9, 'FontName', 'Microsoft YaHei', ...
        'BackgroundColor', 'w', 'EdgeColor', [0.7 0.7 0.7], ...
        'VerticalAlignment', 'top');
    title(axLeg, sprintf('leg  %s  ->  %s', a, b));
    export_png(figLeg, fullfile(opts.outdir, sprintf('leg_%s_%s.png', a, b)));
    close(figLeg);

    if ~isempty(out.samples)
        sx = arrayfun(@(s) s.x, out.samples);
        sy = arrayfun(@(s) s.y, out.samples);
        plot(axAll, sx, sy, '-', 'Color', cols(k, :), 'LineWidth', 1.4);
        mid = round(numel(sx) / 2);
        text(axAll, sx(mid) + 30, sy(mid), sprintf('%s->%s', a, b), ...
            'FontSize', 7, 'Color', cols(k, :));
    end
end
title(axAll, 'all 9 task legs (one colour per leg)');
export_png(figAll, fullfile(opts.outdir, 'mission.png'));
close(figAll);

fprintf('\nPNG written to %s\n', opts.outdir);
fprintf('  field.png      the field, zones, and the legal car centres\n');
fprintf('  leg_*.png      one per task leg (9 files)\n');
fprintf('  mission.png    all legs on one field\n\n');
end

% ----------------------------------------------------------------------
function export_png(fig, path)
set(fig, 'PaperPositionMode', 'auto');
print(fig, path, '-dpng', '-r110');
end

function r = repo_root()
here = fileparts(mfilename('fullpath'));      % <repo>/sim/matlab
r = fileparts(fileparts(here));
end

function o = setdef(o, f, v)
if ~isfield(o, f) || isempty(o.(f))
    o.(f) = v;
end
end
