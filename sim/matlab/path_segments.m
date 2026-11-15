function segs = path_segments(path, spec)
%PATH_SEGMENTS Merge an A* cell path into straight segments.
%
%   segs = PATH_SEGMENTS(path, spec)
%
%   path  Nx2 0-based [cx cy] from ASTAR_GRID
%   segs  Mx3 [letter, cells, mm] where letter is:
%           'U' = map +y      'D' = map -y
%           'R' = map +x      'L' = map -x
%
% CAREFUL with the letters: cell row cy increases DOWNWARD, so a step with
% dy>0 moves toward map -y and is therefore 'D'. Getting this backwards is a
% documented real bug ("段方向字母写反 -> 日志里 D/U 与实际相反").

segs = zeros(0, 3);
if size(path, 1) < 2
    return
end

letter_of = @(dx, dy) ...
    char(82 * (dx > 0) + 76 * (dx < 0) + 68 * (dy > 0) + 85 * (dy < 0));
% 82='R' 76='L' 68='D' 85='U'

for i = 2:size(path, 1)
    dx = path(i, 1) - path(i - 1, 1);
    dy = path(i, 2) - path(i - 1, 2);
    L  = letter_of(dx, dy);
    if ~isempty(segs) && segs(end, 1) == L
        segs(end, 2) = segs(end, 2) + 1;
    else
        segs(end + 1, :) = [L, 1, 0];   %#ok<AGROW>
    end
end
segs(:, 3) = segs(:, 2) * spec.cell_mm;
end
