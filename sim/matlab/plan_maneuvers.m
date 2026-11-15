function plan = plan_maneuvers(segs, spec, heading)
%PLAN_MANEUVERS What the car actually DOES with an A* path.
%
%   plan = PLAN_MANEUVERS(segs, spec)         % heading unknown
%   plan = PLAN_MANEUVERS(segs, spec, 'U')    % car already faces map +y
%
%   plan.turns    number of in-place turns
%   plan.strafes  number of pure strafes (no heading change)
%   plan.actions  {1 x M} strings, e.g. "turn+go U1600", "strafe L100 (no turn)"
%
% Mirror of the firmware's ZDT_Nav_SegmentStrafeSign() decision:
%
%   * a segment PERPENDICULAR to the current heading and <= STRAFE_MAX_CELLS
%     (3 cells = 150 mm) long is driven as a pure strafe - the nose does not
%     move, so NO turn is needed before or after it.
%   * longer perpendicular segments and reversals (U-turns) still turn in place.
%
% Why this rule exists: turning twice for a 1-cell jog costs ~2 s and stacks two
% +-0.5 deg alignment residuals. Strafing keeps the nose fixed and introduces
% no angular error at all. A*'s turn penalty (120) still shapes the path; this
% only decides how the resulting segments are executed.

if nargin < 3
    heading = '';
end

STRAFE_MAX_CELLS = spec.executor.strafe_max_cells;

turns    = 0;
strafes  = 0;
actions  = {};
h        = heading;

for i = 1:size(segs, 1)
    L     = char(segs(i, 1));
    cells = segs(i, 2);
    mm    = segs(i, 3);

    if isempty(h)
        % First segment with no known heading: the firmware also strafes a tiny
        % lateral jog, so report it the same way.
        if cells <= STRAFE_MAX_CELLS && ~ismember(L, {'U', 'D'})
            strafes = strafes + 1;
            actions{end + 1} = sprintf('strafe %s%d', L, mm);   %#ok<AGROW>
        else
            h = L;
            turns = turns + 1;
            actions{end + 1} = sprintf('turn+go %s%d', L, mm);  %#ok<AGROW>
        end
        continue
    end

    if L == h
        actions{end + 1} = sprintf('go %s%d', L, mm);           %#ok<AGROW>
    elseif cells <= STRAFE_MAX_CELLS
        strafes = strafes + 1;
        actions{end + 1} = sprintf('strafe %s%d (no turn)', L, mm);   %#ok<AGROW>
    else
        h = L;
        turns = turns + 1;
        actions{end + 1} = sprintf('turn+go %s%d', L, mm);      %#ok<AGROW>
    end
end

plan = struct('turns', turns, 'strafes', strafes, 'actions', {actions}, ...
              'heading', h);
end
