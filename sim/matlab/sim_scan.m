function report = sim_scan(opts)
%SIM_SCAN Sweep the navigation parameters and report the tuning trade-offs.
%
%   sim_scan()                          % default sweep
%   r = sim_scan(struct('run', false))  % return a variant table, print nothing
%   r = sim_scan(struct('csv', 'tmp\scan.csv'))
%
% This is the deliverable that answers "what should spd / t1000 / TURN_COST be".
% It reuses SIM_REPORT_EXEC (the same executor SIM_REPORT uses, minus printing),
% so a number here is the same number the report prints - they cannot drift.
%
% WHAT IS SWEPT, and why each one:
%   speed_scale      the firmware `spd` knob: drives EVERY segment faster.
%                    Cost: more wheel RPM, longer stopping distance.
%   time_1000mm_ms   the firmware `t1000` knob. The firmware commands a DURATION,
%                    so this decides over/under-run per segment, NOT speed.
%                    Its only real job is to match the speed model (k -> 1).
%   turn_cost        A* GRID_NAV_TURN_COST: how hard the planner avoids corners.
%                    Every corner costs a real in-place turn (~1.8 s + a
%                    +-0.5 deg residual), so this is a genuine trade, not taste.
%   lane_cost        A* lane-centre bias: pull routes toward 275/1200/2125, at
%                    the price of a longer path. Interacts with turn_cost, which
%                    is why they are swept as a pair rather than one at a time.
%
% Metrics per variant:
%   seconds     total simulated driving time over the 9 task legs
%   turns       total in-place turns (each one is time + alignment residual)
%   legs_ok     legs that reach their planned cell centre within tolerance
%   min_clear   tightest clearance from the body to any forbidden rectangle
%   min_band    tightest margin from the body to the FIELD EDGE
%
% A variant that breaks the map (a leg with no route, or an illegal body pose)
% is reported as such and sorted to the bottom - never silently dropped.

if nargin < 1 || isempty(opts)
    opts = struct();
end
opts = setdefault(opts, 'run', true);

% ---- build the variant matrix -------------------------------------------
% Baseline first, then one-at-a-time sweeps so each effect is readable, then a
% small 2-D grid where the interesting interaction lives.
V = {};
V{end + 1} = variant('baseline             ', 0.50, 2050.7, 120, 3);
for s = [0.35 0.75 1.00]
    V{end + 1} = variant(sprintf('spd=%.2f             ', s), s, 2050.7, 120, 3);   %#ok<AGROW>
end
for t = [1800 1900 2200]
    V{end + 1} = variant(sprintf('t1000=%d          ', t), 0.50, t, 120, 3);        %#ok<AGROW>
end
for tc = [0 60 240 600]
    V{end + 1} = variant(sprintf('turn_cost=%d       ', tc), 0.50, 2050.7, tc, 3);   %#ok<AGROW>
end
for lc = [0 6 12]
    V{end + 1} = variant(sprintf('lane_cost=%d        ', lc), 0.50, 2050.7, 120, lc); %#ok<AGROW>
end
for tc = [60 240]
    for lc = [0 12]
        V{end + 1} = variant(sprintf('tc=%d lane=%d       ', tc, lc), 0.50, 2050.7, tc, lc); %#ok<AGROW>
    end
end

% ---- run them ------------------------------------------------------------
rows = struct([]);
for i = 1:numel(V)
    % hand the engine only keys it knows; `label` is display-only metadata and
    % SIM_REPORT_EXEC rejects unknown keys on purpose
    o = rmfield(V{i}, 'label');
    r = sim_report_exec(o);
    r.label = V{i}.label;
    if isempty(rows)
        rows = r;
    else
        rows(end + 1) = r;   %#ok<AGROW>
    end
end

% ---- report --------------------------------------------------------------
report = struct();
report.variants = rows;

if ~opts.run
    return
end

fprintf('\n%s\n', repmat('=', 1, 96));
fprintf('PARAMETER SCAN  (9 task legs, planned and executed for every variant)\n');
fprintf('%s\n', repmat('=', 1, 96));
fprintf('%-20s %8s %6s %6s %9s %10s %9s  %s\n', ...
    'variant', 'sec', 'turns', 'legs', 'worst_dev', 'min_clear', 'min_band', 'note');
fprintf('%s\n', repmat('-', 1, 96));

ok = [rows.legs_ok];
for i = 1:numel(rows)
    r = rows(i);
    note = '';
    if r.n_nopath > 0
        note = sprintf('NO PATH on %d leg(s) - variant invalid', r.n_nopath);
    elseif ~r.legal
        note = 'ILLEGAL body pose - variant invalid';
    elseif r.worst_dev_mm > 60
        note = 'arrival outside snapping scale';
    end
    fprintf('%-20s %8.2f %6d %3d/%-2d %9.1f %10.1f %9.1f  %s\n', ...
        r.label, r.seconds, r.turns, r.legs_ok, r.n_legs, r.worst_dev_mm, ...
        r.min_clearance_mm, r.min_band_mm, note);
end

fprintf('\n%s\n', repmat('-', 1, 96));
fprintf('HOW TO READ THIS\n');
fprintf('  sec        total driving time - the number to minimise, but see below\n');
fprintf('  turns      in-place turns. Each costs ~1.8 s AND leaves a +-0.5 deg\n');
fprintf('             residual, so "fewer turns" is worth real time later even\n');
fprintf('             when it looks slower here.\n');
fprintf('  min_clear  tightest distance from the body to a yellow zone / table.\n');
fprintf('             Already ~10 mm at a corner, so a variant that REDUCES it is\n');
fprintf('             not worth any time saving.\n');
fprintf('  min_band   tightest margin to the field edge. Off the mat ends the run.\n');

% ---- recommendations -----------------------------------------------------
v = valid_only(rows);
if isempty(v)
    fprintf('\nNo valid variant - nothing to recommend.\n');
else
    [~, ib] = min([v.seconds]);
    [~, it] = min([v.turns]);
    fprintf('\n%s\n', repmat('-', 1, 96));
    fprintf('FASTEST valid variant : %s (%.2f s, %d turns)\n', ...
        v(ib).label, v(ib).seconds, v(ib).turns);
    fprintf('FEWEST turns          : %s (%.2f s, %d turns)\n', ...
        v(it).label, v(it).seconds, v(it).turns);

    base = rows(1);
    fprintf('\nAgainst the baseline (%.2f s, %d turns):\n', base.seconds, base.turns);
    for i = 1:numel(v)
        if strcmp(strtrim(v(i).label), 'baseline')
            continue
        end
        d = v(i).seconds - base.seconds;
        dt = v(i).turns - base.turns;
        if abs(d) > 0.05 || dt ~= 0
            fprintf('   %-20s %+6.2f s  %+3d turns\n', strtrim(v(i).label), d, dt);
        end
    end
end

fprintf('\nNOTE on t1000: it changes arrival accuracy, not speed. The firmware\n');
fprintf('commands a duration, so a mismatched t1000 under/over-runs every segment\n');
fprintf('and the executor spends time on encoder corrections. Compare worst_dev\n');
fprintf('and the seconds column together, not seconds alone.\n');
fprintf('%s\n\n', repmat('=', 1, 96));

if isfield(opts, 'csv') && ~isempty(opts.csv)
    write_csv(rows, opts.csv);
    fprintf('CSV written: %s\n', opts.csv);
end
end

% ----------------------------------------------------------------------
function o = setdefault(o, f, v)
if ~isfield(o, f) || isempty(o.(f))
    o.(f) = v;
end
end

function v = variant(label, speed_scale, t1000, turn_cost, lane_cost)
% NOTE the field name: SIM_REPORT_EXEC reads `time_1000mm_ms`, not `t1000`.
% Getting this wrong does not error - the unknown key is ignored and the sweep
% silently measures the baseline over and over (that happened once here, and
% made t1000 look like a knob with no effect). The label still says t1000
% because that is the on-car command name.
v = struct('label', label, 'speed_scale', speed_scale, ...
           'time_1000mm_ms', t1000, 'turn_cost', turn_cost, 'lane_cost', lane_cost);
end

function v = valid_only(rows)
ok = false(1, numel(rows));
for i = 1:numel(rows)
    ok(i) = rows(i).legs_ok == rows(i).n_legs && rows(i).legal;
end
v = rows(ok);
end

function write_csv(rows, path)
fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    error('sim_scan:csv', 'cannot write %s', path);
end
fprintf(fid, 'variant,speed_scale,t1000,turn_cost,lane_cost,seconds,turns,');
fprintf(fid, 'legs_ok,n_legs,worst_dev_mm,min_clearance_mm,min_band_mm,legal\n');
for i = 1:numel(rows)
    r = rows(i);
    fprintf(fid, '"%s",%.3f,%.1f,%d,%d,%.3f,%d,%d,%d,%.2f,%.2f,%.2f,%d\n', ...
        strtrim(r.label), r.speed_scale, r.t1000, r.turn_cost, r.lane_cost, ...
        r.seconds, r.turns, r.legs_ok, r.n_legs, r.worst_dev_mm, ...
        r.min_clearance_mm, r.min_band_mm, r.legal);
end
fclose(fid);
end
