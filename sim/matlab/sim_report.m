function report = sim_report(opts)
%SIM_REPORT Batch run every task leg and report trajectories, timing, safety.
%
%   report = SIM_REPORT()
%   report = SIM_REPORT(struct('speed_scale', 0.75, 'csv', 'tmp\legs.csv'))
%
% This is the deliverable that answers "can the car actually drive the mission,
% and how close does it come to losing the run". For each leg it prints and
% records:
%
%   cells / segments / turns / strafes   the action plan
%   the action list                      what the car will be told to do
%   seconds                              simulated time for the leg
%   end position error                   can it hit the target mm?
%   minimum clearance to any forbidden rect (yellow zone, table top, QR board)
%   body-leaves-the-gray-lane verdict    the rule that ENDS the run
%
% The last two are the simulator's real product: on hardware you can only find
% out by driving and looking at the damage.

if nargin < 1 || isempty(opts)
    opts = struct();
end

spec = load_mapspec();
map  = build_grid(spec);
% Only pass real parameter overrides to SIM_PARAMS - it rejects unknown keys on
% purpose (a typo in a parameter name should not be silently ignored), so the
% report's own options (csv, quiet, ...) must be filtered out first.
over = struct();
known = {'speed_scale', 'time_1000mm_ms', 'counts_per_rev', 'position_mode'};
for i = 1:numel(known)
    if isfield(opts, known{i})
        over.(known{i}) = opts.(known{i});
    end
end
P = sim_params(over);
if isfield(opts, 'overrides') && ~isempty(opts.overrides)
    P = sim_params(opts.overrides);
end
log  = sim_log();

% the mission legs, in the order the run performs them
legs = { ...
    'h1',    'exit1'; ...
    'exit1', 'raw'; ...
    'raw',   'rough2'; ...
    'rough2','temp2'; ...
    'temp2', 'qr'; ...
    'qr',    'h2'; ...
    'h2',    'exit2'; ...
    'exit2', 'rough2'; ...
    'h2',    'h1' };

fprintf('\n%s\n', repmat('=', 1, 78));
fprintf('NAVIGATION REGRESSION REPORT  (spd=%.2f, t1000=%.0fms)\n', ...
    P.speed_scale, P.time_1000mm_ms);
fprintf('%s\n', repmat('=', 1, 78));

results = struct([]);
total_s = 0;
n_fail = 0;

for i = 1:size(legs, 1)
    a = legs{i, 1};
    b = legs{i, 2};
    pa = spec.waypoints_mm.(a);
    pb = spec.waypoints_mm.(b);

    % Start each leg from the zone's pose when the leg begins at a start zone,
    % otherwise from the previous stop point (the pose drives the nose binding).
    if a(1) == 'h'
        zone = str2double(a(2));
        % mapspec zone keys are x1/x2 (a JSON key cannot be a bare digit)
        car = car_init(pa(1), pa(2), spec.zones.(sprintf('x%d', zone)).nose_map);
    else
        car = car_init(pa(1), pa(2), [0 1]);
    end

    out = run_leg(map, car, a, b, P, struct('log', log));
    log = out.log;

    v = out.verdict;
    fprintf('\n%-8s -> %-7s  %2d cells, %d segs, %d turns, %d strafes\n', ...
        a, b, size(out.path, 1) - 1, out.n_segments, out.turns, out.strafes);
    fprintf('   actions : %s\n', strjoin(out.actions, ' | '));
    fprintf('   time    : %6.2f s\n', out.seconds);
    % Two distinct numbers. `goal offset` is NOT an error: start-zone centres are
    % illegal car centres, so the planner snaps them to the nearest legal cell.
    % `drive dev` is what the executor actually owes - it must stay within the
    % snapping scale (~25-50 mm), otherwise the car is not following its plan.
    fprintf('   snapped : %d   goal offset (not an error) %.1f mm   drive dev %.1f mm\n', ...
        out.snapped, out.snapped_off_mm, out.err_mm);
    fprintf('   clearance to forbidden rects: %s mm\n', num2str(round(v.min_clearance_mm), '%7.1f'));
    fprintf('   yellow %d  pad %d  off-field %d  locked-zone %d  OFF-GRAY-LANE %d\n', ...
        v.hit_blocked, v.hit_pad, v.off_field, v.zone_locked, v.off_gray);
    if ~v.ok
        fprintf('   *** ILLEGAL: body overlaps a forbidden area ***\n');
        n_fail = n_fail + 1;
    end

    total_s = total_s + out.seconds;
    r = struct('from', a, 'to', b, 'cells', size(out.path, 1) - 1, ...
               'segs', out.n_segments, 'turns', out.turns, 'strafes', out.strafes, ...
               'seconds', out.seconds, 'err_mm', out.err_mm, ...
               'clearance_mm', v.min_clearance_mm, 'off_gray', v.off_gray, ...
               'hit_blocked', v.hit_blocked, 'hit_pad', v.hit_pad, ...
               'off_field', v.off_field, 'actions', {out.actions}, ...
               'samples', out.samples, 'end_mm', out.end_mm);
    if isempty(results)
        results = r;
    else
        results(end + 1) = r;   %#ok<AGROW>
    end
end

fprintf('\n%s\n', repmat('-', 1, 78));
fprintf('TOTAL simulated driving time : %.2f s over %d legs\n', total_s, numel(results));
fprintf('Illegal-body legs            : %d\n', n_fail);
fprintf('Legs whose body leaves gray  : %d\n', sum([results.off_gray]));
fprintf('Tightest clearance anywhere  : %.1f mm\n', min([results.clearance_mm]));

report = struct();
report.legs = results;
report.total_seconds = total_s;
report.P = P;
report.spec = spec;
report.n_illegal = n_fail;

% --- regression assertions -------------------------------------------------
% These make the report usable as a test, not just a printout. They encode the
% properties the simulator must keep; see sim/docs/设计说明.md for the current
% baselines.
fprintf('\n%s\n', repmat('-', 1, 78));
fprintf('REGRESSION ASSERTIONS\n');
chk = @(ok, fmt, varargin) assert(ok, ['REGRESSION FAILED: ' fmt], varargin{:});

devs = [results.err_mm];
chk(all(devs < 60), ...
    'every leg must arrive within the half-cell snapping scale (60 mm); worst was %.1f mm', ...
    max(devs));
chk(n_fail == 0, 'no leg may drive the body into a forbidden area');
chk(all(~[results.off_field]), 'no leg may put the body off the mat');
fprintf('  [ ok ] all %d legs arrive within %.1f mm of their planned cell centre\n', ...
    numel(devs), max(devs));
fprintf('  [ ok ] no body-overlap with yellow zones / pads / off-mat\n');

if isfield(opts, 'csv') && ~isempty(opts.csv)
    write_csv(report, opts.csv);
    fprintf('CSV written: %s\n', opts.csv);
end
fprintf('%s\n\n', repmat('=', 1, 78));
end

function write_csv(report, path)
fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    error('sim_report:csv', 'cannot write %s', path);
end
fprintf(fid, 'from,to,cells,segs,turns,strafes,seconds,err_mm,clearance_mm,');
fprintf(fid, 'off_gray,hit_blocked,hit_pad,off_field,actions\n');
for i = 1:numel(report.legs)
    r = report.legs(i);
    fprintf(fid, '%s,%s,%d,%d,%d,%d,%.3f,%.2f,%.2f,%d,%d,%d,%d,"%s"\n', ...
        r.from, r.to, r.cells, r.segs, r.turns, r.strafes, r.seconds, ...
        r.err_mm, r.clearance_mm, r.off_gray, r.hit_blocked, r.hit_pad, ...
        r.off_field, strjoin(r.actions, ' | '));
end
fclose(fid);
end
