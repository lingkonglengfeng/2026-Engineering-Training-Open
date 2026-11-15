function log = sim_log(opts)
%SIM_LOG Firmware-format log sink for the simulator.
%
%   log = SIM_LOG()
%   log = SIM_LOG(struct('enabled', false))
%   log = SIM_LOG_ADD(log, sprintf('...'))    % returns the log, appends a line
%
% The strings written by the executor are copied VERBATIM from
% Core/Src/zdt_motor.c with the same field order and formatting, so simulated
% output can be placed next to a real serial capture. Every line carries the
% `[SIM]` tag, so a line WITHOUT that tag came from real hardware.
%
% Source lines (Core/Src/zdt_motor.c):
%   ~3126  [NAV] EXEC start.
%   ~3240  [NAV] align turn %+.1f deg (target %+.0f, fast)
%   ~3280  [NAV] applied rpm FL%+d FR%+d RL%+d RR%+d
%   ~2014  [NAV] drive %.3frev (%.0fmm) %s spd=%.3f v=%.0fmm/s t=%lums
%   ~2085  [NAV] strafe %.0fmm %s (no turn) spd=%.3f t=%lums
%   ~2210  [TURN] done %lums target=%+.2f err=%+.3f overshoot=%.3f peak_o=%.2f
%   ~3387  [NAV] waiting for wheels to settle
%   ~3408  [NAV] revs FL%+.3f FR%+.3f RL%+.3f RR%+.3f spread=%.3f
%   ~3500  [NAV] SEG%u done (strafe %.0fmm, no turn)
%   ~3508  [NAV] SEG%u done: %.3f/%.3frev (hold peak %.2f deg)
%   ~3555  [NAV] pose now (%umm,%umm) cell=(%u,%u) after SEG%u
%   ~3481  [NAV] heading drift %+.1f deg > %.0f deg limit
%   ~2929  [NAV] start zone %u is now NO-CONTACT
%   ~2963  [NAV] script done: pose=(%umm,%umm)
%   ~3100  [NAV] no executable path.
%   ~3105  [NAV] ARRIVED (already at target).
%   ~3565  [NAV] ARRIVED.
%
% NOT REPLICATED (stated rather than faked): the `&cross`/`&leave` script
% progress lines, the interleaving of unrelated [IMU]/[CHASSIS] task output, and
% the real-time jitter of the 1 ms tick. A simulation log is structurally
% identical, not byte-identical.

if nargin < 1 || isempty(opts)
    opts = struct();
end
log = struct();
log.lines = {};
log.enabled = ~isfield(opts, 'enabled') || opts.enabled;
end
