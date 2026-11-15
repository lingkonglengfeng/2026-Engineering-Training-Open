function log = sim_log_add(log, fmt, varargin)
%SIM_LOG_ADD Append one `[SIM] `-tagged line to a SIM_LOG.
%
%   log = SIM_LOG_ADD(log, '[NAV] ARRIVED.')
%   log = SIM_LOG_ADD(log, '[NAV] pose now (%umm,%umm) cell=(%u,%u)', x, y, cx, cy)
%
% Returns the log (not the string) - use SIM_LOG_FMT for the text alone.

if ~log.enabled
    return
end
log.lines{end + 1} = ['[SIM] ' sim_log_fmt(fmt, varargin{:})];
end
