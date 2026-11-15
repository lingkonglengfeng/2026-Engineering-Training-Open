function s = sim_log_fmt(fmt, varargin)
%SIM_LOG_FMT Format a log line WITHOUT appending it.
%
%   sim_log_fmt('[NAV] ARRIVED.')
%   sim_log_fmt('[NAV] drive %.3frev (%.0fmm)', 3.9, 1000)
%
% Exists so a string can be both printed and stored/tested without going through
% SIM_LOG_ADD (which returns the log, not the text).

if isempty(varargin)
    s = fmt;
else
    s = sprintf(fmt, varargin{:});
end
end
