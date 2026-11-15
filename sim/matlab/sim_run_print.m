function sim_run_print(log)
%SIM_RUN_PRINT Echo a SIM_LOG to stdout, exactly as captured.
%
%   sim_run_print(log)
%
% Verbatim echo - no re-wrapping, no filtering - so the text can be diffed
% against a real serial capture with an ordinary text tool.

if ~log.enabled
    return
end
for i = 1:numel(log.lines)
    fprintf('%s\n', log.lines{i});
end
end
