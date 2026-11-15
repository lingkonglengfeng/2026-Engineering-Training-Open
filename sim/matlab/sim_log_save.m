function sim_log_save(log, path)
%SIM_LOG_SAVE Write a SIM_LOG to a UTF-8 text file (for diffing against captures).
%
%   sim_log_save(log, 'tmp\sim_goto_raw.log')

fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    error('sim_log_save:open', 'cannot write %s', path);
end
for i = 1:numel(log.lines)
    fprintf(fid, '%s\n', log.lines{i});
end
fclose(fid);
end
