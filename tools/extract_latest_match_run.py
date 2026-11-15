"""Extract only the last launch-to-end run from a legacy serial session file.

New PC releases write runs/latest.json directly; use that index first.
This migration tool seeks backwards in64KiB chunks, never scans older runs
after finding the last LEAVING marker, and leaves the original file untouched.
"""
import argparse
import json
from pathlib import Path
import re

START=re.compile(rb'^.*\bRX .*\[MATCH\] LEAVING\b.*$',re.M)

def latest_slice(path):
    with Path(path).open('rb') as stream:
        stream.seek(0,2);end=stream.tell();cursor=end;tail=b''
        while cursor:
            length=min(65536,cursor);cursor-=length;stream.seek(cursor)
            tail=stream.read(length)+tail
            matches=list(START.finditer(tail))
            if matches:
                start=matches[-1].start()
                return tail[start:],cursor+start,end
    raise ValueError('No MATCH LEAVING marker found')

def extract(path,folder=None):
    path=Path(path);data,offset,source_end=latest_slice(path)
    lines=data.decode('utf-8',errors='replace').splitlines()
    outcome='RUNNING';checkpoint=None;finished_time=None
    for i,line in enumerate(lines):
        terminal=re.search(r'\bRX .*\[MATCH\] (FINISHED|FAULT|IDLE)\s+(\S+)',line)
        if terminal:
            outcome,checkpoint=terminal.groups();finished_time=line.split(']',1)[0][1:]
        if outcome=='FINISHED' and re.search(r'\[MATCH STAGE\] '+re.escape(checkpoint)+r'\b',line):
            lines=lines[:i+1];break
        if outcome=='FAULT' and '[MATCH] flow stopped.' in line:
            lines=lines[:i+1];break
        if outcome=='IDLE':lines=lines[:i+1];break
    record=re.search(r'#(\d+)\s+RX',lines[0]);request=re.search(r'\brequest=(\d+)',lines[0])
    folder=Path(folder) if folder else path.parent/'runs';folder.mkdir(parents=True,exist_ok=True)
    output=folder/(path.stem+'_run_at_'+record.group(1)+'.log')
    output.write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')
    info=dict(log_file=output.name,path=str(output.resolve()),outcome=outcome,active=outcome=='RUNNING',
              start_wall_time=lines[0].split(']',1)[0][1:],end_wall_time=finished_time,
              first_record_id=int(record.group(1)),request=request.group(1) if request else None,
              source_log_file=path.name,source_start_offset=offset,source_end_offset=source_end,extracted=True)
    pending=folder/'latest.json.tmp';pending.write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
    pending.replace(folder/'latest.json')
    return info

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_file',type=Path);parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args();print(json.dumps(extract(args.session_file,args.output_dir),ensure_ascii=False,indent=2))
