"""Bounded, asynchronous CAM2 obstacle evidence, associated by UART session/seq.

No serial/camera/control IO. Disk errors never change a motion command.
"""
from collections import OrderedDict
import copy
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import re
import threading
import time
import cv2

def fields(line):return dict(re.findall(r'(\w+)=([^\s;]+)',line))
def pair(value):
    try:
        values=[float(v) for v in value.split(',')]
        return values if len(values)==2 and all(math.isfinite(v) for v in values) else None
    except (AttributeError,ValueError):return None

class ObstacleEvidence:
    def __init__(self,folder=None,notify=None):
        self.folder=Path(folder) if folder is not None else None;self.notify=notify
        self.lock=threading.Lock();self.frames=OrderedDict();self.pending=OrderedDict();self.confirmed={};self.seen=set()
        self.latest=None;self.state={};self.jobs=queue.Queue(maxsize=8);self.thread=None
        self.saved=0;self.dropped=0;self.error=None;self.last_path=None;self.closed=False
    def offer(self,snapshot):
        if self.folder is None or self.closed:return
        with self.lock:
            if not snapshot or snapshot.get('operation')!='run' or snapshot.get('camera_role')!='qr':
                self.latest=None;self.frames.clear();return
            report=snapshot.get('obstacle_report') or {};key=tuple(report.get('frame_key') or ())
            if len(key)!=3:key=(snapshot.get('camera_epoch'),snapshot.get('frame'),snapshot.get('camera_read_finished_ns'))
            frame={'key':key,'raw':snapshot.get('raw_image'),'image':snapshot.get('image'),
                   'metadata':copy.deepcopy({k:snapshot.get(k) for k in ('camera_epoch','frame','camera_read_finished_ns','pc_frame_perf_s','pc_frame_wall_time','operation','camera_role','config','status','read_ms','encode_ms','send_ms','wait_ms','decode_ms','obstacle_report')})}
            self.latest=frame
            self.frames[key]=frame;self.frames.move_to_end(key)
            while len(self.frames)>8:self.frames.popitem(last=False)
    def sent(self,command,key):
        if self.folder is None:return
        tokens=command.split()
        if len(tokens)!=6 or tokens[0]!='vobs':return
        try:identity=(int(tokens[1]),int(tokens[2]))
        except ValueError:return
        with self.lock:
            self.pending[identity]={'command':command,'frame':self.frames.get(tuple(key or ())) }
            while len(self.pending)>8:self.pending.popitem(last=False)
    def receive(self,line,stamp,context):
        try:self._receive(line,stamp,context)
        except Exception as exc:
            with self.lock:self.error=str(exc)
    def _receive(self,line,stamp,context):
        if self.folder is None or self.closed:return
        if line.startswith('[VOBS STATE]'):
            with self.lock:self.state=fields(line)
            return
        if line.startswith('[VOBS EVICT]'):
            data=fields(line)
            try:key=(int(data['session']),int(data['obstacle']))
            except (KeyError,ValueError):return
            with self.lock:
                self.confirmed.pop(key,None)
                self.seen={item for item in self.seen if item[0]!=key}
            return
        is_map=line.startswith('[VOBS MAP]');is_ack=line.startswith('[VOBS ACK]')
        if not (is_ack or is_map):return
        data=fields(line)
        try:session=int(data['session']);seq=int(data.get('seq',0));obj=int(data.get('obstacle',0))
        except (KeyError,ValueError):return
        result=data.get('result');site=data.get('site','RAW')
        if is_ack and result not in ('WATCH','SLOW','DEFERRED','ACCEPTED','KEEP_NEARER','MERGED','LIMIT2'):return
        event='map' if is_map else 'limit' if result=='LIMIT2' else 'confirmed'
        record_key=(session,obj if obj else site)
        dedupe=(record_key,site,event)
        with self.lock:
            pending=self.pending.pop((session,seq),None) if is_ack else None
            previous=self.confirmed.get(record_key)
            if is_ack:
                candidate={'frame':pending.get('frame') if pending else None,'command':pending.get('command') if pending else None,
                           'ack':data,'line':line}
                if obj:self.confirmed[record_key]=candidate
            else:candidate=previous or {}
            if dedupe in self.seen:return
            # ACK evidence uses the exact source frame attached before TX.
            # MAP evidence uses a current camera view plus the original ACK.
            exact=bool(is_ack and candidate.get('frame'))
            frame=candidate.get('frame') if exact else self.latest
            if frame is None:
                self.error='没有可用的CAM2图像；未伪造截图';return
            if self.jobs.full():self.dropped+=1;return
            metadata={'schema':1,'event':event,'controller':copy.deepcopy(data),'controller_line':line,
                      'controller_state':copy.deepcopy(self.state),'serial':copy.deepcopy(context),
                      'received':dict(stamp),'tx_command':candidate.get('command'),
                      'confirmation':copy.deepcopy({k:v for k,v in candidate.items() if k!='frame'}),
                      'image':frame['metadata'],'exact_observation_frame':exact,
                      'image_kind':'observation_matched_to_TX' if exact else 'current_camera_view',
                      'car_current_mm':pair(data.get('car')),'car_at_frame_mm':pair(data.get('frame_car') or candidate.get('ack',{}).get('frame_car')),
                      'obstacle_estimate_mm':pair(data.get('raw') or candidate.get('ack',{}).get('raw')),
                      'obstacle_plan_mm':pair(data.get('plan')) or pair(data.get('x','')+','+data.get('y',''))}
            self.seen.add(dedupe)
            # Bound archived bookkeeping too, over long desktop sessions.
            if len(self.seen)>128:self.seen={dedupe}
            if len(self.confirmed)>16:self.confirmed={record_key:candidate}
            self.jobs.put_nowait((frame,metadata))
            if self.thread is None or not self.thread.is_alive():
                self.thread=threading.Thread(target=self._writer,name='obstacle-evidence',daemon=True);self.thread.start()
    def _writer(self):
        while True:
            job=self.jobs.get()
            try:
                if job is None:return
                self._save(*job)
            except Exception as exc:
                with self.lock:self.error=str(exc)
                if self.notify:self.notify('障碍自动存证失败：'+str(exc))
            finally:self.jobs.task_done()
    def _save(self,frame,metadata):
        ctx=metadata['serial'];run=Path(ctx.get('run_log_path') or ctx.get('session') or 'manual').stem
        folder=self.folder/run;folder.mkdir(parents=True,exist_ok=True)
        data=metadata['controller'];stem=datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+metadata['event']+'_obj'+data.get('obstacle','0')+'_'+data.get('site','RAW')
        raw=frame['raw'] if frame['raw'] is not None else frame['image']
        if raw is None:raise ValueError('CAM2图像为空')
        overlay=(frame['image'] if frame['image'] is not None else raw).copy()
        label='obj%s %s car=%s yaw=%s plan=%s raw=%s'%(data.get('obstacle','?'),data.get('site','?'),data.get('car','?'),data.get('yaw','?'),data.get('plan',str(metadata['obstacle_plan_mm'])),data.get('raw','?'))
        cv2.putText(overlay,label,(8,max(18,overlay.shape[0]-12)),cv2.FONT_HERSHEY_SIMPLEX,.36,(0,230,255),1,cv2.LINE_AA)
        for suffix,img in (('_raw.png',raw),('_overlay.png',overlay)):
            ok,encoded=cv2.imencode('.png',img)
            if not ok:raise ValueError('PNG编码失败')
            encoded.tofile(str(folder/(stem+suffix)))
        metadata['saved_wall_time']=datetime.now().astimezone().isoformat(timespec='milliseconds')
        path=folder/(stem+'.json');path.write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
        with self.lock:self.saved+=1;self.last_path=str(path);self.error=None
        if self.notify:self.notify('障碍自动存证已保存：'+str(path))
    def status(self):
        with self.lock:return dict(saved=self.saved,dropped=self.dropped,error=self.error,last_path=self.last_path,enabled=self.folder is not None)
    def close(self):
        self.closed=True
        if self.thread and self.thread.is_alive():
            try:self.jobs.put_nowait(None)
            except queue.Full:return
            self.thread.join(timeout=1)
