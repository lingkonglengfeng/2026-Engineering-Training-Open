"""Controller RX/TX logging on the same PC clock as camera snapshots."""
import codecs
import collections
import copy
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk,messagebox
from obstacle_link import ObstacleLink
from obstacle_map import ObstacleMapState
from match_timing import MatchTiming
from arm_control import ArmState, reply_line


COMMON_COMMANDS=(('准备（归位 + 零漂，不定起点）','&prepare'),
                 ('1号区发车 / 继续','&OK 1'),('2号区发车 / 继续','&OK 2'),('流程状态','&match status'),('相机通信状态','&camstat'),
                 ('机械臂状态','&arm status'),('物料抓取状态','&arm material status'),
                 ('扫码状态','&qrscan status'),('视觉纠偏状态','&valign status'),
                 ('障碍联动状态','vobs status'),('工位地图读取','fieldmap status'),('工位地图保存','fieldmap save'),('工位地图读回','fieldmap load'),('底盘参数读取','motion status'),('底盘参数保存','motion save'),('底盘参数读回','motion load'),
                 ('复位比赛流程','&match reset'),('停止当前动作','&STOP'))


def command_packet(text):
    text=text.strip()
    if not text or any(ord(c)<32 or ord(c)>126 for c in text):raise ValueError('一次发送一条ASCII指令，不允许换行或控制字符')
    limit=63 if text.startswith('&motion') else 127 if text.startswith('motion ') else 31 if text.startswith('&') else 63
    if len(text)>limit:raise ValueError('指令超过固件文本长度上限')
    return text,(text+'\r\n').encode('ascii')


def timestamp():
    return {'wall_time':datetime.now().astimezone().isoformat(timespec='milliseconds'),
            'perf_s':time.perf_counter()}


TURN_SAMPLE_LIMIT=2000
TURN_FIELD=re.compile(r'([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)')


def turn_fields(line):
    """Unknown key order and trailing markers are ignored; keys carry the meaning."""
    return dict(TURN_FIELD.findall(line))


def turn_number(value):
    try:number=float(value)
    except (TypeError,ValueError):return None
    return number if math.isfinite(number) else None


def turn_gain_fields(line):
    values=turn_fields(line)
    gains=tuple(turn_number(values.get(key)) for key in ('kp','ki','kd'))
    return None if any(value is None for value in gains) else gains


class TurnTrace:
    """转向 PID 遥测解析结果。

    ``receive`` 只在 SerialLog._record 的 RX 分支里、持 ``SerialLog.lock`` 时调用；
    Tk 面板通过 ``SerialLog.turn_snapshot`` 在同一把锁下取副本，因此本类不另建锁。
    """
    def __init__(self):
        # 上限 2000 点：约 50 s @40 Hz，面板只画最近 10 s，留足回溯余量。
        self.samples=collections.deque(maxlen=TURN_SAMPLE_LIMIT)
        self.status=None;self.start=None;self.done=None
        self.pid=None;self.pid_note=None
        self.trace=None;self.reject=None
        self.active=False;self.supported=False
        self.sequence=0;self.received=None

    def clear(self):
        self.samples.clear()

    def receive(self,text,stamp):
        line=reply_line(text)
        if line.startswith('[TURNW]'):
            # 逐点波形：约 40 Hz，键顺序会变，行尾还可能有别的标记。
            values=turn_fields(line)
            sample=tuple(turn_number(values.get(key)) for key in ('t','tgt','act','err','out'))
            if any(value is None for value in sample):return
            self.supported=True;self.received=stamp
            self.samples.append((stamp,)+sample)
        elif line.startswith('[TURN] start'):
            values=turn_fields(line)
            self.supported=True;self.received=stamp;self.active=True
            # 新一次转向从空波形开始：面板“收到 start 自动清空”由这里统一实现，
            # 避免面板还要记录上一次的样本下标。
            self.samples.clear()
            self.start=dict(tgt=turn_number(values.get('tgt')),act=turn_number(values.get('act')),
                            profile=values.get('profile'),raw=line,received=stamp)
        elif line.startswith('[TURN] done'):
            match=re.match(r'^\[TURN\]\s+done\s+(\d+(?:\.\d+)?)\s*ms\b(.*)$',line)
            if match is None:return
            rest=match.group(2);values=turn_fields(rest)
            # FAST/FB/ST/OVER-3s 等是位置不固定的收尾标记，不能当成字段解析。
            flags=[token for token in rest.split() if '=' not in token]
            self.supported=True;self.received=stamp;self.active=False
            self.done=dict(ms=float(match.group(1)),target=turn_number(values.get('target')),
                           err=turn_number(values.get('err')),overshoot=turn_number(values.get('overshoot')),
                           peak_o=turn_number(values.get('peak_o')),flags=flags,raw=line,received=stamp)
        elif line.startswith('[TURN] trace'):
            match=re.match(r'^\[TURN\]\s+trace\s+(on|off)\b(.*)$',line,re.IGNORECASE)
            if match is None:return
            values=turn_fields(match.group(2))
            period=None;raw_period=values.get('period')
            if raw_period is not None:
                period_match=re.match(r'^(\d+(?:\.\d+)?)ms$',raw_period,re.IGNORECASE)
                period=float(period_match.group(1)) if period_match else turn_number(raw_period)
            self.supported=True;self.received=stamp
            self.trace=dict(on=match.group(1).lower()=='on',period_ms=period,raw=line,received=stamp)
        elif line.startswith('[TURN] status'):
            values=turn_fields(line)
            if not values:return
            status={}
            for key,text in values.items():
                number=turn_number(text)
                status[key]=number if number is not None else text
            status['raw']=line;status['received']=stamp
            self.supported=True;self.received=stamp
            if 'active' in status:self.active=status['active']==1
            self.status=status
        elif line.startswith('[TURN]'):
            # 任何 [TURN] 行都证明固件带转向遥测；rejected 说明保护条件未满足。
            self.supported=True;self.received=stamp
            if 'rejected' in line:
                self.active=False
                self.reject=dict(reason=line.split('rejected',1)[1].strip(' :'),raw=line,received=stamp)
        elif line.startswith('[PID]'):
            self.supported=True;self.received=stamp
            self.pid_note=dict(text=line,received=stamp)
            gains=turn_gain_fields(line)
            if gains is not None:
                # V236：主控回报当前 PID 来源 DEFAULT/W25Q64/MANUAL，
                # 用来解释“我调过参数为什么运行还是原来的”。
                source=turn_fields(line).get('source')
                self.pid=dict(kp=gains[0],ki=gains[1],kd=gains[2],source=source,
                              raw=line,received=stamp)

    def snapshot(self):
        return {'samples':list(self.samples),'status':copy.deepcopy(self.status),
                'start':copy.deepcopy(self.start),'done':copy.deepcopy(self.done),
                'pid':copy.deepcopy(self.pid),'pid_note':copy.deepcopy(self.pid_note),
                'trace':copy.deepcopy(self.trace),'reject':copy.deepcopy(self.reject),
                'active':self.active,'supported':self.supported,'sequence':self.sequence,
                'received':self.received}


class SerialLog:
    def __init__(self,folder,serial_factory=None):
        self.folder=Path(folder);self.serial_factory=serial_factory
        self.lock=threading.RLock();self.stop_event=threading.Event();self.thread=None
        self.connected=False;self.port='';self.baud=115200;self.path=None;self.session=None
        self.error=None
        self.history=collections.deque(maxlen=2000);self.events=collections.deque(maxlen=1000)
        self._file=None;self._sequence=0
        self._idle_path=None;self._run_info=None;self._run_number=0
        self.last_run_path=None;self._run_finish_due=None
        self.run_index_error=None;self._index_retry=None
        self.outbound=queue.Queue(maxsize=16)
        self.obstacles=ObstacleLink()
        self.manual_map_mode=False
        self.obstacle_map=ObstacleMapState()
        self.match_timing=MatchTiming()
        self.obstacle_evidence=None
        self.arm=ArmState()
        self.turn=TurnTrace()

    def _record(self,kind,text,stamp=None,partial=False):
        record=dict(stamp or timestamp(),kind=kind,text=text,partial=partial)
        with self.lock:
            self._sequence+=1;record['record_id']=self._sequence
            line=reply_line(text) if kind=='RX' and not partial else ''
            start=re.match(r'^\[MATCH\] LEAVING\b.*\brequest=(\d+)',line)
            if self._file is not None and start:
                self._begin_run(record,start.group(1))
            elif self._run_info and line.startswith('[ARM] CAMERA_DEPLOY_V'):
                self._finish_run('CONTROLLER_RESTART',record)
            self.history.append(record);self.events.append(record)
            if kind=='RX' and not partial:
                self.obstacles.receive(text,record['perf_s'])
                if line.startswith('[VOBS STATE]') and self.obstacles.state is not None and 'manual' in self.obstacles.state:
                    self.manual_map_mode=self.obstacles.state['manual']
                elif line.startswith('[FIELD MAP BEGIN]'):
                    self.manual_map_mode=bool(re.search(r'\bmanual_mode=1\b',line))
                self.obstacle_map.receive(line)
                self.match_timing.receive(line,record['perf_s'])
                # A bare ZDT> prompt can predate the command that completes
                # its line. UI ACKs use line-completion time; raw logging and
                # screenshot association retain their original first-byte time.
                self.arm.receive(text,time.perf_counter())
                # 转向遥测与机械臂状态同一时刻喂入；波形横轴用 PC 收行时间。
                self.turn.receive(text,record['perf_s'])
                if self.obstacle_evidence is not None:
                    self.obstacle_evidence.receive(line,record,{'session':self.session,
                        'run_log_path':str(self.last_run_path) if self.last_run_path else None,
                        'serial_log_path':str(self.path) if self.path else None,
                        'firmware':copy.deepcopy(self.arm.data.get('version',{}))})
            if self._file is not None:
                self._file.write('[%s] [pc=%.6f] #%d %s %s\n'%(
                    record['wall_time'],record['perf_s'],record['record_id'],kind,text))
            if self._run_info:
                self._run_info['last_record_id']=record['record_id']
                self._run_info['last_wall_time']=record['wall_time']
                terminal=re.match(r'^\[MATCH\] (FINISHED|FAULT|IDLE)\s+(\S+)',line)
                if terminal:
                    self._run_info.update(outcome=terminal.group(1),end_wall_time=record['wall_time'],
                                          end_perf_s=record['perf_s'],checkpoint=terminal.group(2))
                    self._run_finish_due=time.perf_counter()+.75
                    if terminal.group(1)=='IDLE':self._finish_run('RESET',record)
                elif line.startswith(('[MATCH TIME]','[MATCH STAGE]')):
                    self._run_finish_due=time.perf_counter()+.75
                    last=re.match(r'^\[MATCH STAGE\] (\S+)',line)
                    if last and self._run_info.get('outcome')=='FINISHED' and last.group(1)==self._run_info.get('checkpoint'):
                        self._finish_run('FINISHED',record)
                elif line.startswith('[MATCH] flow stopped.'):
                    self._finish_run(self._run_info.get('outcome','FAULT'),record)
                elif kind=='ERROR' or (kind=='STATUS' and text=='串口接收已结束'):
                    outcome=self._run_info['outcome']
                    self._finish_run(outcome if outcome!='RUNNING' else ('ERROR' if kind=='ERROR' else 'DISCONNECTED'),record)
        return record

    def _run_index(self,info=None):
        folder=self.folder/'runs';folder.mkdir(parents=True,exist_ok=True)
        pending=folder/'latest.json.tmp'
        self._index_retry=dict(info if info is not None else self._run_info)
        try:
            pending.write_text(json.dumps(self._index_retry,ensure_ascii=False,indent=2),encoding='utf-8')
            pending.replace(folder/'latest.json')
        except OSError as exc:
            # A Windows reader can briefly lock latest.json. Its index must
            # not interrupt the serial owner or discard the actual run log.
            self.run_index_error=str(exc)
        else:self.run_index_error=None;self._index_retry=None

    def _begin_run(self,record,request):
        if self._run_info and self._run_info['request']==request and self._run_info['outcome']=='RUNNING':return
        if self._run_info:self._finish_run(self._run_info['outcome'] if self._run_info['outcome']!='RUNNING' else 'RESTARTED',record)
        self._file.close();self._run_number+=1
        folder=self.folder/'runs';folder.mkdir(parents=True,exist_ok=True)
        self.path=folder/('%s_run%03d.log'%(self.session,self._run_number))
        self.last_run_path=self.path;self._file=self.path.open('w',encoding='utf-8',newline='\n')
        self._run_finish_due=None
        self._run_info=dict(log_file=self.path.name,path=str(self.path.resolve()),port=self.port,baud=self.baud,
                            session=self.session,request=request,outcome='RUNNING',active=True,
                            start_wall_time=record['wall_time'],start_perf_s=record['perf_s'],
                            first_record_id=record['record_id'])
        self._run_index()

    def _finish_run(self,outcome,record=None):
        if self._run_info is None:return
        self._file.close()
        self._run_info.update(outcome=outcome,active=False)
        if record and 'end_wall_time' not in self._run_info:
            self._run_info.update(end_wall_time=record['wall_time'],end_perf_s=record['perf_s'])
        self._run_index();self._run_info=None;self._run_finish_due=None
        self.path=self._idle_path
        self._file=self.path.open('a',encoding='utf-8',newline='\n')

    def _flush(self):
        if self._file is not None:self._file.flush()
        # Older firmware may omit stage statistics. Keep its terminal tail
        # briefly, then close this run without sending any controller command.
        if self._run_info and self._run_finish_due is not None and time.perf_counter()>=self._run_finish_due:
            self._finish_run(self._run_info['outcome'])
        if self._index_retry is not None:self._run_index(self._index_retry)

    def start(self,port,baud):
        if self.thread is not None and self.thread.is_alive():raise RuntimeError('串口仍在连接或接收中')
        if not port:raise ValueError('请选择主控调试串口')
        if baud not in (9600,19200,38400,57600,115200,230400,460800,921600):raise ValueError('请选择支持的波特率')
        self.stop_event=threading.Event();self.port=port;self.baud=baud
        self.error=None
        self.outbound=queue.Queue(maxsize=16)
        with self.lock:
            self.obstacles.connected()
            self.manual_map_mode=False
            self.arm=ArmState()
            self.turn=TurnTrace()
            self.history.clear();self.events.clear();self.path=self.session=None;self._sequence=0
            self._idle_path=self.last_run_path=self._run_info=self._run_finish_due=None;self._run_number=0
            self.run_index_error=self._index_retry=None
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()

    def stop(self):
        with self.lock:self.obstacles.disconnected()
        self.stop_event.set()

    def update_obstacle(self,snapshot):
        with self.lock:self.obstacles.submit(snapshot if self.connected and not self.stop_event.is_set() else None)

    def enable_obstacle_evidence(self,folder):
        from obstacle_evidence import ObstacleEvidence
        if self.obstacle_evidence is None:
            self.obstacle_evidence=ObstacleEvidence(folder,lambda message:self._record('STATUS',message))
        return self.obstacle_evidence

    def obstacle_status(self):
        with self.lock:
            status=self.obstacles.summary()
            if self.obstacle_evidence is not None:status['evidence']=self.obstacle_evidence.status()
            return status

    def arm_snapshot(self):
        with self.lock:return self.arm.snapshot()

    def manual_obstacle_mode(self):
        with self.lock:return self.manual_map_mode

    def obstacle_map_snapshot(self):
        with self.lock:return self.obstacle_map.snapshot()

    def match_timing_snapshot(self):
        with self.lock:return self.match_timing.snapshot()

    def turn_snapshot(self):
        # 面板只读快照，不触碰 events 队列（SerialPanel 独占 drain）。
        with self.lock:return self.turn.snapshot()

    def turn_clear(self):
        with self.lock:self.turn.clear()

    def send(self,text):
        text,packet=command_packet(text)
        normalized=text.lower().lstrip('&').strip()
        with self.lock:
            if not self.connected or self.stop_event.is_set():raise RuntimeError('请先接入主控串口')
            try:self.outbound.put_nowait((text,packet))
            except queue.Full:raise RuntimeError('指令发送队列已满，请稍后再试')
            if normalized in ('stop','arm stop','match reset','disable','dis','off'):
                self.arm.manual_cancel(text)
            if normalized in ('stop','dis','off','disable','match reset','vobs off') or re.match(r'^prepare(?:$|\s|[12]$)',normalized):
                reason='手工停止/复位/准备：清除旧障碍观测，等待主控新会话'
                self.obstacles.cancel(reason)
                self._record('STATUS',reason)

    def _run(self):
        device=None;pending='';first=None
        decoder=codecs.getincrementaldecoder('utf-8')(errors='replace')
        try:
            if self.serial_factory is None:
                import serial
                device=serial.Serial(port=None,baudrate=self.baud,timeout=.05,write_timeout=.05,
                                     xonxoff=False,rtscts=False,dsrdtr=False)
            else:device=self.serial_factory()
            device.dtr=False;device.rts=False;device.port=self.port;device.open()
            self.folder.mkdir(parents=True,exist_ok=True)
            session=datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+re.sub(r'[^A-Za-z0-9_-]','_',self.port)
            with self.lock:
                self.session=session;self.path=self.folder/(session+'_idle.log');self._idle_path=self.path
                self._file=self.path.open('w',encoding='utf-8',newline='\n');self.connected=True
            self._record('STATUS','串口已连接 %s %d；障碍联动自动跟随主控比赛状态'%(self.port,self.baud))
            next_flush=0.
            while not self.stop_event.is_set():
                for _ in range(4):
                    if self.stop_event.is_set():break
                    try:command,packet=self.outbound.get_nowait()
                    except queue.Empty:break
                    offset=0
                    while offset<len(packet):
                        written=device.write(packet[offset:])
                        if not isinstance(written,int) or not 0<written<=len(packet)-offset:
                            raise OSError('指令发送不完整，未自动重发')
                        offset+=written
                    self._record('TX',command)
                # A latest-only automatic observation is independent of the
                # manual command queue. Validate and calculate age HERE, just
                # before UART write, rather than when the GUI published it.
                with self.lock:
                    if self.connected and not self.stop_event.is_set():
                        command=self.obstacles.observation_command(time.perf_counter())
                        if command is not None:
                            if self.obstacle_evidence is not None:self.obstacle_evidence.sent(command,self.obstacles.last_sent_key)
                            _,packet=command_packet(command)
                            offset=0
                            while offset<len(packet):
                                written=device.write(packet[offset:])
                                if not isinstance(written,int) or not 0<written<=len(packet)-offset:
                                    raise OSError('障碍指令发送不完整，未自动重发')
                                offset+=written
                            self._record('TX',command)
                data=device.read(min(4096,max(1,device.in_waiting)))
                if data:
                    stamp=timestamp()
                    if first is None:first=stamp
                    pending+=decoder.decode(data)
                    while '\n' in pending:
                        line,pending=pending.split('\n',1)
                        self._record('RX',line.rstrip('\r'),first)
                        first=stamp if pending else None
                    if len(pending)>8192:
                        self._record('RX',pending,first,partial=True);pending='';first=None
                if time.perf_counter()>=next_flush:
                    with self.lock:
                        self._flush()
                    next_flush=time.perf_counter()+.25
        except Exception as exc:
            self.error=str(exc)
            self._record('ERROR','串口连接/接收失败：%s；确认COM口，并关闭占用该口的其它串口工具。'%exc)
        finally:
            pending+=decoder.decode(b'',final=True)
            if pending:self._record('RX',pending.rstrip('\r'),first,partial=True)
            with self.lock:self.connected=False;self.obstacles.disconnected()
            self._record('STATUS','串口接收已结束')
            with self.lock:
                if self._file is not None:self._file.close();self._file=None
            if device is not None:device.close()

    def drain(self):
        with self.lock:
            records=list(self.events);self.events.clear()
        return records

    def mark_snapshot(self,camera,frame,frame_perf_s,image_path,stamp):
        with self.lock:
            preceding=[r for r in self.history if r['kind']=='RX' and r['perf_s']<=frame_perf_s]
            match=next((r for r in reversed(preceding) if '[MATCH]' in r['text']),None)
            stage=next((r for r in reversed(preceding) if re.search(r'\[MATCH\]\s+(MOVING|LEAVING|WAIT_START_OK|FAULT|FINISHED)\b',r['text'])),None)
            request=next((r for r in reversed(preceding) if '['+camera+'] request' in r['text']),None)
            last_tx=next((r for r in reversed(self.history) if r['kind']=='TX' and r['perf_s']<=frame_perf_s),None)
            nearby=[dict(r) for r in self.history if r['kind'] in ('RX','TX') and frame_perf_s-2<=r['perf_s']<=stamp['perf_s']][-100:]
            marker=self._record('SNAPSHOT',json.dumps({'camera':camera,'frame':frame,
                'frame_pc_perf_s':frame_perf_s,'image_path':str(image_path)},ensure_ascii=False),stamp)
            if self._file is not None:self._file.flush()
            return {'connected':self.connected,'port':self.port,'baud':self.baud,'session':self.session,
                    'log_path':str(self.path) if self.path is not None else None,
                    'run_log_path':str(self.last_run_path) if self.last_run_path is not None else None,
                    'marker_record_id':marker['record_id'],'frame_pc_perf_s':frame_perf_s,
                    'last_match_before_frame':dict(match) if match else None,
                    'last_stage_before_frame':dict(stage) if stage else None,
                    'last_tx_before_frame':dict(last_tx) if last_tx else None,
                    'last_camera_request_before_frame':dict(request) if request else None,
                    'nearby_serial':nearby}


def scan_ports(result_queue):
    try:
        from serial.tools.list_ports import comports
        result_queue.put(([(p.device,p.description) for p in sorted(comports())],None))
    except Exception as exc:result_queue.put(([],str(exc)))


class SerialPanel:
    def __init__(self,parent,logger,settings_path,refresh=True):
        self.parent,self.logger,self.settings_path=parent,logger,Path(settings_path)
        self.closed=False;self._after=None;self._ports_queue=None
        try:config=json.loads(self.settings_path.read_text(encoding='utf-8'))
        except (OSError,ValueError):config={}
        self.port=tk.StringVar(value=config.get('port',''));self.baud=tk.StringVar(value=str(config.get('baud',115200)))
        self.status=tk.StringVar(value='串口未连接');self.path_text=tk.StringVar(value='每次比赛启动到结束单独保存；不自动打开COM口。')
        self.summary=tk.StringVar(value='未连接')
        bar=ttk.Frame(parent,padding=12);bar.pack(fill='x')
        ttk.Label(bar,text='主控串口').pack(side='left')
        self.port_combo=ttk.Combobox(bar,textvariable=self.port,width=12);self.port_combo.pack(side='left',padx=6)
        self.scan_button=ttk.Button(bar,text='刷新串口',command=self.refresh_ports);self.scan_button.pack(side='left',padx=4)
        self.baud_combo=ttk.Combobox(bar,textvariable=self.baud,values=(9600,19200,38400,57600,115200,230400,460800,921600),state='readonly',width=10)
        self.baud_combo.pack(side='left',padx=6)
        self.connect_button=ttk.Button(bar,text='接入串口',command=self.connect);self.connect_button.pack(side='left',padx=4)
        self.disconnect_button=ttk.Button(bar,text='断开串口',command=logger.stop);self.disconnect_button.pack(side='left',padx=4)
        ttk.Button(bar,text='打开日志目录',command=self.open_folder).pack(side='left',padx=4)
        commands=ttk.Frame(parent,padding=(12,4));commands.pack(fill='x')
        ttk.Label(commands,text='常用指令').pack(side='left')
        self.command=tk.StringVar(value=COMMON_COMMANDS[0][0])
        self.command_combo=ttk.Combobox(commands,textvariable=self.command,values=[label for label,_ in COMMON_COMMANDS],width=28)
        self.command_combo.pack(side='left',padx=6)
        self.command_hint=tk.StringVar(value=COMMON_COMMANDS[0][1])
        self.command_combo.bind('<<ComboboxSelected>>',lambda event:self.command_hint.set(dict(COMMON_COMMANDS).get(self.command.get(),self.command.get())))
        self.send_button=ttk.Button(commands,text='发送',command=self.send,state='disabled');self.send_button.pack(side='left',padx=4)
        ttk.Label(commands,textvariable=self.command_hint).pack(side='left',padx=8)
        ttk.Label(parent,text='选择指令不会自动发送；请点击“发送”。比赛流程：先双路接入运行，再准备（只归位/去零漂）；READY后选1/2号发车才确定当前位置。',padding=(12,4)).pack(fill='x')
        launch = ttk.Frame(parent, padding=(12, 3)); launch.pack(fill='x')
        self.launch_buttons = []
        for title, command in (('准备：只归位与零漂', '&prepare'), ('1号启停区发车', '&OK 1'), ('2号启停区发车', '&OK 2')):
            button = ttk.Button(launch, text=title, command=lambda c=command: self.send_named(c), state='disabled')
            button.pack(side='left', padx=4); self.launch_buttons.append(button)
        ttk.Label(parent, text='车上PD1：短按松开=1号区，长按满1秒松开=2号区；按下必须晚于READY。', padding=(12, 2)).pack(fill='x')
        ttk.Label(parent,textvariable=self.status,padding=(12,4)).pack(fill='x')
        ttk.Label(parent,textvariable=self.path_text,wraplength=1050,padding=(12,4)).pack(fill='x')
        frame=ttk.Frame(parent,padding=12);frame.pack(fill='both',expand=True)
        self.text=tk.Text(frame,font=('Consolas',10),wrap='none',state='disabled')
        scroll=ttk.Scrollbar(frame,command=self.text.yview);self.text.configure(yscrollcommand=scroll.set)
        horizontal=ttk.Scrollbar(frame,orient='horizontal',command=self.text.xview)
        self.text.configure(xscrollcommand=horizontal.set);horizontal.pack(side='bottom',fill='x')
        scroll.pack(side='right',fill='y');self.text.pack(side='left',fill='both',expand=True)
        parent.bind('<Destroy>',self._destroyed,add='+')
        if refresh:self.refresh_ports()
        self._after=parent.after(80,self.poll)

    def refresh_ports(self):
        if self._ports_queue is not None:return
        self._ports_queue=queue.Queue();self.scan_button.configure(state='disabled')
        threading.Thread(target=scan_ports,args=(self._ports_queue,),daemon=True).start()

    def connect(self):
        try:
            port=self.port.get().strip();baud=int(self.baud.get())
            if not port:raise ValueError('请选择主控日志COM口')
            self.settings_path.parent.mkdir(parents=True,exist_ok=True)
            self.settings_path.write_text(json.dumps({'port':port,'baud':baud},ensure_ascii=False,indent=2),encoding='utf-8')
            self.logger.start(port,baud)
        except (ValueError,OSError,RuntimeError) as exc:messagebox.showerror('串口配置',str(exc),parent=self.parent)

    def poll(self):
        self._after=None
        if self.closed:return
        if self._ports_queue is not None:
            try:ports,error=self._ports_queue.get_nowait()
            except queue.Empty:pass
            else:
                self._ports_queue=None;self.scan_button.configure(state='normal')
                self.port_combo.configure(values=[p[0] for p in ports])
                if not self.port.get() and len(ports)==1:self.port.set(ports[0][0])
                if error:self.status.set('串口枚举失败：'+error)
        records=self.logger.drain()
        if records:
            self.text.configure(state='normal')
            self.text.insert('end',''.join('[%s] %s %s\n'%(r['wall_time'],r['kind'],r['text']) for r in records))
            if int(self.text.index('end-1c').split('.')[0])>6000:self.text.delete('1.0','1000.0')
            self.text.see('end');self.text.configure(state='disabled')
            errors=[r['text'] for r in records if r['kind']=='ERROR']
            if errors:self.status.set(errors[-1])
        active=self.logger.thread is not None and self.logger.thread.is_alive()
        self.connect_button.configure(state='disabled' if active else 'normal')
        self.send_button.configure(state='normal' if self.logger.connected and not self.logger.stop_event.is_set() else 'disabled')
        self.disconnect_button.configure(state='normal' if active else 'disabled')
        for button in self.launch_buttons: button.configure(state='normal' if self.logger.connected and not self.logger.stop_event.is_set() else 'disabled')
        self.port_combo.configure(state='disabled' if active else 'normal')
        self.baud_combo.configure(state='disabled' if active else 'readonly')
        if self.logger.error:
            self.status.set('串口失败：'+self.logger.error+'；请先关闭占用该COM口的串口工具。');self.summary.set('异常')
        elif self.logger.connected:self.status.set(self.logger.port+' · 日志接收中');self.summary.set(self.logger.port+'已连接')
        elif active:self.status.set(self.logger.port+' · 正在连接/断开');self.summary.set('连接/断开中')
        else:self.status.set('串口未连接');self.summary.set('未连接')
        if self.logger.last_run_path is not None:
            self.path_text.set('本轮日志：'+str(self.logger.last_run_path)+('（索引待更新）' if self.logger.run_index_error else ''))
        elif self.logger.path is not None:self.path_text.set('连接/手动调试日志：'+str(self.logger.path))
        self._after=self.parent.after(80,self.poll)

    def open_folder(self):
        folder=self.logger.folder/'runs'
        folder.mkdir(parents=True,exist_ok=True);os.startfile(folder)

    def send_named(self, command):
        try: self.logger.send(command)
        except (ValueError,RuntimeError) as exc: messagebox.showerror('串口发送',str(exc),parent=self.parent)

    def send(self):
        try:self.logger.send(dict(COMMON_COMMANDS).get(self.command.get(),self.command.get()))
        except (ValueError,RuntimeError) as exc:messagebox.showerror('串口发送',str(exc),parent=self.parent)

    def _destroyed(self,event):
        if event.widget is not self.parent:return
        self.closed=True;self.logger.stop()
        if self._after is not None:self.parent.after_cancel(self._after);self._after=None
