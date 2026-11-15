"""Chinese desktop control panel; all network/detection work stays off Tk's thread."""
import argparse
import collections
from datetime import datetime
import csv
import json
import logging
import os
from pathlib import Path
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
sys.path.insert(1,str(HERE.parent/'maixcam_service'))
from settings import read_config,motion_settings
from backend import PcBackend
from qr_backend import PcQrBackend,qr_service
from transport import ControlClient, VideoClient, StreamContextChanged
from discovery import scan_devices
from serial_monitor import SerialLog,SerialPanel,timestamp
from arm_panel import ArmPanel
from turn_panel import TurnPanel
from screen_preview import ScreenPreviewClient
import service

COLORS = ("红色", "黄色", "蓝色", "绿色", "黑色", "浅蓝色")
PREVIEW_MODES = {"原始画面":(0,0,0,0,0),"物料识别":(2,1,1,20,0),"空圆环":(3,2,2,20,1),
                 "带物料圆环":(3,3,2,20,3),"圆盘切点":(8,1,0,20,0),"颜色取样":(2,1,1,20,4)}
QR_PREVIEW_MODES={"原始画面":(0,0,0,0,0),"二维码识别":(6,4,0,20,0)}


def discover_in_background(results,known_hosts,cancel):
    try:results.put((scan_devices(known_hosts,cancel),None))
    except Exception as exc:results.put(([],str(exc)))


def set_app_icon(root):
    path=Path(__file__).resolve().parent/'assets/camera.png'
    if path.exists():
        root._vision_icon=tk.PhotoImage(master=root,file=str(path))
        root.iconphoto(True,root._vision_icon)


def color_text(report):
    regions=report.get('colors', [])
    model=report.get('blue_model') or {}
    if model.get('state')=='CHROMA_OVERLAP':
        return '蓝/浅蓝判色：两类取样色度重叠，暂不判定 | '
    if not regions:return ''
    counts=collections.Counter(region['color'] for region in regions)
    names={3:'蓝色',6:'浅蓝色',0:'不确定区域'}
    text='、'.join('%s×%d'%(names[key],counts[key]) for key in (3,6,0) if counts[key])
    return '区域判色：'+text+'（框为物体范围） | '


def task_text(config):
    names={0:'STOP等待任务',2:'物料识别',3:'圆环定位',6:'扫码',8:'圆盘定位'}
    stations={0:'待机',1:'原料/圆盘',2:'ROUGH粗加工',3:'TEMP暂存',4:'二维码'}
    return '当前任务：%s（模式%d） / 工位%d %s'%(names.get(config[0],'未知'),config[0],config[1],stations.get(config[1],''))


def return_text(snapshot):
    if snapshot.get('camera_role')=='qr':
        if snapshot.get('operation')=='preview':return '扫码预览：不回传；点击“接入运行并回传”跟随STM32扫码。'
        report=snapshot.get('return_report') or {};last=report.get('last')
        if snapshot['config'][0]==0:return '扫码回传：已接入，等待STM32 SET6；保留上一任务显示。'
        if last is not None:return '四组任务码已发往CAM2桥：累计 %d 组（主控接收以串口日志为准）。'%report['groups']
        return '扫码回传：等待连续3帧确认合法任务码。'
    if snapshot.get('operation', 'preview') == 'preview':
        return '坐标回传：未启用（独立预览）；点击“接入运行并回传”接入主控任务。'
    config=snapshot['config']
    if config[0] == 0:
        return '坐标回传：已接入，等待STM32下发任务；当前只显示画面。'
    if snapshot.get('calibration'):
        return '坐标回传：颜色取样中，不发送抓取坐标。'
    report=snapshot.get('return_report') or {}
    last=report.get('last')
    if last is None:
        return '坐标回传：已接入主控任务，等待本任务的观测结果。'
    age=report['age_ms']
    if age > 500:
        return '坐标回传：最近发送距今 %.0f ms，累计 %d 组。'%(age,report['groups'])
    if not last['valid']:
        return '坐标回传：本次目标无效，已发送无效观测；累计 %d 组。'%report['groups']
    identity='ID %d'%last['id']
    if last.get('type')==2:
        names=('自动','红色','黄色','蓝色','绿色','浅蓝色','黑色')
        identity='%s（协议ID%d）'%(names[last['id']],last['id'])
    return '坐标已发往相机桥：%s | X %+.1f / Y %+.1f px（右/下为正）| 累计 %d 组。'%(
        identity,last['dx_px'],last['dy_px'],report['groups'])


def motion_text(report):
    if not report:return '运动预判：未启用或未进入物料识别；仅观察，不下发动作。'
    states={'DISABLED':'未启用','CAMERA_MOVING':'检测到背景移动，请固定相机',
            'TARGET_UNCONFIRMED':'等待可靠顶面','INVALID_OBSERVATION':'观测无效',
            'STALE_OBSERVATION':'观测时间未更新','LEARNING_ORBIT':'收集圆周轨迹',
            'STOPPED':'近似静止或运动过小','ORBIT_MISMATCH':'轨道不匹配，重新学习',
            'PREDICTING':'预测通过观测校验','CHECKING_MODEL':'预测模型待验证','SURFACE_REQUIRED':'需要开启顶面分析'}
    text='运动预判：'+states.get(report['state'],report['state'])
    direction=report.get('direction')
    if direction in ('clockwise','counterclockwise'):
        text+=' | 图像'+('顺时针' if direction=='clockwise' else '逆时针')
        text+=' %.1f°/s'%report['omega_deg_s']
    elif report.get('velocity_px_s'):
        vx,vy=report['velocity_px_s']
        if vx*vx+vy*vy>9:text+=' | 方向：'+('右' if vx>2 else '左' if vx< -2 else '')+('下' if vy>2 else '上' if vy< -2 else '')
    if report.get('future') is not None:
        text+=' | 预测 +%.0fms'%report['horizon_ms']
        if report.get('eta_ms') is not None:text+=' | 到窗口约 %.0fms'%report['eta_ms']
    if report.get('error_p95_px') is not None:text+=' | 后续观测P95 %.1fpx（%d次）'%(report['error_p95_px'],report['validation_count'])
    if report.get('grasp_advice'):text+=' | 预计夹持时刻位于窗口（未执行）'
    return text


def obstacle_text(report,link=None):
    if not report:text='前方障碍：等待相机新图像'
    elif report.get('selected'):
        item=report['selected']
        text='前方圆柱：%s %d/3帧 | 宽%.0fpx | 车头净距约%.0fmm | 右向%+.0fmm'%(
            '已确认' if report['confirmed'] else '确认中',min(3,report['confirmed_frames']),
            item['width_px'],item['front_gap_mm'],item['body_right_mm'])
        if item.get('extrapolated'):text+=' | 超出20～30cm标定段，粗估'
        if item.get('side_clipped'):text+=' | 左右截断，不上报'
        if item['front_gap_mm']<50:text+=' | 太近，超出本次接收范围'
    else:text='前方障碍：未检测到符合特征的圆柱'
    if link:
        text+='\n联动：'+link['reason']
        if 'sent' in link:text+=' | 已发送%d帧'%link['sent']
    else:text+='\n等待CAM2桥接连接'
    return text


class VisionWorker:
    def __init__(self,config,config_path,mode):
        self.config,self.config_path,self.mode = config,config_path,mode
        self.camera_role=config.get('camera_role','material')
        self.stop_event = threading.Event()
        self.commands = queue.Queue(maxsize=32)
        self.lock = threading.Lock()
        self.latest = None
        self.events = collections.deque(maxlen=100)
        self.thread = None
        self.obstacle_sink = None
        self.wireless_last_frame = None
        self.wireless_sent = 0
        self.wireless_reason = '等待CAM2无线桥接'

    def log(self,text):
        text=('CAM2 · ' if self.camera_role=='qr' else 'CAM1 · ')+text
        with self.lock:self.events.append(time.strftime("%H:%M:%S ")+text)
        logging.info(text)

    def start(self):
        self.thread = threading.Thread(target=self.run,daemon=True)
        self.thread.start()

    def command(self,name,value=None):
        try:self.commands.put_nowait((name,value))
        except queue.Full:self.log("操作较多，请稍后再试")

    def snapshot(self):
        with self.lock:
            latest,events = self.latest,list(self.events)
            self.events.clear()
        return latest,events

    def stop(self):
        self.stop_event.set()
        if self.obstacle_sink is not None:self.obstacle_sink(None)

    def handle_commands(self,backend):
        for _ in range(32):
            try:name,value = self.commands.get_nowait()
            except queue.Empty:break
            try:
                if name == "select" and self.mode == "preview":
                    label,color = value
                    desired = (QR_PREVIEW_MODES if self.camera_role=='qr' else PREVIEW_MODES)[label]
                    if desired[0] == 2:
                        desired = desired[:2]+((0,1,2,3,4,6,5)[color+1],)+desired[3:]
                    backend.configure(desired)
                    self.log("预览切换："+label)
                elif name == "color":
                    if backend.config[0] == 2 and backend.config[4] & 4:
                        backend.cal_color,backend.points = value+1,[]
                    elif self.mode == "preview":
                        backend.configure((2,1,(0,1,2,3,4,6,5)[value+1],20,0))
                elif name == "sample":backend.collect_sample(*value)
                elif name == "clear_samples":backend.points=[]
                elif name == "save_samples":
                    backend.save_calibration()
                    self.log("颜色标定已保存到电脑")
                elif name == "finish_samples":
                    backend.save_calibration(done=True)
                    self.log("取样结束；主控联动时仍等待原STOP流程")
                elif name in ('surface_enabled','surface_all','ring_refinement','use_surface_output'):
                    setattr(backend,name,bool(value))
                    backend.surface_analyzer.reset()
                    backend.motion_predictor.reset();backend.background_motion.reset()
                    self.log('增强选项 %s=%s'%(name,bool(value)))
                elif name == 'screen_preview':
                    self.config['screen_preview'] = bool(value)
                    sender = getattr(backend, 'screen_sender', None)
                    if sender is not None: sender.enabled = bool(value)
                elif name=='motion_config':
                    backend.configure_motion(value)
                    self.log('运动预判参数已更新；仅观察，不下发动作')
                elif name=='motion_reset':
                    backend.motion_predictor.reset();backend.background_motion.reset()
                    self.log('已清空运动轨迹，开始重新学习')
            except (ValueError,RuntimeError) as exc:self.log(str(exc))

    def wireless_capable(self,control):
        hello=getattr(control,'hello',{})
        maximum=hello.get('obstacle_max_age_ms')
        return (self.camera_role=='qr' and hello.get('role')=='qr' and
                hello.get('obstacle_uart_type')==0x86 and type(maximum) is int and maximum>0)

    def wireless_obstacle(self,backend):
        """Send one current measured observation; the bridge stamps UART age."""
        if self.mode!='run' or self.stop_event.is_set():
            self.wireless_reason='独立预览只显示，不回传障碍'
            return
        report=getattr(backend,'obstacle_report',None) or {}
        selected=report.get('selected')
        if not report.get('reportable') or selected is None:
            self.wireless_reason='CAM2无线桥接可用；等待确认障碍'
            return
        key=tuple(report.get('frame_key') or ())
        frame=getattr(backend.control,'frame',None)
        if (len(key)!=3 or frame is None or key!=tuple(frame) or
                key[0]!=backend.control.epoch or key==self.wireless_last_frame):
            return
        now=time.perf_counter()
        frame_time=report.get('frame_pc_perf_s')
        if not isinstance(frame_time,(int,float)):
            return
        age=now-frame_time
        maximum=min(200,backend.control.hello['obstacle_max_age_ms'])/1000.
        if not 0<=age<=maximum:
            self.wireless_reason='观测已过期，等待CAM2新图像'
            return
        has_new_context=getattr(backend.control,'has_new_context',None)
        if has_new_context is not None and has_new_context(key[0]):
            return
        gap,right=round(selected['front_gap_mm']),round(selected['body_right_mm'])
        if not (50<=gap<=500 and -190<=right<=190):
            return
        head=bytes((0x6b,0x86,0))+struct.pack('<Hh',gap,right)
        backend.control.write(head+bytes((qr_service.crc8(head),)))
        self.wireless_last_frame=key
        self.wireless_sent+=1
        self.wireless_reason='已发往CAM2无线桥；主控是否采纳以日志为准'

    def publish(self,backend,video):
        wireless=self.wireless_capable(backend.control)
        if wireless:
            if self.obstacle_sink is not None:self.obstacle_sink(None)
            self.wireless_obstacle(backend)
            obstacle_link={'channel':'cam2_wifi','reason':self.wireless_reason,
                           'sent':self.wireless_sent,'last_frame':self.wireless_last_frame}
        elif self.camera_role=='qr' and self.mode=='preview':
            if self.obstacle_sink is not None:self.obstacle_sink(None)
            obstacle_link={'channel':'preview','reason':'独立预览只显示，不回传障碍','sent':0}
        else:
            obstacle_link={'channel':'serial_fallback','reason':'相机桥不支持无线障碍；等待主控串口','sent':0}
        intervals=list(backend.intervals)
        fps=1/np.mean(intervals[-60:]) if intervals else 0.
        meta=backend.last_meta
        image=backend.display_image.copy() if backend.display_image is not None else None
        sender = getattr(backend, 'screen_sender', None)
        if sender is not None: sender.submit(image, meta)
        snapshot={"image":image,"frame":meta[1] if meta else 0,"fps":fps,
                  "camera_epoch":meta[0] if meta else None,"camera_read_finished_ns":meta[2] if meta else None,
                  "pc_frame_perf_s":backend.previous_frame,"pc_frame_wall_time":backend.last_pc_frame_wall_time,
                  "raw_image":backend.raw_image.copy() if backend.raw_image is not None else None,
                  "surface_report":backend.surface_report,"ring_report":backend.ring_report,
                  "motion_report":backend.motion_report,
                  "operation":self.mode,"return_report":backend.control.observation_status(),
                  "camera_role":self.camera_role,"qr_report":getattr(backend,'qr_report',None),
                  "camera_screen":{'available':bool(sender and sender.port), 'sent':sender.sent if sender else 0,
                                   'error':sender.error if sender else None},
                  "obstacle_report":getattr(backend,'obstacle_report',None),"obstacle_link":obstacle_link,
                  "decode_ok":backend.decode_ok,"decode_fail":backend.decode_fail,
                  "wait_ms":backend.last_transfer_ms,"decode_ms":backend.last_decode_ms,
                  "detect_ms":backend.last_detect_ms,"read_ms":meta[5]/1000 if meta else 0.,
                  "encode_ms":meta[6]/1000 if meta and len(meta)>6 else None,
                  "send_ms":meta[7]/1000 if meta and len(meta)>7 else None,
                  "network":video.mode,"skipped":video.skipped,"status":backend.status,
                  "config":backend.config,"calibration":bool(backend.config[0]==2 and backend.config[4]&4),
                  "color":backend.cal_color-1,"points":len(backend.points),"cal_done":backend.cal_done}
        with self.lock:self.latest=snapshot
        if self.camera_role=='qr' and not wireless and self.mode=='run' and self.obstacle_sink is not None:
            self.obstacle_sink(None if self.stop_event.is_set() else {
                'operation':self.mode,'camera_role':'qr','obstacle_report':snapshot['obstacle_report']})

    def connected(self,folder):
        control=video=backend=None
        try:
            control=ControlClient(self.config["host"],self.config["control_port"])
            peer_role=control.hello.get('role','material')
            if peer_role!=self.camera_role:raise ValueError('相机角色不匹配：请核对CAM1/CAM2 IP与对应桥接程序')
            self.wireless_last_frame=None
            if self.wireless_capable(control) and self.obstacle_sink is not None:
                self.obstacle_sink(None)
            size=(640,480) if self.camera_role=='qr' else (480,360)
            if tuple(control.hello.get('frame_size',size))!=size:raise ValueError('相机分辨率与当前通道不匹配')
            video=VideoClient(self.config["host"],self.config["video_port"],mode=control.hello.get("video_mode","pull-v1"),control=control,expected_size=size)
            path=Path(self.config["calibration_file"])
            if not path.is_absolute():path=self.config_path.parent/path
            backend=(PcQrBackend if self.camera_role=='qr' else PcBackend)(video,control,path,show=False)
            backend.external_ui=True
            backend.screen_sender=ScreenPreviewClient(self.config['host'],control.hello)
            backend.screen_sender.enabled=self.config.get('screen_preview',True)
            backend.preview_only=self.mode=="preview"
            if self.camera_role!='qr':backend.configure_motion(self.config.get('motion'))
            engine=(qr_service.QrService if self.camera_role=='qr' else service.VisionService)(control,backend,clock=time.perf_counter)
            self.log("已连接 "+str(control.hello.get("build"))+" / "+video.mode)
            if video.mode=="pull-v1":self.log("相机仍是旧拉帧程序，重新上传1.1.0可启用连续图传")
            csv_path=folder/("gui_frames_"+time.strftime("%H%M%S")+".csv")
            with csv_path.open("w",newline="",encoding="utf-8") as stream:
                rows=csv.writer(stream)
                rows.writerow(["pc_perf_s","epoch","frame","camera_ns","read_us","encode_us","previous_send_us",
                               "video_mode","wait_ms","decode_ms","detect_ms","mode","status","surface_report","ring_report","motion_report","return_report","camera_role","qr_report","pc_frame_wall_time","obstacle_report","obstacle_link"])
                previous,next_flush=0,0.
                while not self.stop_event.is_set():
                    control.check()
                    self.handle_commands(backend)
                    if self.mode=="preview":
                        for _ in range(32):
                            if not control.read(256):break
                        if backend.config[0]==0:
                            try:backend.fetch();backend.status="原图解码正常"
                            except StreamContextChanged:continue
                        elif self.camera_role=='qr':
                            result=backend.observe(time.perf_counter(),scan=backend.task_display is None)
                            backend.preview(backend.config,result,result.get('task') or backend.task_display)
                        else:backend.observe(time.perf_counter())
                    elif not engine.tick():
                        backend.idle_preview()
                        self.stop_event.wait(.002)
                    if self.camera_role=='qr' and self.mode=='run':
                        backend.preview(engine.config,None,engine.locked_task or engine.saved_display)
                    if video.error is not None:raise ConnectionError(str(video.error))
                    self.publish(backend,video)
                    meta=backend.last_meta
                    if meta and meta[1]!=previous:
                        rows.writerow([round(time.perf_counter(),6),meta[0],meta[1],meta[2],meta[5],
                                       meta[6] if len(meta)>6 else "",meta[7] if len(meta)>7 else "",
                                       video.mode,round(backend.last_transfer_ms,3),round(backend.last_decode_ms,3),
                                       round(backend.last_detect_ms,3),backend.config[0],backend.status,
                                       json.dumps(backend.surface_report,ensure_ascii=True),json.dumps(backend.ring_report,ensure_ascii=True),
                                       json.dumps(backend.motion_report,ensure_ascii=True),
                                       json.dumps(control.observation_status(),ensure_ascii=True),self.camera_role,
                                       json.dumps(getattr(backend,'qr_report',None),ensure_ascii=True),backend.last_pc_frame_wall_time,
                                       json.dumps(getattr(backend,'obstacle_report',None),ensure_ascii=True),
                                       json.dumps(self.latest.get('obstacle_link'),ensure_ascii=True)])
                        previous=meta[1]
                    if time.perf_counter()>=next_flush:
                        stream.flush();next_flush=time.perf_counter()+2
        finally:
            if backend and getattr(backend,'screen_sender',None): backend.screen_sender.close()
            if backend:backend.close()
            if video:video.close()
            if control:control.close()

    def run(self):
        service.PREVIEW_ENABLED=False
        cv2.setNumThreads(1)
        folder=HERE/"logs"/time.strftime("%Y-%m-%d")
        if self.camera_role=='qr':folder=folder/'cam2'
        folder.mkdir(parents=True,exist_ok=True)
        if self.camera_role!='qr':service.DIAGNOSTIC_LOG_PATH=str(folder/"service_errors.log")
        while not self.stop_event.is_set():
            try:self.connected(folder)
            except Exception as exc:
                self.log("连接或图像异常："+str(exc)+"；正在重试")
                if self.obstacle_sink is not None:self.obstacle_sink(None)
                with self.lock:self.latest=None
                self.stop_event.wait(self.config["reconnect_seconds"])
        self.log("连接已断开")


class App:
    def __init__(self,root,config_path,mode="preview",autoconnect=True,worker_factory=VisionWorker,camera_role=None,embedded=False,serial_log=None):
        self.root,self.config_path,self.worker_factory=root,config_path,worker_factory
        self.config=read_config(config_path)
        self.camera_role=camera_role or self.config.get('camera_role','material')
        self.config['camera_role']=self.camera_role
        if self.camera_role=='qr' and not config_path.exists():self.config['calibration_file']='calibration.cam2.json'
        self.embedded=embedded
        self.serial_log=serial_log
        self.preview_modes=QR_PREVIEW_MODES if self.camera_role=='qr' else PREVIEW_MODES
        self.worker=None
        self._disconnecting=False
        self.latest=None
        self.photo=None
        self.screen_preview = tk.BooleanVar(value=True)
        self.screen_status = tk.StringVar(value='新桥接应用支持相机屏幕同步电脑标注，最高10FPS')
        self._canvas_image=None
        self._image_position=None
        self._photo_size=None
        self._controls_state=None
        self._selected_mode=None
        self._wrap_width=None
        self._menu_open=False
        self._menu_poll=None
        self.motion_dialog=None
        self.motion_vars={}
        self.selecting_grasp=False
        self.image_geometry=None
        self.last_ui_frame=None
        self.closing=False
        self._refresh_after=self._autoconnect_after=self._disconnect_after=None
        self._scan_after=None;self._scan_thread=None;self._scan_results=None
        self._scan_cancel=threading.Event();self._scan_autoconnect=False
        self._discovered={}
        self._recent_hosts=list(dict.fromkeys(([self.config['host']] if self.config['host'] else [])+self.config.get('recent_hosts',[])))[:8]
        if not embedded:
            root.title("工训双路视觉上位机 2.6.7（初赛决赛统一调试）")
            set_app_icon(root)
            root.geometry("1180x800");root.minsize(1000,720)
            root.configure(bg="#edf1f5")
        style=ttk.Style(root)
        if not embedded:style.theme_use("clam")
        style.configure("TFrame",background="#edf1f5")
        style.configure("TLabel",background="#edf1f5",font=("Microsoft YaHei UI",10))
        style.configure("TButton",font=("Microsoft YaHei UI",10),padding=7)
        style.configure("Selected.TButton",background="#d9e9ff",foreground="#174f9b")
        style.configure("Title.TLabel",font=("Microsoft YaHei UI",18,"bold"))
        style.configure("TLabelframe",background="#edf1f5")
        style.configure("TLabelframe.Label",font=("Microsoft YaHei UI",10,"bold"),background="#edf1f5")
        self.host=tk.StringVar(value=self.config["host"])
        self.video_port=tk.StringVar(value=str(self.config["video_port"]))
        self.control_port=tk.StringVar(value=str(self.config["control_port"]))
        self.operation=tk.StringVar(value="独立预览" if mode=="preview" else "主控联动")
        self.connection=tk.StringVar(value="未连接")
        self.status=tk.StringVar(value="连接相机后开始图传。独立预览不向小车回传数据。")
        self.color=tk.StringVar(value=COLORS[0])
        self.surface_enabled=tk.BooleanVar(value=True)
        self.surface_all=tk.BooleanVar(value=False)
        self.ring_refinement=tk.BooleanVar(value=True)
        self.use_surface_output=tk.BooleanVar(value=False)
        self._build()
        if not embedded:root.protocol("WM_DELETE_WINDOW",self.close)
        root.bind('<Destroy>',self._on_destroy,add='+')
        self._refresh_after=root.after(33,self.refresh)
        if autoconnect:self._autoconnect_after=root.after(200,lambda:self.scan_cameras(autoconnect=True))

    def _build(self):
        header=ttk.Frame(self.root,padding=(18,12));header.pack(fill="x")
        ttk.Label(header,text="CAM2 扫码与前方障碍" if self.camera_role=='qr' else "CAM1 物料与定位",style="Title.TLabel").pack(side="left")
        self.run_button=ttk.Button(header,text="接入运行并回传",command=self.connect_run)
        self.run_button.pack(side="left",padx=20)
        ttk.Label(header,textvariable=self.connection,foreground="#167d5d").pack(side="right")
        bar=ttk.Frame(self.root,padding=(18,0,18,12));bar.pack(fill="x")
        ttk.Label(bar,text="相机 IP").pack(side="left")
        self.host_entry=ttk.Combobox(bar,textvariable=self.host,values=self._recent_hosts,width=18)
        self.host_entry.pack(side="left",padx=(6,4));self.host_entry.bind('<<ComboboxSelected>>',self.select_device)
        self.scan_button=ttk.Button(bar,text='扫描相机',command=self.scan_cameras)
        self.scan_button.pack(side='left',padx=(0,8))
        self.port_entries=[]
        for label,var in (("图像端口",self.video_port),("通讯端口",self.control_port)):
            ttk.Label(bar,text=label).pack(side="left")
            entry=ttk.Entry(bar,textvariable=var,width=6)
            entry.pack(side="left",padx=(5,10));self.port_entries.append(entry)
        self.operation_combo=ttk.Combobox(bar,textvariable=self.operation,values=("独立预览","主控联动"),state="readonly",width=10)
        self.operation_combo.pack(side="left",padx=5)
        self.connect_button=ttk.Button(bar,text="连接相机",command=self.connect);self.connect_button.pack(side="left",padx=4)
        self.disconnect_button=ttk.Button(bar,text="断开",command=self.disconnect,state="disabled");self.disconnect_button.pack(side="left",padx=4)
        options=ttk.Menubutton(bar,text="增强选项");options.pack(side='left',padx=6)
        self.options_menu=tk.Menu(options,tearoff=False,postcommand=self._menu_posted)
        self.options_menu.bind('<Unmap>',self._menu_closed,add='+')
        for label,name,var in (("物料顶面分析",'surface_enabled',self.surface_enabled),
                               ("全色表面分析（更耗时）",'surface_all',self.surface_all),
                               ("带物料圆环联合精修",'ring_refinement',self.ring_refinement),
                               ("回传增强中心（需核对抓取窗口）",'use_surface_output',self.use_surface_output)):
            self.options_menu.add_checkbutton(label=label,variable=var,command=lambda name=name,var=var:self.command(name,var.get()))
        options.configure(menu=self.options_menu)
        motion_button=ttk.Button(bar,text='运动预判',command=self.open_motion);motion_button.pack(side='left',padx=4)
        if self.camera_role=='qr':options.pack_forget();motion_button.pack_forget()
        screen_row=ttk.Frame(self.root,padding=(18,0,18,3));screen_row.pack(fill='x')
        ttk.Checkbutton(screen_row,text='相机屏幕同步标注',variable=self.screen_preview,
                        command=lambda:self.command('screen_preview',self.screen_preview.get())).pack(side='left',padx=3)
        ttk.Label(screen_row,textvariable=self.screen_status).pack(side='left',padx=8)
        body=ttk.Frame(self.root,padding=(18,0,18,10));body.pack(fill="both",expand=True)
        body.columnconfigure(1,weight=1);body.rowconfigure(0,weight=1)
        sidebar=ttk.Frame(body,width=215);sidebar.grid(row=0,column=0,sticky="ns",padx=(0,14))
        modes=ttk.LabelFrame(sidebar,text="识别切换",padding=10);modes.pack(fill="x")
        mode_grid=ttk.Frame(modes);mode_grid.pack(fill="x")
        mode_grid.columnconfigure(0,weight=1);mode_grid.columnconfigure(1,weight=1)
        self.mode_buttons={}
        for index,label in enumerate(self.preview_modes):
            button=ttk.Button(mode_grid,text=label,command=lambda label=label:self.select_mode(label))
            button.grid(row=index//2,column=index%2,sticky="ew",padx=2,pady=3);self.mode_buttons[label]=button
        color_label=ttk.Label(modes,text="物料颜色");color_label.pack(anchor="w",pady=(10,4))
        self.color_combo=ttk.Combobox(modes,textvariable=self.color,values=COLORS,state="readonly",width=16)
        self.color_combo.pack(fill="x");self.color_combo.bind("<<ComboboxSelected>>",self.select_color)
        calibration=ttk.LabelFrame(sidebar,text="三点颜色取样",padding=10);calibration.pack(fill="x",pady=(12,0))
        self.sample_status=tk.StringVar(value="进入颜色取样后，在顶面内部点3处，避开侧面与边缘")
        ttk.Label(calibration,textvariable=self.sample_status,wraplength=185).pack(fill="x",pady=(0,8))
        self.cal_buttons=[]
        cal_grid=ttk.Frame(calibration);cal_grid.pack(fill="x")
        cal_grid.columnconfigure(0,weight=1);cal_grid.columnconfigure(1,weight=1)
        for index,(label,name) in enumerate((("重新取样","clear_samples"),("保存到电脑","save_samples"),("保存并结束取样","finish_samples"))):
            button=ttk.Button(cal_grid,text=label,command=lambda name=name:self.command(name))
            button.grid(row=index//2,column=index%2,columnspan=2 if index==2 else 1,sticky="ew",padx=2,pady=3);self.cal_buttons.append(button)
        if self.camera_role=='qr':
            color_label.pack_forget();self.color_combo.pack_forget();calibration.pack_forget()
            ttk.Label(sidebar,text='完整640×480扫码\n默认倒装180°显示\n连续3帧确认后锁定\n主控任务：模式6 / 工位4',wraplength=220).pack(fill='x',pady=12)
            obstacle_box=ttk.LabelFrame(sidebar,text='前方障碍自动联动',padding=10)
            obstacle_box.pack(fill='x')
            ttk.Label(obstacle_box,text='CAM2桥1.4自动无线回传\n旧桥需主控串口调试回传\n比赛准备后自动联动\n按当前车速，观察前方\n相机左偏约100mm\n当前为尺寸粗估，非精确测距',wraplength=205).pack(fill='x')
        actions=ttk.LabelFrame(sidebar,text="记录",padding=10);actions.pack(fill="x",pady=(12,0))
        self.screenshot_button=ttk.Button(actions,text="保存截图",command=self.save_screenshot)
        self.screenshot_button.pack(side="left",expand=True,fill="x",padx=2)
        ttk.Button(actions,text="打开日志",command=self.open_logs).pack(side="left",expand=True,fill="x",padx=2)
        right=ttk.Frame(body);right.grid(row=0,column=1,sticky="nsew")
        right.rowconfigure(0,weight=1);right.columnconfigure(0,weight=1)
        self.canvas=tk.Canvas(right,bg="#111923",highlightthickness=0)
        self.canvas.grid(row=0,column=0,sticky="nsew");self.canvas.bind("<Button-1>",self.click_image)
        self.canvas.create_text(370,210,text="等待连接 MaixCAM",fill="#aab9ca",font=("Microsoft YaHei UI",18),tags="placeholder")
        self.telemetry=tk.StringVar(value="帧率 —    帧号 —    解码成功 —    等待图像 —")
        self.telemetry_label=ttk.Label(right,textvariable=self.telemetry,wraplength=900)
        self.telemetry_label.grid(row=1,column=0,sticky="ew",pady=(8,4))
        self.status_label=ttk.Label(right,textvariable=self.status,wraplength=900,foreground="#53657a")
        self.status_label.grid(row=2,column=0,sticky="ew")
        self.return_status=tk.StringVar(value='坐标回传：未连接；预览与主控联动均支持实时画面和截图。')
        self.return_label=ttk.Label(right,textvariable=self.return_status,wraplength=900,foreground='#167d5d')
        self.return_label.grid(row=3,column=0,sticky='ew',pady=(4,0))
        self.geometry_status=tk.StringVar(value='扫码任务与解码状态将在这里显示。' if self.camera_role=='qr' else '增强几何结果将在这里显示；质量分不是正确率。')
        self.geometry_label=ttk.Label(right,textvariable=self.geometry_status,wraplength=900,foreground='#174f9b')
        self.geometry_label.grid(row=4,column=0,sticky='ew',pady=(4,0))
        self.motion_status=tk.StringVar(value='运动预判：未启用；点击顶部“运动预判”设置，仅观察。')
        self.motion_label=ttk.Label(right,textvariable=self.motion_status,wraplength=900,foreground='#8b3fa7')
        self.motion_label.grid(row=5,column=0,sticky='ew',pady=(4,0))
        if self.camera_role=='qr':self.motion_status.set(obstacle_text(None))
        logbox=ttk.LabelFrame(self.root,text="运行日志",padding=6);logbox.pack(fill="x",padx=18,pady=(0,12))
        self.log_text=tk.Text(logbox,height=4,font=("Microsoft YaHei UI",9),bg="#f8fafc",relief="flat",state="disabled")
        self.log_text.pack(fill="x")
        self.set_controls(False)

    def log(self,text):
        self.log_text.configure(state="normal")
        self.log_text.insert("end",text+"\n");self.log_text.see("end")
        if int(self.log_text.index("end-1c").split(".")[0])>150:self.log_text.delete("1.0","30.0")
        self.log_text.configure(state="disabled")

    def set_controls(self,connected,calibration=False):
        preview=self.operation.get()=="独立预览"
        signature=(bool(connected),bool(calibration),preview,self._disconnecting)
        if signature==self._controls_state:return
        self.run_button.configure(state='disabled' if self._disconnecting or connected and not preview else 'normal')
        self.connect_button.configure(state="disabled" if connected else "normal")
        self.disconnect_button.configure(state="normal" if connected else "disabled")
        self.host_entry.configure(state="disabled" if connected else "normal")
        for entry in self.port_entries:entry.configure(state="disabled" if connected else "normal")
        self.operation_combo.configure(state="disabled" if connected else "readonly")
        for button in self.mode_buttons.values():button.configure(state="normal" if connected and preview else "disabled")
        self.color_combo.configure(state="readonly" if connected and (preview or calibration) else "disabled")
        for button in self.cal_buttons:button.configure(state="normal" if connected and calibration else "disabled")
        self.options_menu.entryconfigure(3,state='normal' if connected and not preview else 'disabled')
        if self._disconnecting:
            self.disconnect_button.configure(state='disabled')
            for button in self.mode_buttons.values():button.configure(state='disabled')
            self.color_combo.configure(state='disabled')
            for button in self.cal_buttons:button.configure(state='disabled')
        self._controls_state=signature

    @staticmethod
    def _set_text(variable,value):
        if variable.get()!=value:variable.set(value)

    def _menu_posted(self):
        self._menu_open=True
        if self._menu_poll is not None:self.root.after_cancel(self._menu_poll)
        self._menu_poll=self.root.after_idle(self._check_menu)

    def _check_menu(self):
        self._menu_poll=None
        if self.closing:return
        if self.options_menu.winfo_ismapped():
            self._menu_poll=self.root.after(80,self._check_menu)
        else:self._menu_closed()

    def _menu_closed(self,event=None):
        self._menu_open=False
        if self._menu_poll is not None:
            self.root.after_cancel(self._menu_poll)
            self._menu_poll=None

    def connect(self):
        if self.worker is not None:return
        if not self.host.get().strip():
            self.scan_cameras(autoconnect=True);return
        try:
            host=self.host.get().strip()
            if not host:raise ValueError("请输入相机网络设置中显示的IP地址")
            recent=list(dict.fromkeys([host]+self._recent_hosts))[:8]
            config=dict(self.config,host=host,video_port=int(self.video_port.get()),control_port=int(self.control_port.get()),recent_hosts=recent)
            for key in ("video_port","control_port"):
                if not 1<=config[key]<=65535:raise ValueError("端口应为1～65535")
            self.config_path.parent.mkdir(parents=True,exist_ok=True)
            self.config_path.write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding="utf-8")
            self.config=config
            self._recent_hosts=recent
            self.host_entry.configure(values=list(dict.fromkeys(list(self._discovered)+recent)))
        except (ValueError,OSError) as exc:messagebox.showerror("配置错误",str(exc),parent=self.root);return
        mode="preview" if self.operation.get()=="独立预览" else "run"
        self.latest=None;self.last_ui_frame=None
        self.worker=self.worker_factory(config,self.config_path,mode)
        self.worker.command('screen_preview',self.screen_preview.get())
        if self.camera_role=='qr' and self.serial_log is not None:
            self.worker.obstacle_sink=self.serial_log.update_obstacle
        if self.camera_role!='qr':
            for name in ('surface_enabled','surface_all','ring_refinement','use_surface_output'):
                self.worker.command(name,getattr(self,name).get())
        self.connection.set("正在连接 "+host)
        self.status.set("独立预览：按钮只切换电脑识别，不回传数据。" if mode=="preview" else "主控联动：识别模式由STM32的SET指令决定。")
        self.return_status.set('坐标回传：未启用（独立预览）。' if mode=='preview' else '坐标回传：正在接入，等待相机连接和STM32任务。')
        self.set_controls(True);self.worker.start()

    def scan_cameras(self,autoconnect=False):
        if self.closing:return
        self._scan_autoconnect=self._scan_autoconnect or autoconnect
        if self._scan_results is not None:return
        self._scan_cancel=threading.Event();self._scan_results=queue.Queue()
        known=tuple(dict.fromkeys([self.host.get().strip()]+self._recent_hosts))
        self.scan_button.configure(state='disabled',text='扫描中…')
        if self.worker is None:self.connection.set('正在搜索相机…')
        self._scan_thread=threading.Thread(target=discover_in_background,
                                          args=(self._scan_results,known,self._scan_cancel),daemon=True)
        self._scan_thread.start()
        self._scan_after=self.root.after(80,self._finish_scan)

    def _finish_scan(self):
        self._scan_after=None
        if self.closing:return
        try:devices,error=self._scan_results.get_nowait()
        except queue.Empty:
            self._scan_after=self.root.after(80,self._finish_scan);return
        self._scan_results=None
        self._scan_thread=None
        self.scan_button.configure(state='normal',text='扫描相机')
        self.apply_discovered_devices(devices)
        if error:self.log('设备扫描失败：'+error)
        autoconnect=self._scan_autoconnect;self._scan_autoconnect=False
        if autoconnect and self.worker is None and self.host.get().strip():self.connect()

    def apply_discovered_devices(self,devices):
        matching=[item for item in devices if item['role']==self.camera_role]
        self._discovered={item['host']:item for item in matching}
        self.host_entry.configure(values=list(dict.fromkeys(list(self._discovered)+self._recent_hosts)))
        if self.worker is not None:
            self.log('扫描完成：发现本通道 %d 台相机；当前连接保持原地址'%len(matching))
            return
        if self.host.get() not in self._discovered and len(self._discovered)==1:
            self.host.set(next(iter(self._discovered)))
        elif len(self._discovered)>1 and self.host.get() not in self._discovered:
            self.host.set('')
        self.select_device()
        if matching:
            self.connection.set('发现 %d 台 · 未连接'%len(matching))
            self.status.set('在IP下拉框选择相机，再连接或接入运行。')
            self.log('扫描发现：'+'；'.join(item['host']+' '+item['name'] for item in matching))
        else:
            self.connection.set('未发现在线相机')
            self.status.set('保留历史IP；搜索需要1.3.0桥接应用，并连接同一热点。')

    def select_device(self,event=None):
        if self.worker is not None:return
        device=self._discovered.get(self.host.get())
        if device:
            self.video_port.set(str(device['video_port']));self.control_port.set(str(device['control_port']))

    def connect_run(self):
        if self._disconnecting:return
        if self.worker is not None and self.operation.get()=='主控联动':return
        def start():
            self.operation.set('主控联动')
            self.connect()
        if self.worker is not None:
            self.log('正在结束预览连接，随后接入主控联动；画面会短暂重连')
            self.disconnect(after=start)
        else:start()

    def disconnect(self,after=None):
        if self.worker is None:return
        if self._disconnecting:return
        if self.camera_role=='qr' and self.serial_log is not None:
            self.serial_log.update_obstacle(None)
        self._disconnecting=True
        worker=self.worker;worker.stop()
        self.latest=None
        self.connection.set("正在断开…")
        self.return_status.set('坐标回传：正在断开当前连接。')
        self.set_controls(True)
        def finish():
            if self.closing:return
            if worker.thread is not None and worker.thread.is_alive():
                self._disconnect_after=self.root.after(100,finish);return
            self._disconnect_after=None
            if self.worker is worker:self.worker=None
            self._disconnecting=False
            self.connection.set("未连接");self.set_controls(False)
            self.return_status.set('坐标回传：已断开。')
            if after is not None:after()
        self._disconnect_after=self.root.after(50,finish)

    def command(self,name,value=None):
        if self.worker:self.worker.command(name,value)

    def select_mode(self,label):
        if self.operation.get()=="独立预览":self.command("select",(label,COLORS.index(self.color.get())))

    def obstacle_link_status(self):
        link=(self.latest or {}).get('obstacle_link')
        if link and link.get('channel')!='serial_fallback':return link
        if self.serial_log is not None:
            status=self.serial_log.obstacle_status()
            status['channel']='serial_fallback'
            status['reason']='有线调试回退：'+status['reason']
            return status
        return link

    def select_color(self,event=None):self.command("color",COLORS.index(self.color.get()))

    def click_image(self,event):
        if self.selecting_grasp:
            if self.image_geometry is None:return
            left,top,scale,width,height=self.image_geometry
            x,y=(event.x-left)/scale,(event.y-top)/scale
            if 0<=x<width and 0<=y<height:
                self.motion_vars['grasp_x'].set('%.1f'%(x*320/width))
                self.motion_vars['grasp_y'].set('%.1f'%(y*240/height))
                self.selecting_grasp=False;self.canvas.configure(cursor='')
                self.log('抓取点已选中；在运动预判窗口点击“应用参数”生效')
            return
        if self.latest is None or not self.latest["calibration"] or self.image_geometry is None:return
        left,top,scale,width,height=self.image_geometry
        x,y=int((event.x-left)/scale),int((event.y-top)/scale)
        if 0<=x<width and 0<=y<height:self.command("sample",(x,y))

    def save_screenshot(self):
        if not self.latest or self.latest["image"] is None:
            messagebox.showinfo("截图","当前还没有图像",parent=self.root);return
        saved=timestamp()
        metadata={key:self.latest.get(key) for key in ('frame','camera_epoch','camera_read_finished_ns','status','config','surface_report','ring_report','motion_report','obstacle_report','obstacle_link','operation','return_report','camera_role','qr_report','pc_frame_perf_s','pc_frame_wall_time')}
        metadata=json.loads(json.dumps(metadata,ensure_ascii=False))
        metadata.update(pc_saved_wall_time=saved['wall_time'],pc_saved_perf_s=saved['perf_s'])
        folder=HERE/"screenshots"
        if self.camera_role=='qr':folder=folder/'cam2'
        folder.mkdir(parents=True,exist_ok=True)
        stem=datetime.now().strftime('%Y%m%d_%H%M%S_%f')+"_%d"%self.latest["frame"]
        path=folder/(stem+'_raw.png')
        raw=self.latest.get('raw_image')
        cv2.imencode('.png',raw if raw is not None else self.latest['image'])[1].tofile(str(path))
        cv2.imencode('.png',self.latest['image'])[1].tofile(str(folder/(stem+'_overlay.png')))
        if self.serial_log is not None:
            if self.camera_role=='qr':metadata['obstacle_link']=self.obstacle_link_status()
            camera='CAM2' if self.camera_role=='qr' else 'CAM1'
            metadata['serial']=self.serial_log.mark_snapshot(camera,self.latest['frame'],
                self.latest.get('pc_frame_perf_s') or saved['perf_s'],path,saved)
            context=metadata['serial']
            lines=['%s frame=%s'%(camera,self.latest['frame']),
                   'Frame PC time: '+str(metadata['pc_frame_wall_time']),
                   'Saved PC time: '+saved['wall_time'],
                   'Serial log: '+str(context['log_path']),
                   'Serial marker record: '+str(context['marker_record_id']),
                   'Serial connected: '+str(context['connected']),
                   'Controller stage before frame: '+str((context['last_stage_before_frame'] or {}).get('text')),
                   'Nearby serial RX/TX (PC frame decode -2s through save):']
            lines+=['[%s] %s %s'%(r['wall_time'],r['kind'],r['text']) for r in context['nearby_serial']]
            (folder/(stem+'_serial.txt')).write_text('\n'.join(lines)+'\n',encoding='utf-8')
        (folder/(stem+'.json')).write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
        self.log("原图与标注图已保存："+str(path))

    def open_logs(self):
        folder=HERE/"logs"
        if self.camera_role=='qr':folder=folder/time.strftime('%Y-%m-%d')/'cam2'
        folder.mkdir(parents=True,exist_ok=True)
        os.startfile(folder)

    def open_motion(self):
        if self.motion_dialog is not None and self.motion_dialog.winfo_exists():
            self.motion_dialog.lift();return
        dialog=tk.Toplevel(self.root);self.motion_dialog=dialog
        dialog.title('圆周运动预判 · 仅观察');dialog.resizable(False,False)
        frame=ttk.Frame(dialog,padding=16);frame.pack(fill='both',expand=True)
        options=motion_settings(self.config.get('motion'))
        self.motion_vars={'enabled':tk.BooleanVar(value=options['enabled'])}
        ttk.Checkbutton(frame,text='启用预测（仅观察，不下发夹爪指令）',variable=self.motion_vars['enabled']).grid(row=0,column=0,columnspan=2,sticky='w',pady=(0,10))
        labels=(('lead_ms','动作到有效夹持时间 / ms（需实测）'),('latency_ms','图像延迟估计 / ms（手动补偿）'),
                ('grasp_x','抓取点 X / px'),('grasp_y','抓取点 Y / px'),
                ('half_width','窗口半宽 / px'),('half_height','窗口半高 / px'),('error_limit_px','观测验证误差上限 / px'))
        for row,(key,label) in enumerate(labels,1):
            ttk.Label(frame,text=label).grid(row=row,column=0,sticky='w',padx=(0,12),pady=4)
            var=tk.StringVar(value='%g'%options[key]);self.motion_vars[key]=var
            ttk.Entry(frame,textvariable=var,width=12).grid(row=row,column=1,sticky='e')
        buttons=ttk.Frame(frame);buttons.grid(row=8,column=0,columnspan=2,sticky='ew',pady=10)
        ttk.Button(buttons,text='点选抓取点',command=self.select_grasp).pack(side='left',padx=3)
        ttk.Button(buttons,text='重新学习轨迹',command=lambda:self.command('motion_reset')).pack(side='left',padx=3)
        ttk.Button(buttons,text='应用参数',command=self.apply_motion).pack(side='left',padx=3)
        ttk.Label(frame,text='固定相机，选择物料颜色，让物料转动约1～3秒。\n青色箭头：方向；橙色点：待验证；紫色点：预测通过观测校验。\n默认180/80ms只是试验值，需测量实际动作与图像延迟。\n圆/椭圆是图像轨迹近似；大透视和机械坐标仍需标定。',wraplength=480).grid(row=9,column=0,columnspan=2,sticky='w')
        dialog.protocol('WM_DELETE_WINDOW',self.close_motion)

    def close_motion(self):
        self.selecting_grasp=False;self.canvas.configure(cursor='')
        if self.motion_dialog is not None:self.motion_dialog.destroy();self.motion_dialog=None

    def select_grasp(self):
        if self.latest is None or self.latest['config'][0]!=2 or self.latest['calibration']:
            messagebox.showinfo('点选抓取点','请先切换到物料识别，等到图像显示后再点选。',parent=self.root);return
        self.selecting_grasp=True;self.canvas.configure(cursor='crosshair')
        self.log('请在图像中点击夹爪实际接触位置（仅设置预测窗口）')

    def apply_motion(self):
        try:
            values={key:(var.get() if key=='enabled' else float(var.get())) for key,var in self.motion_vars.items()}
            options=motion_settings(values)
            self.config['motion']=options
            self.config_path.write_text(json.dumps(self.config,ensure_ascii=False,indent=2),encoding='utf-8')
        except (ValueError,OSError) as exc:
            messagebox.showerror('参数错误',str(exc),parent=self.motion_dialog);return
        if options['enabled']:
            self.surface_enabled.set(True);self.command('surface_enabled',True)
        self.command('motion_config',options)
        if self.worker is not None:self.worker.config['motion']=options
        self.log('运动预判参数已保存；预测结果仅显示和记录，不改变回传坐标')

    def draw_image(self,image):
        available_w,available_h=max(1,self.canvas.winfo_width()),max(1,self.canvas.winfo_height())
        h,w=image.shape[:2];scale=min(available_w/w,available_h/h)
        if scale<=0:return
        rgb=cv2.cvtColor(cv2.resize(image,(max(1,round(w*scale)),max(1,round(h*scale)))),cv2.COLOR_BGR2RGB)
        header=("P6\n%d %d\n255\n"%(rgb.shape[1],rgb.shape[0])).encode("ascii")
        data=header+rgb.tobytes()
        if self.photo is None:self.photo=tk.PhotoImage(data=data,format="PPM")
        else:
            options={'data':data,'format':'PPM'}
            if self._photo_size!=(rgb.shape[1],rgb.shape[0]):options.update(width=rgb.shape[1],height=rgb.shape[0])
            self.photo.configure(**options)
        self._photo_size=(rgb.shape[1],rgb.shape[0])
        left,top=(available_w-rgb.shape[1])//2,(available_h-rgb.shape[0])//2
        if self._canvas_image is None:
            self.canvas.delete('placeholder')
            self._canvas_image=self.canvas.create_image(left,top,image=self.photo,anchor="nw")
        elif self._image_position!=(left,top):
            self.canvas.coords(self._canvas_image,left,top)
        self._image_position=(left,top)
        self.image_geometry=left,top,scale,w,h

    def refresh(self):
        self._refresh_after=None
        if self.closing:return
        if self.camera_role=='qr' and self.serial_log is not None:
            link=self.obstacle_link_status()
            self._set_text(self.motion_status,obstacle_text(self.latest.get('obstacle_report') if self.latest else None,link))
        # Native popup menus must not be reconfigured/repainted by video timers.
        # The worker continues receiving, detecting and logging in its own thread.
        if self._menu_open:
            self._refresh_after=self.root.after(33,self.refresh)
            return
        width=max(300,self.canvas.winfo_width())
        if width!=self._wrap_width:
            for label in (self.telemetry_label,self.status_label,self.return_label,self.geometry_label,self.motion_label):label.configure(wraplength=width)
            self._wrap_width=width
        if self.worker and not self._disconnecting:
            snapshot,events=self.worker.snapshot()
            for event in events:self.log(event)
            if snapshot is not None:
                self.latest=snapshot
                screen=snapshot.get('camera_screen') or {}
                self.screen_status.set('已关闭标注回传，相机显示本地原图' if not self.screen_preview.get() else
                    '相机桥接未提供标注回显；需要CAM1 1.3.1 / CAM2 1.4.1' if not screen.get('available') else
                    '标注回显连接异常，主图传仍独立运行' if screen.get('error') else
                    '标注已发送%d帧（最高10FPS）；实际小屏显示请现场核对' % screen.get('sent',0))
                self._set_text(self.connection,"已连接 · "+("连续图传" if snapshot["network"]=="push-v2" else "旧版逐帧图传"))
                self.set_controls(True,snapshot["calibration"])
                config=snapshot['config']
                selected=("二维码识别" if config[0]==6 else "原始画面" if config[0]==0 else "颜色取样" if snapshot['calibration'] else
                          "物料识别" if config[0]==2 else "带物料圆环" if config[0]==3 and config[4]&2 else
                          "空圆环" if config[0]==3 else "圆盘切点")
                if selected!=self._selected_mode:
                    for label,button in self.mode_buttons.items():button.configure(style="Selected.TButton" if label==selected else "TButton")
                    self._selected_mode=selected
                self._set_text(self.status,task_text(config)+' | '+snapshot["status"])
                self._set_text(self.return_status,return_text(snapshot))
                surface=snapshot.get('surface_report');ring_info=snapshot.get('ring_report');qr_info=snapshot.get('qr_report')
                labels={'SURFACE_OK':'已确认顶面','ACQUIRE':'正在连续确认','TOP_UNRESOLVED':'顶面和侧面未分开',
                        'COLOR_AMBIGUOUS':'颜色存在歧义','FACE_AMBIGUOUS':'顶面候选有歧义',
                        'SAME_COLOR_AMBIGUOUS':'存在多个同色目标','BLUE_PAIR_CALIBRATION':'请分别取样蓝色和浅蓝色',
                        'NO_TOP_EVIDENCE':'顶面边界证据不足','COLOR_OVERLAP':'颜色标定重叠',
                        'NO_CONFIRMED_SURFACE':'未确认顶面'}
                if qr_info:
                    summary='任务码：'+(qr_info.get('task') or '尚未锁定')+' | '+qr_info['status']
                    if qr_info.get('candidate') and not qr_info.get('task'):summary+=' | 候选 '+qr_info['candidate']
                    summary+=' | '+qr_info['decoder']+' / 640×480'
                    self._set_text(self.geometry_status,summary)
                elif surface:
                    summary=color_text(surface)+'顶面：'+labels.get(surface['status'],surface['status'])
                    if surface['quality'] is not None:
                        summary+=' | 质量 %.2f（非概率）| 轴 %.1f×%.1f px | 连续 %d 帧'%(surface['quality'],*surface['axes'],surface['stable'])
                    if surface.get('rim_coverage') is not None:
                        summary+=' | 实测边缘覆盖 %.0f%%'%(100*surface['rim_coverage'])
                    summary+=' | 回传：'+('顶面中心' if surface['output']=='surface' else '旧版中心')
                    self._set_text(self.geometry_status,summary)
                elif ring_info:
                    ring_labels={'RING_BASELINE':'当前帧圆环观测','JOINT_RING_OK':'多层共同中心已确认','NEED_THREE':'等待三组圆环身份',
                                 'NEED_THREE_VISIBLE_LAYERS':'可见环层不足','INCONSISTENT_ARCS':'弧线不一致',
                                 'LAYER_DISAGREEMENT':'各层圆心不一致','RING_SPACING_MISMATCH':'环层间距不匹配',
                                 'ILL_CONDITIONED':'弧线覆盖不足'}
                    reason=ring_info.get('reason','未知')
                    summary='带料环：'+ring_labels.get(reason,reason)
                    if 'profile' in ring_info:
                        summary=('带料环：' if ring_info['profile']=='occupied' else '空环：')+ring_labels.get(reason,reason)
                        summary+=' | %d/3组 | '%ring_info['ring_count']+('坐标有效' if ring_info['coordinates_valid'] else '定位待确认')
                    if 'p95_px' in ring_info:summary+=' | 共同中心残差P95 %.2f px | 层间差 %.2f px'%(ring_info['p95_px'],ring_info.get('spread_px',0))
                    self._set_text(self.geometry_status,summary+' | '+('精修回传' if ring_info['output']=='enhanced' else '旧版回传'))
                else:self._set_text(self.geometry_status,'扫码任务与解码状态将在这里显示。' if self.camera_role=='qr' else '增强几何结果将在这里显示；质量分不是正确率。')
                if self.camera_role=='qr':
                    self._set_text(self.motion_status,obstacle_text(snapshot.get('obstacle_report'),self.obstacle_link_status()))
                else:self._set_text(self.motion_status,motion_text(snapshot.get('motion_report')))
                self._set_text(self.sample_status,("%s：%d / 3 点；请点顶面内部"%(COLORS[snapshot["color"]],snapshot["points"])) if snapshot["calibration"] else "进入颜色取样后，在顶面内部点3处，避开侧面与边缘")
                if snapshot["calibration"] or self.operation.get()=="主控联动" and config[0]==2:self._set_text(self.color,COLORS[snapshot["color"]])
                timing="JPEG编码 %.2f ms，上一帧发送 %.2f ms"%(snapshot["encode_ms"],snapshot["send_ms"]) if snapshot["encode_ms"] is not None else "旧相机程序未提供压缩与发送计时"
                wait_label="等待新图" if snapshot['network']=='push-v2' else "拉帧往返"
                self.telemetry.set("%.1f FPS  | 帧 %d | 解码 %d 成功 / %d 失败 | 读帧 %.2f ms\n%s\n%s %.2f ms | 解码 %.2f ms / 识别 %.2f ms | 跳过旧帧 %d"%
                                   (snapshot["fps"],snapshot["frame"],snapshot["decode_ok"],snapshot["decode_fail"],snapshot["read_ms"],timing,wait_label,snapshot['wait_ms'],snapshot["decode_ms"],snapshot["detect_ms"],snapshot["skipped"]))
                key=(snapshot["frame"],self.canvas.winfo_width(),self.canvas.winfo_height())
                if snapshot["image"] is not None and key!=self.last_ui_frame and (not self.embedded or self.canvas.winfo_viewable()):
                    self.draw_image(snapshot["image"]);self.last_ui_frame=key
            elif not self.worker.stop_event.is_set():
                self.latest=None
                self._set_text(self.connection,"连接中 / 自动重试")
                self._set_text(self.return_status,'坐标回传：连接中，尚未接入。')
                self.set_controls(True,False)
        self._refresh_after=self.root.after(33,self.refresh)

    def _on_destroy(self,event):
        if event.widget is not self.root:return
        self.closing=True
        self._scan_cancel.set()
        for name in ('_refresh_after','_autoconnect_after','_disconnect_after','_menu_poll','_scan_after'):
            pending=getattr(self,name)
            if pending is not None:
                self.root.after_cancel(pending)
                setattr(self,name,None)

    def close(self):
        self.closing=True
        self._menu_closed()
        if self.worker:self.worker.stop()
        self.root.destroy()


class DualApp:
    """Two panels share only the Tk thread; sockets, queues and decoders are separate."""
    def __init__(self,root,cam1_path,cam2_path,mode='preview',autoconnect=True,worker_factory=VisionWorker,camera_roles=('material','qr')):
        self.root=root
        root.title('工训双路视觉上位机 2.6.7（初赛决赛统一调试）');root.geometry('1180x870');root.minsize(1000,780)
        set_app_icon(root)
        root.configure(bg='#edf1f5');ttk.Style(root).theme_use('clam')
        bar=ttk.Frame(root,padding=(12,8));bar.pack(fill='x')
        self.notebook=ttk.Notebook(root);self.notebook.pack(fill='both',expand=True)
        self.serial_log=SerialLog(HERE/'logs'/time.strftime('%Y-%m-%d')/'serial')
        self.panels=[]
        for path,role,title in ((cam1_path,'material','CAM1 · 物料 / 圆环 / 圆盘'),(cam2_path,'qr','CAM2 · 扫码 / 前方障碍')):
            if role not in camera_roles:continue
            page=ttk.Frame(self.notebook);self.notebook.add(page,text=title)
            self.panels.append(App(page,path,mode,autoconnect,worker_factory,camera_role=role,embedded=True,serial_log=self.serial_log))
        serial_page=ttk.Frame(self.notebook);self.notebook.add(serial_page,text='主控串口日志')
        self.serial_panel=SerialPanel(serial_page,self.serial_log,HERE/'serial_config.json',refresh=autoconnect)
        arm_page=ttk.Frame(self.notebook);self.notebook.add(arm_page,text='机械臂控制')
        self.arm_panel=ArmPanel(arm_page,self.serial_log,connect_vision=self.connect_run)
        turn_page=ttk.Frame(self.notebook);self.notebook.add(turn_page,text='转向PID')
        self.turn_panel=TurnPanel(turn_page,self.serial_log)
        ttk.Button(bar,text='双路接入运行',command=self.connect_run).pack(side='left',padx=4)
        for panel in self.panels:
            title='CAM2：' if panel.camera_role=='qr' else 'CAM1：'
            ttk.Label(bar,text=title).pack(side='left',padx=(16,0))
            ttk.Label(bar,textvariable=panel.connection).pack(side='left')
        ttk.Label(bar,text='串口：').pack(side='left',padx=(16,0))
        ttk.Label(bar,textvariable=self.serial_panel.summary).pack(side='left')
        root.protocol('WM_DELETE_WINDOW',self.close)

    def connect_run(self):
        for panel in self.panels:panel.connect_run()

    def close(self):
        self.serial_log.stop()
        for panel in self.panels:panel.close()
        self.root.destroy()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=HERE/"config.json")
    parser.add_argument("--mode",choices=("preview","run"),default="preview")
    parser.add_argument("--no-autoconnect",action="store_true")
    parser.add_argument('--camera',choices=('both','material','qr'),default='both')
    parser.add_argument('--cam2-config',type=Path,default=HERE/'config.cam2.json')
    args=parser.parse_args()
    folder=HERE/"logs"/time.strftime("%Y-%m-%d");folder.mkdir(parents=True,exist_ok=True)
    logging.basicConfig(level=logging.INFO,format="%(asctime)s %(message)s",handlers=[logging.FileHandler(folder/("gui_"+time.strftime("%H%M%S")+".log"),encoding="utf-8")])
    root=tk.Tk()
    app=DualApp(root,args.config,args.cam2_config,args.mode,not args.no_autoconnect,
                camera_roles=('material','qr') if args.camera=='both' else (args.camera,))
    root.mainloop()
    for panel in app.panels if isinstance(app,DualApp) else (app,):
        if panel.worker and panel.worker.thread:panel.worker.thread.join(timeout=3)
    if app.serial_log.thread:app.serial_log.thread.join(timeout=3)


if __name__=="__main__":main()
