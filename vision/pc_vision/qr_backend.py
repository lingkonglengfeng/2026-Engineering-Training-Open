"""CAM2 PC QR decoding; reuses the frozen service's task/sequence contract.

Decode the complete 640x480 image so a fast crop cannot hide another task.
The installed MaixCAM receives no actuator commands from this backend.
"""
import time

import cv2
import numpy as np

from backend import PcBackend,load
from transport import StreamContextChanged
from obstacle_detection import ObstacleDetector

qr_service=load('pc_qr_contract','vision/maixcam_qr/service.py')
# This module is the PC's isolated service instance; the legacy standalone
# camera entry retains its own defaults. One complete legal image confirms.
QR_CONFIRM_FRAMES=1
qr_service.CONFIRM_FRAMES=QR_CONFIRM_FRAMES


class PcQrBackend(PcBackend):
    def __init__(self,video,control,calibration_path,show=False):
        super().__init__(video,control,calibration_path,show)
        self.capture_size=(640,480)
        self.rotation=int(control.hello.get('rotation',180))
        if self.rotation not in (0,180):raise ValueError('CAM2 rotation must be 0 or 180')
        detector_type=getattr(cv2,'QRCodeDetectorAruco',cv2.QRCodeDetector)
        self.decoder=detector_type()
        self.decoder_name=detector_type.__name__
        self.tracker=qr_service.TaskTracker()
        self.qr_report=None
        self.task_display=None
        self.status='WAIT SET6 / CAM2'
        self.obstacle_detector=ObstacleDetector()
        self.obstacle_report=None
        self.obstacle_detection_enabled=True

    def set_obstacle_detection_enabled(self,enabled):
        enabled=bool(enabled)
        if enabled!=self.obstacle_detection_enabled:
            self.obstacle_detector.reset();self.obstacle_report=None
            self.obstacle_detection_enabled=enabled

    def fetch(self):
        bgr=super().fetch()
        if hasattr(self,'obstacle_detector'):
            if self.obstacle_detection_enabled:
                self.obstacle_report=self.obstacle_detector.update(bgr,tuple(self.last_meta[:3]),self.previous_frame)
            else:
                self.obstacle_report=dict(status='DISABLED_MANUAL',confirmed=False,confirmed_frames=0,reportable=False,selected=None,candidates=[],detect_ms=0.)
            self.display_image=bgr.copy()
            self.obstacle_detector.draw(self.display_image)
        return bgr

    def configure(self,config):
        if config[0] not in (0,6):raise ValueError('CAM2 supports STOP0 / QR6 only')
        if config[0]==6:
            if config[1]!=4 or config[2]!=0 or config[4]!=0:raise ValueError('CAM2 QR requires station4 target0 flags0')
            self.fetch()
            self.task_display=None
        self.config=config
        self.tracker.reset()
        self.qr_report=None
        self.obstacle_detector.reset();self.obstacle_report=None
        self.status='SCANNING' if config[0]==6 else 'STOP / waiting for STM32 SET6'

    def observe(self,now,scan=True):
        try:bgr=self.fetch()
        except StreamContextChanged:return {'task':None}
        start=time.perf_counter();task=None;points=None;decoded=()
        if scan:
            gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
            _,decoded,points,_=self.decoder.detectAndDecodeMulti(gray)
            decoded=tuple(decoded or ())
            if len(decoded)>16:
                self.tracker.reset();self.tracker.status='TOO_MANY_QR'
            elif points is not None and (len(points)>len(decoded) or any(not text for text in decoded)):
                self.tracker.reset();self.tracker.status='INCOMPLETE_QR'
            else:task=self.tracker.update(decoded)
            self.status=self.tracker.status
        else:self.status='LATCHED'
        self.last_detect_ms=(time.perf_counter()-start)*1000
        self.qr_report={'status':self.status,'task':self.task_display,'candidate':self.tracker.pending,
                        'confirmed_frames':self.tracker.count,'decoder':self.decoder_name,
                        'decode_size':list(self.capture_size),'rotation':self.rotation,
                        'decoded_count':len(decoded),'decode_ms':self.last_detect_ms}
        self.display_image=bgr.copy()
        if points is not None:
            for polygon in np.asarray(points).reshape(-1,4,2):
                cv2.polylines(self.display_image,[np.rint(polygon).astype(np.int32)],True,(0,200,255),2,cv2.LINE_AA)
        self._label()
        self.obstacle_detector.draw(self.display_image)
        self.stage='observe_ok'
        return {'task':task}

    def _label(self):
        if self.display_image is None:return
        cv2.rectangle(self.display_image,(0,0),(self.display_image.shape[1],54),(15,20,25),-1)
        cv2.putText(self.display_image,'CAM2 QR '+self.status,(10,21),cv2.FONT_HERSHEY_SIMPLEX,.55,(220,240,240),1,cv2.LINE_AA)
        cv2.putText(self.display_image,self.task_display or 'Waiting for 1 valid frame',(10,45),
                    cv2.FONT_HERSHEY_SIMPLEX,.60,(80,240,150),1,cv2.LINE_AA)

    def preview(self,config,observation,text):
        self.task_display=text
        if text and config[0]==6:self.status='LATCHED'
        if self.qr_report is None:
            self.qr_report={'decoder':self.decoder_name,'decode_size':list(self.capture_size),'rotation':self.rotation}
        self.qr_report.update(task=text,status=self.status)
        self._label()

    def invalidate(self):
        self.tracker.reset();self.qr_report=None;self.status='CAMERA / DECODER ERROR'
        self.obstacle_detector.reset();self.obstacle_report=None

    def show(self):
        if not self.show_window or time.perf_counter()<self.next_ui:return
        self.window_open=True
        cv2.imshow('PC VISION QR',self.render())
        key=cv2.waitKey(1)&255
        if key in (ord('q'),27):raise KeyboardInterrupt
        if self.preview_only and key==ord('s'):self.configure((6,4,0,20,0))
        self.next_ui=time.perf_counter()+1/30
