"""PC array backend for the existing service and detectors; wire coordinates stay measured."""
import collections
from datetime import datetime
import importlib.util
import json
import logging
import math
from pathlib import Path
import time
from types import SimpleNamespace

import cv2
import numpy as np
from material_surface import SurfaceAnalyzer
from ring_refinement import refine_occupied
from motion_prediction import MotionPredictor, BackgroundMotion
from transport import StreamContextChanged
from parking import ParkingDetector
from color_classifier import pc_calibrated_threshold,black_pixel_gate

ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


material = load("pc_material", "vision/maixcam_material/main.py")
ring = load("pc_ring", "vision/maixcam_ring/middle_ring.py")
plate = load("pc_plate", "vision/maixcam_material/plate_tangent.py")


def lab_values(bgr):
    return cv2.cvtColor(bgr.astype(np.float32)/255.0, cv2.COLOR_BGR2LAB)


class Blob:
    def __init__(self, stats, center):
        self.stats, self.center = stats, center

    def x(self): return int(self.stats[0])
    def y(self): return int(self.stats[1])
    def w(self): return int(self.stats[2])
    def h(self): return int(self.stats[3])
    def pixels(self): return int(self.stats[4])
    def cx(self): return float(self.center[0])
    def cy(self): return float(self.center[1])


class Mask:
    def __init__(self, mask): self.array = mask
    def width(self): return self.array.shape[1]
    def height(self): return self.array.shape[0]

    def find_blobs(self, thresholds, **kwargs):
        _, _, stats, centers = cv2.connectedComponentsWithStats(self.array, 8)
        minimum = max(kwargs.get("area_threshold", 0), kwargs.get("pixels_threshold", 0))
        return [Blob(s, c) for s, c in zip(stats[1:], centers[1:]) if s[4] >= minimum]


class MaterialFrame:
    def __init__(self, bgr, allowed_mask=None, lab=None):
        self.array,self.mask,self.allowed_mask,self.lab=bgr,None,allowed_mask,lab

    def binary(self, thresholds, copy=False):
        threshold = thresholds[0]
        if self.lab is None:self.lab=lab_values(self.array)
        self.mask = cv2.inRange(self.lab, np.array(threshold[::2], np.float32),
                                np.array(threshold[1::2], np.float32))
        if self.allowed_mask is not None:self.mask[~self.allowed_mask]=0
        return self

    def to_format(self, fmt): return Mask(self.mask)


MATERIAL_API = SimpleNamespace(Format=SimpleNamespace(FMT_GRAYSCALE=0),
                               image2cv=lambda mask, **kwargs: mask.array)


class PcBackend:
    def __init__(self, video, control, calibration_path, show=True):
        self.video, self.control = video, control
        self.capture_size=(480,360)
        self.rotation=0
        self.path, self.show_window = Path(calibration_path), show
        initial = control.hello.get("calibration", {})
        if self.path.exists():
            initial = json.loads(self.path.read_text(encoding="utf-8"))
        self.samples = material.valid_samples(initial.get("samples", {}))
        self.config = (0, 0, 0, 0, 0)
        self.stage, self.status = "idle", "Waiting for STM32 SET"
        self.image = self.display_image = None
        self.raw_image = None
        self.surface_enabled,self.surface_all,self.use_surface_output = True,False,False
        self.surface_report = None
        self.ring_refinement=True
        self.ring_report=None
        self.motion_predictor=MotionPredictor()
        self.parking_detector=ParkingDetector();self.parking_report=None
        self.background_motion=BackgroundMotion()
        self.motion_report=None
        self.frame_count, self.decode_ok, self.decode_fail = 0, 0, 0
        self.last_meta, self.last_transfer_ms, self.last_detect_ms, self.last_decode_ms = None, 0., 0., 0.
        self.intervals, self.transfers = collections.deque(maxlen=300), collections.deque(maxlen=300)
        self.previous_frame = None
        self.last_pc_frame_wall_time=None
        self.next_ui = 0.
        self.window_open = False
        self.preview_only = False
        self.external_ui = False
        self.cal_color, self.points, self.cal_done = 1, [], False
        self.binary_view, self.geometry = False, None
        self.selected_color = 0
        self.surface_analyzer = SurfaceAnalyzer(material)
        self.reset_detectors()

    def reset_detectors(self):
        self.ring_detector, self.ring_tracker = ring.RingDetector(), ring.MiddleTracker()
        self.ring_detector.require_scene = True
        self.plate_tracker = plate.PlateTracker()
        self.selected_color = 0
        self.surface_analyzer = SurfaceAnalyzer(material)
        self.surface_report = None
        self.material_confirm_id,self.material_confirm_stamp,self.material_confirm_count=None,None,0
        self.ring_report=None;self.parking_report=None
        self.motion_predictor.reset()
        self.background_motion.reset()
        self.motion_report=None

    def confirm_material(self,identity,coords,stamp):
        """Two distinct consecutive frames; invalid/color/gap changes reset acquisition."""
        if coords is None:
            self.material_confirm_id,self.material_confirm_stamp,self.material_confirm_count=None,None,0
            return None
        if self.material_confirm_id!=identity or self.material_confirm_stamp is None or not 0<=stamp-self.material_confirm_stamp<=.25:
            self.material_confirm_count=0
        if stamp!=self.material_confirm_stamp or self.material_confirm_id!=identity:
            self.material_confirm_count+=1
        self.material_confirm_id,self.material_confirm_stamp=identity,stamp
        return coords if self.material_confirm_count>=2 else None

    def configure_motion(self,options):
        self.motion_predictor.configure(options)
        self.background_motion.reset()
        self.motion_report=None

    def configure(self, config):
        if config[0] not in (0, 2, 3, 8, 9):
            raise ValueError("Unsupported mode")
        if config[0] == 2 and config[4] & 4 and not (self.show_window or self.external_ui):
            raise ValueError("Color calibration requires desktop window")
        if config[0] != 0:
            self.fetch()  # ACK only after the real camera/decoder can deliver an image
        self.config = config
        self.reset_detectors()
        self.ring_detector.allow_occupied = bool(config[4] & 2)
        self.points, self.cal_done = [], False
        self.cal_color = (0,1,2,3,4,6,5)[config[2]] or 1
        self.status = "Waiting for image" if config[0] else "STOP / waiting for STM32 SET"
        logging.info("Configured mode=%s", config)

    def fetch(self):
        self.stage = "network frame"
        meta, jpeg, transfer = self.video.get(self.control.epoch)
        start = time.perf_counter()
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.last_decode_ms = (time.perf_counter()-start)*1000
        if bgr is None or bgr.shape != (self.capture_size[1],self.capture_size[0],3):
            self.decode_fail += 1
            raise ValueError("JPEG decode/size failed")
        now = time.perf_counter()
        if self.previous_frame is not None:
            self.intervals.append(now-self.previous_frame)
        self.previous_frame = now
        self.last_pc_frame_wall_time=datetime.now().astimezone().isoformat(timespec='milliseconds')
        self.transfers.append(transfer)
        self.last_meta, self.last_transfer_ms = meta, transfer
        self.control.frame = meta[:3]
        self.frame_count += 1
        self.decode_ok += 1
        if self.rotation==180:bgr=cv2.rotate(bgr,cv2.ROTATE_180)
        self.image = self.display_image = bgr
        self.raw_image = bgr.copy()
        return bgr

    def observe(self, now):
        try:
            bgr = self.fetch()
        except StreamContextChanged:
            # VisionService polls SET again before submitting any observation.
            # Yield without treating a normal task switch as a camera failure.
            return {"coordinates": None, "id": self.config[2], "scale": 1.0}
        start = time.perf_counter()
        mode, station, requested, hz, flags = self.config
        coords, identity = None, requested
        result = {"coordinates": None, "id": requested, "scale": 1.0}
        if mode == 9:
            coords,axis,self.display_image,self.parking_report=self.parking_detector.observe(bgr)
            self.image=self.display_image.copy();identity=2
            result.update(ring_count=3 if coords is not None else 0,ring_count_trusted=coords is not None,ring_axis=axis)
            self.status='PARK BLUE '+self.parking_report['reason']
        elif mode == 2:
            captured=bgr
            bgr = cv2.resize(bgr, (320, 240), interpolation=cv2.INTER_AREA)
            self.image, self.display_image = bgr, bgr.copy()
            if flags & 4:
                self.status = "CAL DONE: wait STM32 STOP" if self.cal_done else "CAL: 1-6 color / click 3 points / S save / E finish"
            else:
                face=None
                inverse = (0,1,2,3,4,6,5)
                chosen = inverse[requested] if requested else self.selected_color
                target, diagnostics = None, {}
                surface_output=self.surface_enabled and (self.preview_only or self.use_surface_output)
                # Black has no hue to separate it from the gripper, a shadow or the grey
                # floor, so it may only report pixels from a confirmed black TOP FACE.
                black_top_face = not surface_output and chosen == 5
                if not surface_output and not black_top_face:
                    choices=[chosen] if chosen else list(range(1,7))
                    for color_id in choices:
                        threshold,calibration=pc_calibrated_threshold(material.COLORS[color_id-1],self.samples,material)
                        if calibration=='OVERLAP':continue
                        lab=lab_values(bgr) if color_id==5 else None
                        allowed=black_pixel_gate(lab,cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)) if color_id==5 else None
                        target,mask=material.detect_material_center(MaterialFrame(bgr,allowed,lab),threshold,MATERIAL_API,diagnostics)
                        if target is not None or chosen:
                            chosen=color_id;break
                    if chosen:self.selected_color,identity=chosen,inverse[chosen]
                    if target is not None:coords=(target[0]-160,120-target[1])
                    coords=self.confirm_material(identity,coords,self.last_meta[2]/1e9)
                if self.surface_enabled or black_top_face:
                    scene=self.surface_analyzer.process(bgr,self.samples,chosen,
                                                        stamp=self.last_meta[2]/1e9,all_colors=self.surface_all,raw_bgr=captured)
                    self.display_image=self.surface_analyzer.annotate(bgr,scene)
                    self.surface_report=self.surface_analyzer.describe(scene,chosen)
                    self.surface_report['output']='surface' if surface_output else ('surface-black' if black_top_face else 'legacy')
                    face=scene['target']
                    if surface_output:
                        self.confirm_material(identity,None,self.last_meta[2]/1e9)
                        if face is not None:
                            chosen=face['color'];self.selected_color=chosen;identity=inverse[chosen]
                            x,y=face['center'];coords=(x-160,120-y)
                        else:coords=None
                    elif black_top_face:
                        # No dark-blob fallback: without a confirmed black top face the
                        # pixel stays invalid and the mainboard keeps waiting.
                        self.confirm_material(identity,None,self.last_meta[2]/1e9)
                        if face is not None:
                            chosen=face['color'];self.selected_color=chosen;identity=inverse[chosen]
                            x,y=face['center'];coords=(x-160,120-y)
                        else:coords=None
                    elif target is not None:
                        point=(round(target[0]),round(target[1]))
                        cv2.drawMarker(self.display_image,point,(0,0,255),cv2.MARKER_TILTED_CROSS,12,1)
                    self.status='Material %d %s / output=%s'%(identity,scene['reason'],self.surface_report['output'])
                else:
                    self.surface_report=None
                    if target is not None:cv2.circle(self.display_image,(round(target[0]),round(target[1])),round(target[2]),(0,255,0),1)
                    self.status='Material %d %s'%(identity,diagnostics.get('reason','NO_TARGET'))
                if self.motion_predictor.options['enabled']:
                    camera_motion=self.background_motion.update(bgr)
                    point=face['center'] if face is not None else None
                    uid=(face['color'],face['uid']) if face is not None else None
                    self.motion_report=self.motion_predictor.update(point,self.last_meta[2]/1e9,uid,camera_motion)
                    if not self.surface_enabled:self.motion_report['state']='SURFACE_REQUIRED'
                    self.display_image=self.motion_predictor.annotate(self.display_image,self.motion_report)
                else:self.motion_report=None
        elif mode == 8:
            bgr = cv2.resize(bgr, (320, 240), interpolation=cv2.INTER_AREA)
            self.image, self.display_image = bgr, bgr.copy()
            diagnostics = {}
            # PC already has320x240 pixels here. Fit them directly instead
            # of applying the device preview's second reduction to224px.
            raw, _ = plate.fit_plate_color_array(bgr, diagnostics, order="BGR")
            target = self.plate_tracker.update(raw)
            aim_y=round(bgr.shape[0]*plate.PLATE_AIM_Y_FRACTION)
            cv2.line(self.display_image,(0,aim_y),(bgr.shape[1]-1,aim_y),(255,255,255),1)
            reason = diagnostics.get("reason", "UNKNOWN")
            scene, trusted = 0, False
            if target is not None:
                coords = plate.tangent_alignment_error(target, 320, 240, 320, 240)
                scene, trusted = 3, True
                tangent_y=round(target["tangent_y"])
                cv2.line(self.display_image,(0,tangent_y),(bgr.shape[1]-1,tangent_y),(0,255,0),1)
                cv2.circle(self.display_image, (round(target["tangent_x"]),round(target["tangent_y"])), 5, (0,255,0), 2)
            elif (raw is not None and reason == "EDGE_OK") or reason == "AMBIGUOUS_EDGE":
                scene, trusted = 2, True
            elif raw is None and reason == "NO_EDGE_ARC":
                scene, trusted = 1, True
            identity, self.status = 0, "Plate " + reason
            result.update(plate_scene=scene, plate_scene_trusted=trusted)
        elif mode == 3:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            groups, triple, _, status = self.ring_detector.process(gray, self.ring_tracker, now)
            baseline_groups=groups
            enhanced=bool(flags&2 and self.ring_refinement)
            apply_refined=enhanced and (self.preview_only or self.use_surface_output)
            analyzed=[]
            if enhanced:
                for group in groups:
                    refined,quality=refine_occupied(gray,bgr,group,ring)
                    analyzed.append(dict(refined or group,joint_refined=refined is not None,joint_quality=quality))
                if apply_refined:
                    groups=analyzed
                    if triple is not None:triple,_=ring.choose_middle(groups)
            if self.ring_detector.identity_rejected and triple is None:
                self.ring_tracker.reset()
            observed = self.ring_tracker.update(groups, triple, now, status)
            self.ring_detector.accept(observed, triple, gray, now)
            trusted = self.ring_detector.scene_trusted
            if observed is not None and trusted and len(groups) == 3:
                coords = ring.screen_error(observed["position"], (480,360), (480,360))
                if apply_refined and not observed['ring'].get('joint_refined'):coords=None
            if enhanced:
                selected=None
                if triple is not None:
                    selected=min(analyzed,key=lambda group:math.dist(group['center'],triple['ring']['center'])) if analyzed else None
                self.ring_report=dict(selected.get('joint_quality',{}) if selected else {'reason':'NEED_THREE'},
                                      output='enhanced' if apply_refined else 'legacy')
                if selected:
                    baseline=min(baseline_groups,key=lambda group:math.dist(group['center'],selected['center']))
                    self.ring_report.update(baseline_center=baseline['center'],
                                            refined_center=selected['center'] if selected.get('joint_refined') else None)
                if apply_refined and selected and not selected.get('joint_refined'):status+=' / ARC_UNCERTAIN'
            else:self.ring_report={'reason':'RING_BASELINE','output':'legacy'}
            self.ring_report.update(profile='occupied' if flags&2 else 'empty',ring_count=len(groups),
                                    scene_trusted=trusted,status=status,coordinates_valid=coords is not None,
                                    selected_center=observed['position'] if observed is not None else None)
            self.display_image = ring.annotate(gray, groups, observed if not apply_refined or coords is not None else None, None, status, coords)
            identity, self.status = 2, "Ring " + status
            result.update(ring_count=min(len(groups),3), ring_count_trusted=trusted)
            if flags & 1:
                axis_ok=not apply_refined or all(group.get('joint_refined') for group in groups)
                result["ring_axis"] = ring.ring_axis_measurement(groups,triple,observed) if coords is not None and axis_ok else None
        result.update(coordinates=coords, id=identity)
        self.last_detect_ms = (time.perf_counter()-start)*1000
        self.stage = "observe_ok"
        return result

    def invalidate(self):
        self.reset_detectors()

    def preview(self, *args):
        pass  # UI is scheduled separately from existing protocol service

    def idle_preview(self):
        try:
            self.fetch()
        except StreamContextChanged:
            pass  # return to the service immediately so it can process the SET

    def calibration_click(self, event, x, y, flags, extra):
        if event != cv2.EVENT_LBUTTONDOWN or not (self.config[0] == 2 and self.config[4] & 4) or self.cal_done:
            return
        if self.geometry is None or self.image is None or len(self.points) >= 3:
            return
        left, top, scale, width, height = self.geometry
        px, py = int((x-left)/scale), int((y-top)/scale)
        if not (0 <= px < width and 0 <= py < height):
            return
        self.collect_sample(px,py)

    def collect_sample(self,px,py):
        if self.image is None or self.cal_done or not (self.config[0] == 2 and self.config[4] & 4) or len(self.points) >= 3:
            return
        height,width = self.image.shape[:2]
        if not (0 <= px < width and 0 <= py < height):
            return
        patch = self.image[max(0,py-3):min(height,py+4), max(0,px-3):min(width,px+4)]
        value = np.median(lab_values(patch).reshape(-1,3), axis=0).tolist()
        material.valid_samples({str(self.cal_color): [value]})
        self.points.append(value)
        if len(self.points) == 3:
            self.samples[str(self.cal_color)] = self.points.copy()
        logging.info("Calibration color=%d point=%d LAB=%s", self.cal_color, len(self.points), value)

    def save_calibration(self,done=False):
        if 0 < len(self.points) < 3:
            raise ValueError("请先完成当前颜色的3点取样")
        self.path.parent.mkdir(parents=True,exist_ok=True)
        material.save_samples(self.samples,str(self.path))
        if done:
            self.cal_done = True
        logging.info("Calibration saved to PC: %s",self.path)

    def render(self):
        canvas = np.zeros((690, 800, 3), np.uint8)
        intervals = list(self.intervals)
        fps = 1/np.mean(intervals[-60:]) if intervals else 0.
        p95 = np.percentile(self.transfers,95) if self.transfers else 0.
        streaming = getattr(self.video,"mode","") == "push-v2"
        metric = "Wait latest" if streaming else "Request->JPEG"
        lines = ["PC VISION  |  Q exit  |  %s" % ("continuous / latest frame" if streaming else "legacy pull: upload bridge 1.1.0"),
                 "Mode %d  Frame %s  Decode OK %d / FAIL %d  FPS %.1f" %
                 (self.config[0], self.last_meta[1] if self.last_meta else "-",self.decode_ok,self.decode_fail,fps),
                 "%s %.1f ms (P95 %.1f) Decode %.2f Detect %.2f ms" %
                 (metric,self.last_transfer_ms,p95,self.last_decode_ms,self.last_detect_ms), self.status[:100]]
        for i,line in enumerate(lines):
            cv2.putText(canvas,line,(12,22+i*24),cv2.FONT_HERSHEY_SIMPLEX,.48,(230,230,230),1,cv2.LINE_AA)
        if self.display_image is not None:
            h,w = self.display_image.shape[:2]
            scale = min(800/w, 540/h)
            preview = cv2.resize(self.display_image,(round(w*scale),round(h*scale)))
            top, left = 112, (800-preview.shape[1])//2
            canvas[top:top+preview.shape[0],left:left+preview.shape[1]] = preview
            self.geometry = left,top,scale,w,h
        if self.config[0] == 2 and self.config[4] & 4:
            footer = "CAL color %d: %d/3 | 1-6 select, C reset, S save PC, E finish" % (self.cal_color,len(self.points))
        else:
            footer = ("Preview: V raw / M material / R ring / O occupied / P plate / 1-6 color" if self.preview_only
                      else "Request->JPEG includes capture/encode/network; not exposure-to-MCU latency.")
        cv2.putText(canvas,footer,(10,678),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,230,230),1,cv2.LINE_AA)
        return canvas

    def show(self):
        ui_started = time.perf_counter()
        if not self.show_window or ui_started < self.next_ui:
            return
        if not self.window_open:
            cv2.namedWindow("PC Vision",cv2.WINDOW_NORMAL)
            cv2.setMouseCallback("PC Vision",self.calibration_click)
            self.window_open = True
        cv2.imshow("PC Vision",self.render())
        key = cv2.waitKey(1) & 255
        if key in (ord('q'),27) or cv2.getWindowProperty("PC Vision",cv2.WND_PROP_VISIBLE) < 1:
            raise KeyboardInterrupt
        if self.preview_only:
            choices = {ord('v'):(0,0,0,0,0),ord('m'):(2,1,1,20,0),ord('r'):(3,2,2,20,1),
                       ord('o'):(3,3,2,20,3),ord('p'):(8,1,0,20,0)}
            if key in choices:
                self.configure(choices[key])
            elif ord('1') <= key <= ord('6'):
                color = (0,1,2,3,4,6,5)[key-ord('0')]
                self.configure((2,1,color,20,0))
        if self.config[0] == 2 and self.config[4] & 4:
            if ord('1') <= key <= ord('6'):
                self.cal_color, self.points = key-ord('0'), []
            elif key in (ord('c'),ord('C')):
                self.points = []
            elif key in (ord('s'),ord('S'),ord('e'),ord('E')):
                if 0 < len(self.points) < 3:
                    self.status = "Complete 3 points first"
                else:
                    self.path.parent.mkdir(parents=True,exist_ok=True)
                    material.save_samples(self.samples, str(self.path))
                    logging.info("Calibration saved to PC: %s",self.path)
                    if key in (ord('e'),ord('E')):
                        self.cal_done = True
        self.next_ui = ui_started + 1/30

    def close(self):
        if self.window_open:
            cv2.destroyAllWindows()
