"""统一视觉服务：SET切换物料/圆环/圆盘，ACK确认后发送位置，默认STOP。"""
import importlib.util
import gc
import math
import os
from pathlib import Path
import sys
import time
import types
import traceback
import struct

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
# Embedded project protocol8
from protocol8 import crc8, make_frame, encode_pixel_frame, MAT_PX, RING_PX, PLATE_PX

SET, ACK, HEARTBEAT = 0x80, 0x81, 0x82
RING_COUNT = 0x83
RING_AXIS = 0x0A
RING_AXIS_FLAG = 0x01
RING_OCCUPIED_FLAG = 0x02
RING_FLAGS_MASK = RING_AXIS_FLAG | RING_OCCUPIED_FLAG
MATERIAL_CALIBRATION_FLAG = 0x04
PLATE_SCENE = 0x85
PLATE_UNKNOWN, PLATE_SEARCH, PLATE_ACQUIRE, PLATE_FOUND = 0, 1, 2, 3
STOP, MATERIAL, RING, PLATE, PARK = 0, 2, 3, 8, 9
MODE_TYPES = {MATERIAL: MAT_PX, RING: RING_PX, PLATE: PLATE_PX, PARK: RING_PX}
PROTOCOL_VERSION = 1
PREVIEW_ENABLED, PREVIEW_HZ = True, 5
HEARTBEAT_PERIOD, PARTIAL_TIMEOUT, RECOVERY_GAP = .5, .05, .055
CAMERA_READ_TIMEOUT_MS = 250
DIAGNOSTIC_LOG_PATH = os.path.expanduser("~/maixcam_service_error.log")
DIAGNOSTIC_PERIOD_S = 2.0
SERVICE_BUILD = "2026-10-15-material20-plate30-v31"
# 原生MMF/VI保持一个RGB通道；识别各自的处理尺寸和线协议单位不变。
CAPTURE_WIDTH, CAPTURE_HEIGHT, CAPTURE_FPS = 480, 360, 30
PREVIEW_WIDTH, PREVIEW_HEIGHT = 320, 240  # 全部模式/提示图固定VO输入尺寸及RGB格式。
_BUNDLED_DETECTORS = {}  # build_vision_service.py 填入，单文件版不依赖其他py。


# LIGHT_GPIO_BEGIN
LIGHT_PIN = "A19"
LIGHT_GPIO = "GPIOA19"
LIGHT_OFF_LEVEL = 0  # 按高电平使能灯板；低电平使能板需改为1。
LIGHT_PWM_ENABLED = True  # 由各入口配置 GPIO/PWM；亮度为灯板占空比。
LIGHT_PWM_ID, LIGHT_PWM_FREQ_HZ = 7, 20000
LIGHT_BRIGHTNESS_PERCENT = 50.0
RING_LIGHT_BRIGHTNESS_PERCENT = 15.0
MATERIAL_LIGHT_BRIGHTNESS_PERCENT = 20.0  # Material color detection and calibration.
PLATE_LIGHT_BRIGHTNESS_PERCENT = 30.0


def _init_light(gpio, pinmap, err, enabled):
    if LIGHT_OFF_LEVEL not in (0, 1):
        raise ValueError("LIGHT_OFF_LEVEL must be 0 or 1")
    if enabled and LIGHT_PWM_ENABLED:
        return _init_pwm_light(gpio,pinmap,err)
    err.check_raise(pinmap.set_pin_function(LIGHT_PIN, LIGHT_GPIO), "Light GPIO pin mapping failed")
    level = 1 - LIGHT_OFF_LEVEL if enabled else LIGHT_OFF_LEVEL
    # SDK输出模式的Pull决定初始值；构造时直接设为所需电平。
    pull = gpio.Pull.PULL_DOWN if level == 0 else gpio.Pull.PULL_UP
    light = gpio.GPIO(LIGHT_GPIO, gpio.Mode.OUT, pull)
    light.value(level)
    print("[LIGHT] %s output=%d (%s; active level must match module)" % (
        LIGHT_GPIO, level, "ON" if enabled else "OFF"), flush=True)
    return light


def init_light_on(gpio, pinmap, err):
    return _init_light(gpio, pinmap, err, True)


def init_light_off(gpio, pinmap, err):
    return _init_light(gpio, pinmap, err, False)


class _PwmLight:
    def __init__(self,output,gpio,pinmap,err):
        self.output,self.gpio,self.pinmap,self.err=output,gpio,pinmap,err
        self.off_gpio=None

    def set_brightness(self, percent):
        import math
        if not isinstance(percent,(int,float)) or not math.isfinite(percent) or not 0 <= percent <= 100:
            raise ValueError('Invalid light brightness percent')
        if self.output is None:
            raise RuntimeError('Light PWM is disabled')
        duty = percent if LIGHT_OFF_LEVEL == 0 else 100-percent
        actual = self.output.duty(duty)
        if (not isinstance(actual,(int,float)) or not math.isfinite(actual)
                or not 0 <= actual <= 100 or abs(actual-duty)>0.1):
            raise RuntimeError('Light PWM duty request=%g returned=%r' % (duty,actual))
        return actual

    def value(self,level):
        if level != LIGHT_OFF_LEVEL:
            raise ValueError('PWM light handle only supports OFF cleanup')
        failure=None
        if self.output is not None:
            try:
                duty=self.output.duty(0 if LIGHT_OFF_LEVEL==0 else 100)
                if duty<0:raise RuntimeError('PWM duty OFF failed')
            except Exception as exc:failure=exc
            try:self.err.check_raise(self.output.disable(),'PWM disable failed')
            except Exception as exc:failure=exc
            self.output=None
        try:
            self.err.check_raise(self.pinmap.set_pin_function(LIGHT_PIN,LIGHT_GPIO),'Restore light GPIO failed')
            self.off_gpio.value(LIGHT_OFF_LEVEL)  # 复用已持有GPIO，避免重复申请同一引脚。
        except Exception as exc:failure=exc
        if failure is not None:raise failure


def _init_pwm_light(gpio,pinmap,err):
    from maix.peripheral import pwm
    if not 0 <= LIGHT_BRIGHTNESS_PERCENT <= 100 or LIGHT_PWM_FREQ_HZ <= 0:
        raise ValueError('Invalid light PWM configuration')
    # 用户确认灯板支持PWM。A19/PWM7按官方说明应使用USB验证Wi-Fi资源兼容。
    off=_init_light(gpio,pinmap,err,False)
    out=None
    try:
        duty=LIGHT_BRIGHTNESS_PERCENT if LIGHT_OFF_LEVEL==0 else 100-LIGHT_BRIGHTNESS_PERCENT
        print('[LIGHT PWM] A19/PWM7 %dHz %.1f%%; USB test recommended (Wi-Fi resources)' % (
            LIGHT_PWM_FREQ_HZ,LIGHT_BRIGHTNESS_PERCENT),flush=True)
        out=pwm.PWM(LIGHT_PWM_ID,freq=LIGHT_PWM_FREQ_HZ,duty=duty,enable=False)
        err.check_raise(pinmap.set_pin_function(LIGHT_PIN,'PWM%d'%LIGHT_PWM_ID),'Light PWM mapping failed')
        err.check_raise(out.enable(),'Light PWM enable failed')
    except Exception:
        if out is not None:
            try:out.disable()
            except Exception:pass
            out=None
        err.check_raise(pinmap.set_pin_function(LIGHT_PIN,LIGHT_GPIO),'Restore light GPIO failed')
        off.value(LIGHT_OFF_LEVEL)
        raise
    handle=_PwmLight(out,gpio,pinmap,err)
    handle.off_gpio=off
    return handle


def light_off_on_exit(light):
    if light is not None:
        try:
            light.value(LIGHT_OFF_LEVEL)
        except Exception as exc:
            print("[LIGHT] unable to set OFF on exit:", exc)
# LIGHT_GPIO_END

def report_fault(backend, stage, exc):
    """上报具体阶段/模式/异常栈；日志失败不能让识别服务再次退出。"""
    detail = "[SERVICE FAULT] stage=%s config=%s %s: %s\n%s" % (
        stage, getattr(backend, "config", "?"), type(exc).__name__, exc, traceback.format_exc())
    print(detail, flush=True)
    try:
        mode = "w" if os.path.exists(DIAGNOSTIC_LOG_PATH) and os.path.getsize(DIAGNOSTIC_LOG_PATH) > 262144 else "a"
        with open(DIAGNOSTIC_LOG_PATH, mode, encoding="utf-8") as stream:
            stream.write(time.strftime("%Y-%m-%d %H:%M:%S ") + detail + "\n")
    except Exception as save_error:
        print("[SERVICE] fault log unavailable:", save_error)


def encode_set(mode, station=0, target=0, hz=20, sequence=0, flags=0):
    return make_frame(SET, sequence, target, False, bytes((mode, station, hz, flags)))


def encode_ring_count(count, trusted, station, set_sequence, image_sequence):
    if not (isinstance(count,int) and count>=0 and 0<=station<=6 and 0<=set_sequence<=15):
        raise ValueError('Invalid ring count fields')
    return make_frame(RING_COUNT,image_sequence,2,trusted,bytes((min(count,3),station,set_sequence,0)))


def encode_ring_axis(measurement, image_sequence):
    """0A为原始图像行方向，绝不冒充尚未标定的05车体航向误差。"""
    payload=bytes(4)
    valid=False
    try:
        if measurement is not None:
            angle=float(measurement['degrees'])
            quality=measurement['confidence']
            source=measurement['source']
            if (math.isfinite(angle) and -90<=angle<90 and type(quality) is int
                    and 0<=quality<=100 and type(source) is int and source==3):
                scaled=angle*100
                quantized=int(scaled+.5) if scaled>=0 else -int(-scaled+.5)
                if quantized==9000:
                    quantized=-9000  # 无向直线量化也遵守[-90,90)。
                payload=struct.pack('<hBB',quantized,quality,source)
                valid=True
    except (KeyError,TypeError,ValueError,OverflowError):
        pass
    return make_frame(RING_AXIS,image_sequence,2,valid,payload)


def encode_plate_scene(scene, trusted, station, set_sequence, image_sequence):
    if not (isinstance(scene,int) and scene in (0,1,2,3) and 0<=station<=6 and 0<=set_sequence<=15):
        raise ValueError('Invalid plate scene fields')
    if not trusted or scene==PLATE_UNKNOWN:
        scene,trusted=PLATE_UNKNOWN,False
    return make_frame(PLATE_SCENE,image_sequence,0,trusted,bytes((scene,station,set_sequence,0)))


def validate_set(packet):
    mode, station, hz, flags = packet[3:7]
    target = packet[2] & 7
    if packet[2] & 0x80:
        return 6
    if mode in (4, 5):
        return 3  # 尚未标定毫米，不冒充ALIGN_MM。
    if mode not in (STOP, MATERIAL, RING, PLATE, PARK):
        return 1
    if target > 6 or (mode in (STOP, PLATE) and target != 0) or (mode == RING and target not in (0, 2)):
        return 2
    if station > 6:
        return 5
    if mode == PARK and (station!=5 or target!=2 or flags!=RING_AXIS_FLAG):return 6
    if flags:
        if mode == PARK:
            pass
        elif mode == RING:
            if flags & ~RING_FLAGS_MASK or station not in (2,3) or target!=2:
                return 6
        elif mode == MATERIAL:
            if flags != MATERIAL_CALIBRATION_FLAG or station != 1:
                return 6
        else:
            return 6
    if (hz != 0 if mode == STOP else not 1 <= hz <= 50):
        return 6
    return 0


class FrameParser:
    def __init__(self):
        self.buffer, self.started, self.errors = bytearray(), 0.0, 0

    def expire(self, now):
        if self.buffer and now - self.started >= PARTIAL_TIMEOUT:
            self.buffer.clear()
            self.errors += 1

    def feed(self, data, now):
        self.expire(now)
        frames = []
        for byte in data:
            if not self.buffer:
                if byte != 0x6B:
                    continue
                self.started = now
            self.buffer.append(byte)
            if len(self.buffer) < 8:
                continue
            if crc8(self.buffer[:7]) == self.buffer[7]:
                frames.append(bytes(self.buffer))
                self.buffer.clear()
            else:
                self.errors += 1
                del self.buffer[0]
                while self.buffer and self.buffer[0] != 0x6B:
                    del self.buffer[0]
                self.started = now
        return frames


class FrameWriter:
    """一组最多24字节观测、最多8个回执；组内不插控制帧，切换丢弃旧尾。"""
    def __init__(self, serial):
        self.serial = serial
        self.controls = []
        self.pending = b""
        self.original = None
        self.started = self.resume_at = 0.0
        self.errors = self.sent = 0
        self.last_heartbeat_sent = None
        self.max_heartbeat_gap = 0.0

    def _abandon(self, now, retry_control=True):
        if self.original and self.original[1] != "pixel" and retry_control:
            self.controls = self.controls[:7]
            self.controls.insert(0, self.original)
        self.pending, self.original = b"", None
        self.resume_at = now + RECOVERY_GAP

    def discard_observation(self, now):
        if self.original and self.original[1] == "pixel":
            self._abandon(now, False)

    def control(self, packet, kind="ack"):
        if kind == "heartbeat":
            self.controls = [item for item in self.controls if item[1] != "heartbeat"]
        if len(self.controls) >= 8:
            return False
        self.controls.append((packet, kind))
        return True

    def pump(self, now):
        if now < self.resume_at:
            return False
        if not self.pending:
            if not self.controls:
                return False
            self.original = self.controls.pop(0)
            self.pending, self.started = self.original[0], now
        try:
            if now - self.started >= PARTIAL_TIMEOUT:
                raise OSError("partial frame expired")
            written = self.serial.write(self.pending)
            if not isinstance(written, int) or not 0 <= written <= len(self.pending):
                raise OSError("invalid UART write count")
            self.pending = self.pending[written:]
            if self.pending:
                return False
            if self.original is not None and self.original[1] == 'heartbeat':
                if self.last_heartbeat_sent is not None:
                    self.max_heartbeat_gap=max(self.max_heartbeat_gap,now-self.last_heartbeat_sent)
                self.last_heartbeat_sent=now
            self.original = None
            self.sent += 1
            return True
        except Exception as exc:
            self.errors += 1
            self._abandon(now)
            if self.errors == 1:
                print("[SERVICE] UART TX:", exc)
            return False

    def ready(self, now):
        return not self.pending and not self.controls and now >= self.resume_at

    def observation(self, packet, now):
        if not isinstance(packet,bytes) or len(packet) not in (8,16,24):
            raise ValueError('Observation must contain a bounded one/two/three-frame group')
        if not self.ready(now):
            return False
        self.original = (packet, "pixel")
        self.pending, self.started = packet, now
        return self.pump(now)


class VisionService:
    def __init__(self, serial, backend, clock=time.monotonic):
        self.serial, self.backend, self.clock = serial, backend, clock
        self.parser, self.writer = FrameParser(), FrameWriter(serial)
        self.config = (STOP, 0, 0, 0, 0)  # mode/station/target/hz/flags
        self.epoch, self.image_seq = 0, 0
        self.set_sequence = 0
        self.last_request, self.last_ack, self.last_request_time = None, None, -math.inf
        self.next_heartbeat = self.next_observation = self.next_preview = 0.0
        self.last_pixel_state = None
        self.rx_errors = 0
        self.next_error_log = 0.0
        self.next_preview_error_log = 0.0
        self.preview_errors = 0
        self.next_loop_log = 0.0

    def pump_controls(self):
        now=self.clock()
        self.writer.pump(now)
        if now >= self.next_heartbeat:
            self.writer.control(make_frame(HEARTBEAT,0,0,True,
                bytes((PROTOCOL_VERSION,self.config[0],self.config[1],0))),'heartbeat')
            self.next_heartbeat=now+HEARTBEAT_PERIOD
        self.writer.pump(self.clock())

    def request(self, packet, now):
        if packet[1] != SET:
            return
        if packet == self.last_request and now - self.last_request_time <= 2.0:
            self.writer.control(self.last_ack)
            return
        # 有界控制队列；洪泛请求不能让设备积压控制数据。
        if len(self.writer.controls) >= 7:
            return
        result = validate_set(packet)
        desired = tuple(packet[3:5]) + (packet[2] & 7, packet[5], packet[6])
        if result == 0:
            print("[SERVICE SET] seq=%d old=%s requested=%s" % ((packet[2] >> 3) & 15, self.config, desired), flush=True)
            self.writer.controls = [item for item in self.writer.controls if item[1] != "heartbeat"]
            self.writer.discard_observation(now)
            old = self.config
            try:
                self.backend.configure(desired)
            except Exception as exc:
                result = 7
                report_fault(self.backend, getattr(self.backend, "stage", "configure"), exc)
                # 尽力恢复旧资源；即便恢复失败，旧配置下也只会输出无效位置。
                try:
                    self.backend.configure(old)
                except Exception as restore:
                    print("[SERVICE] restore failed:", restore)
            else:
                self.config = desired
                self.set_sequence = (packet[2] >> 3) & 15
            self.next_preview = 0.0
            self.epoch += 1
            self.last_pixel_state = None
            self.next_observation = 0.0
        seq, target = (packet[2] >> 3) & 15, packet[2] & 7
        reply = make_frame(ACK, seq, target, result == 0,
                           bytes((SET, result, self.config[0], self.config[1])))
        self.writer.control(reply)
        self.last_request, self.last_ack = packet, reply
        self.last_request_time = self.clock()

    def poll_rx(self, now):
        try:
            data = self.serial.read(256, timeout=0)
        except Exception as exc:
            self.rx_errors += 1
            if self.rx_errors == 1:
                print("[SERVICE] UART RX:", exc)
            return
        for packet in self.parser.feed(data or b"", now):
            self.request(packet, self.clock())

    def tick(self):
        loop_start=self.clock()
        now = self.clock()
        self.poll_rx(now)
        now = self.clock()
        self.parser.expire(now)
        self.pump_controls()
        # 不在ACK/旧控制半帧前发布新配置位置；先完成所有控制帧。
        self.writer.pump(self.clock())
        mode, station, target, hz, flags = self.config
        if mode == STOP or not self.writer.ready(self.clock()):
            return False
        epoch = self.epoch
        try:
            observation = self.backend.observe(self.clock())
        except Exception as exc:
            if self.clock() >= self.next_error_log:
                report_fault(self.backend, getattr(self.backend, "stage", "observe"), exc)
                self.next_error_log = self.clock() + 2.0
            invalidate = getattr(self.backend, "invalidate", None)
            if invalidate is not None:
                invalidate()
            observation = {"coordinates": None, "id": target if mode not in (RING,PARK) else 2, "scale": 1.0}
        if self.clock()-loop_start > .5:
            # 主控按接收时刻计算像素年龄，不能把处理了一秒的旧画面伪装成新测量。
            observation=dict(observation,coordinates=None,ring_count=0,ring_count_trusted=False,
                plate_scene=PLATE_UNKNOWN,plate_scene_trusted=False)
        sequence = self.image_seq
        # SET可能在一轮检测期间到达：发送/显示之前重新读取，丢弃旧模式结果。
        self.poll_rx(self.clock())
        if epoch != self.epoch or not self.writer.ready(self.clock()):
            return False
        self.pump_controls()  # 检测可能跨过心跳期限，先处理控制流再考虑预览。
        if not self.writer.ready(self.clock()):
            return False
        coords, identity = observation["coordinates"], observation["id"]
        if mode == MATERIAL and flags & MATERIAL_CALIBRATION_FLAG:
            coords = None  # Defense in depth: calibration can never publish grab coordinates.
        count, trusted = observation.get('ring_count',0), observation.get('ring_count_trusted',False)
        if mode in (RING,PARK) and (not trusted or count != 3):
            # 两环仅用于主控搜索，不发布身份不足的纠偏点；异常/未知计数立即失效。
            coords = None
        scene,scene_trusted=PLATE_UNKNOWN,False
        if mode == PLATE:
            reported=observation.get('plate_scene',PLATE_UNKNOWN)
            healthy=observation.get('plate_scene_trusted',False)
            if healthy and reported in (PLATE_SEARCH,PLATE_ACQUIRE,PLATE_FOUND):
                scene,scene_trusted=reported,True
            if scene!=PLATE_FOUND:
                if coords is not None:
                    scene,scene_trusted=PLATE_UNKNOWN,False  # 位置与状态矛盾时不授权搜索。
                coords=None
        if identity == 0 and mode != PLATE:
            packet = make_frame(MODE_TYPES[mode], sequence, 0, False, bytes(4))
        else:
            packet = encode_pixel_frame(MODE_TYPES[mode], identity, coords, sequence, observation["scale"])
        state = (packet[1], identity, bool(packet[2] & 0x80))
        if mode in (RING,PARK):
            count_packet=encode_ring_count(count,trusted,station,self.set_sequence,sequence)
            if flags & RING_AXIS_FLAG:
                axis=observation.get('ring_axis') if trusted and count==3 and packet[2]&0x80 else None
                axis_packet=encode_ring_axis(axis,sequence)
                packet+=axis_packet
                state+=(bool(axis_packet[2]&0x80),)
            packet += count_packet
            state += (count,trusted)
        elif mode == PLATE:
            if scene==PLATE_FOUND and not (packet[2]&0x80):
                scene,scene_trusted=PLATE_UNKNOWN,False
            packet+=encode_plate_scene(scene,scene_trusted,station,self.set_sequence,sequence)
            state+=(scene,scene_trusted)
        now = self.clock()
        if state != self.last_pixel_state:
            self.writer.discard_observation(now)
        if self.writer.ready(now) and (state != self.last_pixel_state or now >= self.next_observation):
            self.writer.observation(packet, now)
            # 按实际提交的观测组递增；限流跳过的图像不消耗SEQ，避免模16后重复旧组。
            self.image_seq = (self.image_seq + 1) & 15
            self.last_pixel_state = state
            self.next_observation = now + 1.0 / hz
        if PREVIEW_ENABLED and self.writer.ready(now) and now >= self.next_preview:
            slow_detection=now-loop_start >= .15
            try:
                self.backend.preview(self.config, observation)
            except Exception as exc:
                # ACK确认只代表配置准备好；预览错误不能退出UART/识别服务。
                self.preview_errors += 1
                if self.clock() >= self.next_preview_error_log:
                    report_fault(self.backend, getattr(self.backend, "stage", "preview"), exc)
                    self.next_preview_error_log = self.clock() + DIAGNOSTIC_PERIOD_S
                fallback = getattr(self.backend, "preview_failure", None)
                if fallback is not None:
                    try:
                        fallback()
                    except Exception as fallback_error:
                        if self.clock() >= getattr(self, "next_fallback_log", 0.0):
                            report_fault(self.backend, "display_fallback", fallback_error)
                            self.next_fallback_log = self.clock() + DIAGNOSTIC_PERIOD_S
            self.next_preview = self.clock() + (1.0 if slow_detection else 1.0 / PREVIEW_HZ)
            self.poll_rx(self.clock())
            self.pump_controls()
        if self.clock() >= self.next_loop_log:
            print('[SERVICE LOOP] mode=%d total_ms=%.1f HB_gap_max_ms=%.1f TXerr=%d' % (
                self.config[0],1000*(self.clock()-loop_start),1000*self.writer.max_heartbeat_gap,self.writer.errors),flush=True)
            self.next_loop_log=self.clock()+DIAGNOSTIC_PERIOD_S
        return True


def detector_module(name):
    module_name = "service_detector_" + name
    if module_name in sys.modules:
        return sys.modules[module_name]
    if name in _BUNDLED_DETECTORS:
        module = types.ModuleType(module_name)
        module.__file__ = name + ".py"
        exec(compile(_BUNDLED_DETECTORS[name], module.__file__, "exec"), module.__dict__)
    else:
        paths = {"material": "maixcam_material/main.py", "plate": "maixcam_material/plate_tangent.py",
                 "ring": "maixcam_ring/middle_ring.py"}
        spec = importlib.util.spec_from_file_location(module_name, Path(__file__).resolve().parent.parent / paths[name])
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    sys.modules[module_name] = module
    return module


def open_calibration_touch():
    from maix import touchscreen
    return touchscreen.TouchScreen()


class MaterialCalibrationUI:
    """Original Color/View/Learn workflow with three-point staged LAB calibration."""
    POINTS_REQUIRED=3
    HEADER_HEIGHT=78
    def __init__(self,module,samples,touch,color_id=1):
        self.module,self.samples,self.touch=module,samples,touch
        self.color_id=color_id or 1
        self.controller=module.TouchButtons()
        self.raw_view=True
        self.learning=False
        self.pending_points=[];self.pending_values=[]
        self.dirty=self.done=False
        self.message='View to compare; Learn to sample'
        self.last_lab=None

    @staticmethod
    def buttons(width,height):
        step=width//5
        return {name:(i*step+4,height-40,step-8,36)
                for i,name in enumerate(('color','view','learn','save','exit'))}

    def save(self):
        if self.learning:
            self.message='Finish 3 points or Cancel'
            return False
        if not self.dirty:
            self.message='No new samples; Learn 3 points'
            return True
        try:
            self.module.save_samples(self.samples)
        except (OSError,ValueError,TypeError) as exc:
            self.message='SAVE FAIL; see console'
            print('[MATERIAL CAL] save failed:',exc,flush=True)
            return False
        self.dirty=False;self.message='Saved; Color / Learn for next'
        return True

    def sample(self,frame,x,y):
        if not self.learning or self.done:return
        if not math.isfinite(x) or not math.isfinite(y):return
        value=self.module.sample_lab(frame,x,y)
        try:
            self.module.valid_samples({str(self.color_id):[value]})
        except ValueError:
            self.message='BAD LAB; tap clean surface'
            return
        if self.color_id in (3,6) and (value[0]<5 or value[0]>95 or value[2]>=-3):
            self.message='Avoid glare/dark/background'
            return
        self.pending_points.append((x,y));self.pending_values.append(value)
        self.last_lab=value
        count=len(self.pending_values)
        self.message='POINT %d/3; tap next surface' % count
        print('[MATERIAL CAL] %s point=%d/3 LAB=%s' % (self.module.COLORS[self.color_id-1][1],count,value),flush=True)
        if count==self.POINTS_REQUIRED:
            self.samples[str(self.color_id)]=list(self.pending_values)
            self.dirty=True;self.learning=False
            threshold,status=self.module.calibrated_threshold(self.module.COLORS[self.color_id-1],self.samples)
            self.message='3/3 READY; View then Save'
            print('[MATERIAL CAL] threshold=%s status=%s (RAM, not saved)' % (threshold,status),flush=True)

    def action(self,name,frame,point=None):
        if self.done:return
        if name=='color':
            self.color_id=self.color_id%6+1;self.last_lab=None
            self.learning=False;self.pending_points=[];self.pending_values=[]
            self.message='View to compare; Learn to sample'
        elif name=='view':
            if self.learning:self.message='Finish 3 points or Cancel'
            else:self.raw_view=not self.raw_view
        elif name=='learn':
            self.learning=not self.learning
            self.pending_points=[];self.pending_values=[];self.last_lab=None
            if self.learning:
                self.raw_view=True;self.message='Tap 3 different surface points'
            else:self.message='Canceled; prior samples kept'
        elif name=='surface' and point is not None:
            self.sample(frame,*point)
        elif name=='save':
            self.save()
        elif name=='exit':
            if self.learning:
                self.message='Finish 3 points or Cancel'
                return
            if not self.dirty or self.save():
                self.done=True;self.message='CAL DONE; send arm color stop'
                print('[MATERIAL CAL] EXIT; waiting STM32 STOP',flush=True)

    def update(self,frame,sw,sh,now):
        if self.done:return
        fw,fh=frame.width(),frame.height()
        buttons=self.buttons(fw,fh)
        buttons['surface']=(0,self.HEADER_HEIGHT,fw,fh-40-self.HEADER_HEIGHT)
        geometry=self.module.fit_geometry(fw,fh,sw,sh)
        for _ in range(16):
            if not self.touch.available():break
            tx,ty,pressed=self.touch.read0()
            x,y=self.module.frame_point(tx,ty,fw,fh,geometry)
            name=self.controller.update(x,y,pressed,now,buttons)
            if name=='surface':self.action('surface',frame,(x,y))
            elif name:self.action(name,frame)
            if self.done:break

    def draw(self,frame,image):
        color=self.module.COLORS[self.color_id-1]
        threshold,status=self.module.calibrated_threshold(color,self.samples)
        count=len(self.samples.get(str(self.color_id),[]))
        width,height=frame.width(),frame.height()
        frame.draw_rect(0,0,width,self.HEADER_HEIGHT,image.COLOR_BLACK,thickness=-1)
        frame.draw_string(4,2,'%s L:%d..%d A:%d..%d' % (color[1],*threshold[:4]),image.COLOR_WHITE)
        frame.draw_string(4,20,'B:%d..%d %s' % (*threshold[4:],status),image.COLOR_WHITE)
        label='CAL DONE / WAIT STOP' if self.done else '%s %s %s NO GRAB' % (
            'LEARN' if self.learning else 'CHECK','RGB' if self.raw_view else 'MASK',
            '%d/3'%len(self.pending_values) if self.pending_values or self.learning else 'N:%d'%count)
        frame.draw_string(4,38,label,image.COLOR_WHITE)
        frame.draw_string(4,57,self.message,image.COLOR_WHITE)
        for index,(x,y) in enumerate(self.pending_points):
            x,y=round(x),round(y)
            frame.draw_rect(x-4,y-4,9,9,image.COLOR_WHITE)
            frame.draw_string(x+5,y-5,str(index+1),image.COLOR_WHITE)
        for name,(x,y,w,h) in self.buttons(width,height).items():
            frame.draw_rect(x,y,w,h,image.COLOR_BLACK,thickness=-1)
            frame.draw_rect(x,y,w,h,image.COLOR_WHITE)
            label='Cancel' if name=='learn' and self.learning else name.title()
            frame.draw_string(x+3,y+9,label,image.COLOR_WHITE)

    def render(self,frame,image):
        # Never overwrite the RGB sampling source with mask pixels or UI text.
        shown=frame.copy()
        if not self.raw_view and not self.learning:
            threshold,_=self.module.calibrated_threshold(self.module.COLORS[self.color_id-1],self.samples)
            shown.binary([threshold],copy=False)
            if shown.format()!=image.Format.FMT_RGB888:shown=shown.to_format(image.Format.FMT_RGB888)
        self.draw(shown,image)
        return shown

    def close(self):
        touch,self.touch=self.touch,None
        close=getattr(touch,'close',None)
        if close is not None:close()


class MaixBackend:
    def __init__(self, camera, image, display):
        self.camera, self.image, self.display = camera, image, display
        self.cam, self.view, self.displayed, self.displayed_array = None, None, None, None
        self.config = (STOP, 0, 0, 0, 0)
        self.selected_color, self.samples = 0, {}
        self.material_target = None
        self.previous_time, self.fps, self.error_log_at = 0.0, 0.0, 0.0
        self.status = "Waiting for STM32 SET"
        self.stage = "idle"
        self.next_diagnostic = 0.0
        self.frames = self.preview_count = 0
        self.last_camera_ms = self.last_detect_ms = self.last_preview_ms = 0.0
        self.camera_info, self.array_info = "not_open", "none"
        self.first_frame_pending = False
        self.light = None  # run_device attaches the single initialized PWM handle.
        self.material_calibration = None

    def close_calibration(self):
        calibration,self.material_calibration=self.material_calibration,None
        if calibration is not None:calibration.close()

    def configure_light(self, mode):
        if self.light is None:
            return
        percent = {RING: RING_LIGHT_BRIGHTNESS_PERCENT,
                   MATERIAL: MATERIAL_LIGHT_BRIGHTNESS_PERCENT,
                   PLATE: PLATE_LIGHT_BRIGHTNESS_PERCENT}.get(mode, LIGHT_BRIGHTNESS_PERCENT)
        actual = self.light.set_brightness(percent)
        print('[LIGHT MODE] mode=%d brightness=%.1f%% PWM=%.1f%%' % (mode,percent,actual),flush=True)

    def show_message(self, message):
        if self.display is None:
            return
        # 显示独立RGB图，替换仍指向相机缓冲的上一次圆盘画面。
        shown = self.image.Image(PREVIEW_WIDTH, PREVIEW_HEIGHT, self.image.Format.FMT_RGB888)
        shown.draw_string(4, 8, "MODE:%d STATION:%d" % self.config[:2], self.image.COLOR_WHITE)
        shown.draw_string(4, 30, message, self.image.COLOR_WHITE)
        self.show_frame(shown)

    def show_frame(self, shown):
        # SDK的show返回错误码；native VO失败可能只打印日志而不抛Python异常。
        # 同一尺寸/格式避免切换时重建pool，仍必须检查错误码。
        if shown.format() != self.image.Format.FMT_RGB888:
            shown = shown.to_format(self.image.Format.FMT_RGB888)
        if (shown.width(), shown.height()) != (PREVIEW_WIDTH, PREVIEW_HEIGHT):
            shown = shown.resize(PREVIEW_WIDTH, PREVIEW_HEIGHT)
        self.stage = "display_show"
        result = self.display.show(shown, fit=self.image.Fit.FIT_CONTAIN)
        if result is not None and int(result) != 0:
            raise RuntimeError("display.show returned %s (native VO failure)" % result)
        self.displayed, self.displayed_array = shown, None

    def release_images(self):
        # 顺序：先释放借用numpy缓冲的Image，再释放缓冲；最后释放相机源图。
        self.displayed = None
        self.displayed_array = None
        self.view = None
        self.material_target = None

    def release_camera(self):
        self.close_calibration()
        self.release_images()
        gc.collect()  # 仅模式切换/关闭执行，识别热路径不运行GC。
        if self.cam is not None:
            old = self.cam
            self.cam = None
            try:
                old.close()
            finally:
                del old
                gc.collect()

    def reset_tracking(self):
        """新轮次清状态：旧锁定/模板/确认计数不可跨STOP或显式SET保留。"""
        self.release_images()
        self.close_calibration()
        self.selected_color = 0
        for name in ("ring_tracker", "ring_detector", "alignment", "plate_tracker"):
            self.__dict__.pop(name, None)
        self.previous_time, self.fps = 0.0, 0.0
        self.array_info = "none"
        self.first_frame_pending = False
        self.next_diagnostic = 0.0

    def configure(self, config):
        if config[0]==PARK:raise ValueError('Parking reference alignment requires PC backend')
        mode = config[0]
        if mode not in (STOP, MATERIAL, RING, PLATE):
            raise ValueError("Unsupported visual mode")
        # 相同请求字节重传在VisionService.request被拦住；走到这里的是新请求。
        # 相同参数的新SEQ也应重获目标，复用的是原生相机而不是旧跟踪器。
        self.stage = "switch_display"
        try:
            self.show_message("Switching to mode %d" % mode)
        except Exception as exc:
            report_fault(self, self.stage, exc)
        self.reset_tracking()
        self.stage = 'light_mode'
        self.configure_light(mode)
        if mode == STOP:
            # STOP仅停观测流；保留已打开通道，避免MMF全局释放破坏显示池。
            self.config, self.status = config, "Waiting for STM32 SET"
            try:
                self.stage = "stop_display"
                self.show_message(self.status)
            except Exception as exc:
                report_fault(self, self.stage, exc)
            print("[SERVICE RESET] STOP; locks/template/counts cleared; calibration kept", flush=True)
            return
        module = detector_module("material" if mode == MATERIAL else "plate" if mode == PLATE else "ring")
        if self.cam is None:
            fmt = self.image.Format.FMT_RGB888
            self.stage = "camera_open"
            self.cam = self.camera.Camera(CAPTURE_WIDTH, CAPTURE_HEIGHT, fmt, fps=CAPTURE_FPS)
            self.camera_info = "%dx%d fmt=%s persistent" % (CAPTURE_WIDTH, CAPTURE_HEIGHT, fmt)
            print("[SERVICE CAMERA] open " + self.camera_info, flush=True)
        else:
            self.stage = "camera_clear_buff"
            clear = getattr(self.cam, "clear_buff", None)
            if clear is not None:
                clear()
            else:
                print("[SERVICE] camera clear_buff unavailable; fresh-frame confirmation still required", flush=True)
        self.module, self.config, self.selected_color = module, config, 0
        if mode == MATERIAL:
            self.samples = module.load_samples()
            if config[4] & MATERIAL_CALIBRATION_FLAG:
                self.stage='material_calibration_touch'
                touch=open_calibration_touch()
                initial=(0,1,2,3,4,6,5)[config[2]] or 1
                self.material_calibration=MaterialCalibrationUI(module,self.samples,touch,initial)
        elif mode == PLATE:
            self.plate_tracker = module.PlateTracker()
        else:
            module.cv2.setNumThreads(1)
            self.ring_tracker, self.ring_detector = module.MiddleTracker(), module.RingDetector()
            self.ring_detector.require_scene = True
            self.ring_detector.allow_occupied = bool(config[4] & RING_OCCUPIED_FLAG)
            self.alignment = module.AlignmentState()
        self.previous_time, self.fps = 0.0, 0.0
        self.array_info, self.stage = "none", "await_frame"
        self.first_frame_pending = True
        print("[SERVICE RESET] mode=%d new acquisition; persistent camera kept" % mode, flush=True)
        print("[SERVICE MODE] ready config=%s camera=%s" % (self.config, self.camera_info), flush=True)

    def observe(self, now):
        mode, _, requested, _, flags = self.config
        module, image = self.module, self.image
        if self.cam is None:
            raise RuntimeError("camera unavailable")
        self.stage = "camera_read"
        camera_start = time.monotonic()
        # 显式超时，避免ACK已确认但驱动read无限阻塞后整个串口心跳也消失。
        frame = self.cam.read(block=True, block_ms=CAMERA_READ_TIMEOUT_MS)
        self.last_camera_ms = 1000 * (time.monotonic() - camera_start)
        if frame is None:
            raise RuntimeError("camera returned no frame")
        if mode != RING and (frame.width(), frame.height()) != (module.CAM_WIDTH, module.CAM_HEIGHT):
            # 物料阈值/尺寸约束、圆盘拟合及线上像素单位继续沿用320×240。
            self.stage = "rgb_processing_resize"
            frame = frame.resize(module.CAM_WIDTH, module.CAM_HEIGHT)
        fw, fh, sw, sh = frame.width(), frame.height(), self.display.width(), self.display.height()
        if self.first_frame_pending:
            print("[SERVICE FRAME] first mode=%d actual=%dx%d read_ms=%.1f" % (
                mode, fw, fh, self.last_camera_ms), flush=True)
            self.first_frame_pending = False
        scale = min(sw / fw, sh / fh)
        elapsed = now - self.previous_time
        if self.previous_time and elapsed > 0:
            instant = 1.0 / elapsed
            self.fps = instant if self.fps == 0 else .8 * self.fps + .2 * instant
        self.previous_time = now
        coords, identity = None, requested
        self.frames += 1
        detect_start = time.monotonic()
        if mode == MATERIAL and flags & MATERIAL_CALIBRATION_FLAG:
            self.stage='material_calibration'
            self.material_calibration.update(frame,sw,sh,now)
            selected=self.material_calibration.color_id
            self.selected_color,identity=selected,(0,1,2,3,4,6,5)[selected]
            self.material_target=None;self.view=frame
            self.status='CAL DONE / WAIT STOP' if self.material_calibration.done else 'MATERIAL CALIBRATION'
        elif mode == MATERIAL:
            self.stage = "material_detect"
            # 线上5浅蓝/6黑映射回原标定文件5黑/6浅蓝。
            inverse = (0, 1, 2, 3, 4, 6, 5)
            selected = inverse[requested] if requested else self.selected_color
            choices = [selected] if selected else list(range(1, 7))
            target, source, diagnostics = None, frame, {}
            for color_id in choices:
                color = module.COLORS[color_id - 1]
                threshold, calibration = module.calibrated_threshold(color, self.samples)
                if calibration == "OVERLAP":
                    continue
                sample = frame if len(choices) == 1 else frame.copy()
                target, source = module.detect_material_center(sample, threshold, image, diagnostics)
                if target is not None or selected:
                    selected = color_id
                    break
            if selected:
                self.selected_color, identity = selected, inverse[selected]
            if target is not None:
                coords = ((target[0] - fw/2) * scale, (fh/2 - target[1]) * scale)
                source.draw_circle(round(target[0]), round(target[1]), round(target[2]), image.COLOR_WHITE, thickness=1)
            self.status = "Material %s %s" % (module.COLORS[selected-1][1] if selected else "ANY", diagnostics.get("reason", "NO_TARGET"))
            self.material_target = target
            self.view = source
        elif mode == PLATE:
            self.stage = "plate_detect"
            diagnostics = {}
            raw, source = module.detect_plate_color(frame, image, diagnostics, False)
            target = self.plate_tracker.update(raw)
            if target is not None:
                coords = module.tangent_alignment_error(target, fw, fh, sw, sh)
            reason=diagnostics.get('reason')
            plate_scene,plate_healthy=PLATE_UNKNOWN,False
            if target is not None:
                if coords is not None and all(math.isfinite(v) for v in coords):
                    plate_scene,plate_healthy=PLATE_FOUND,True
                else:
                    coords=None
            elif raw is not None and reason=='EDGE_OK':
                plate_scene,plate_healthy=PLATE_ACQUIRE,True
            elif raw is None and reason=='AMBIGUOUS_EDGE':
                plate_scene,plate_healthy=PLATE_ACQUIRE,True
            elif raw is None and reason=='NO_EDGE_ARC':
                plate_scene,plate_healthy=PLATE_SEARCH,True
            identity, self.status = 0, "Plate " + diagnostics.get("reason", "SEARCH")
            self.array_info = "%s Ctr=%d Arc=%d Prop=%d Cand=%d Scene=%d Healthy=%d" % (
                diagnostics.get("search", "?"), diagnostics.get("contours", 0),
                diagnostics.get("arcs", 0), diagnostics.get("proposals", 0), diagnostics.get("count", 0),plate_scene,plate_healthy)
            module.draw_plate_overlay(source, target, image)
            self.view = source
        else:
            self.stage = "ring_image2cv"
            gray = image.image2cv(frame, ensure_bgr=False, copy=False)
            self.array_info = "shape=%s dtype=%s profile=%s" % (gray.shape, gray.dtype,
                'OCCUPIED' if flags & RING_OCCUPIED_FLAG else 'EMPTY')
            if gray.ndim == 3 and gray.shape[2] == 1:
                gray = gray[:, :, 0]
            elif gray.ndim == 3 and gray.shape[2] == 3:
                # 固件返回RGB三通道时必须转亮度，不能悄悄取第一个通道当灰度。
                gray = module.cv2.cvtColor(gray, module.cv2.COLOR_RGB2GRAY)
            if gray.ndim != 2 or gray.dtype != module.np.uint8:
                raise ValueError("Unexpected ring camera array: " + self.array_info)
            if gray.shape != (fh, fw):
                raise ValueError("Ring array/frame geometry mismatch: " + self.array_info)
            self.stage = "ring_detect"
            groups, triple, binary, status = self.ring_detector.process(gray, self.ring_tracker, now)
            stamp = time.monotonic()
            if self.ring_detector.identity_rejected and triple is None:
                self.ring_tracker.reset()
            observed = self.ring_tracker.update(groups, triple, stamp, status)
            self.ring_detector.accept(observed, triple, gray, stamp)
            if observed is not None:
                coords = module.screen_error(observed["position"], (fw, fh), (sw, sh))
            self.status = "ROI_MIDDLE" if self.ring_detector.used_roi and observed else self.ring_tracker.status
            if observed and observed['ring'].get('weak_recovered'):
                self.status = 'WEAK_MIDDLE'
            elif observed and observed['ring'].get('occluded'):
                self.status = 'OCCUPIED_MIDDLE'
            count_trusted = getattr(self.ring_detector,'scene_trusted',False)
            if not count_trusted or len(groups) != 3:
                coords = None
                self.status = "COUNT_UNKNOWN" if not count_trusted else "COUNT_%d_NEED_THREE" % min(len(groups),3)
            ring_axis=None
            if flags & RING_AXIS_FLAG:
                if coords is not None and count_trusted:
                    ring_axis=module.ring_axis_measurement(groups,triple,observed)
                self.status += ' AX:--' if ring_axis is None else ' AX:%+.2f Q:%d' % (ring_axis['degrees'],ring_axis['confidence'])
            identity = 2
            raw_coords = module.screen_error(observed["ring"]["center"], (fw, fh), (sw, sh)) if observed and coords is not None else None
            self.view = (frame, gray, groups, observed, coords, self.alignment.update(raw_coords))
            self.array_info += " mean=%.1f range=%d..%d G=%d W=%d O=%d" % (
                float(module.cv2.mean(gray)[0]), int(gray.min()), int(gray.max()), len(groups),
                sum(g.get('weak_recovered',False) for g in groups),
                sum(g.get('occluded',False) for g in groups)) if time.monotonic() >= self.next_diagnostic else ""
        self.last_detect_ms = 1000 * (time.monotonic() - detect_start)
        self.stage = "observe_ok"
        stamp = time.monotonic()
        if stamp >= self.next_diagnostic:
            print("[SERVICE PERF] mode=%d frames=%d previews=%d C=%.1f D=%.1f V=%.1f ms status=%s %s" % (
                mode, self.frames, self.preview_count, self.last_camera_ms, self.last_detect_ms,
                self.last_preview_ms, self.status, self.array_info), flush=True)
            sampling=getattr(getattr(module,'occupied_sample_group',None),'last_profile',None) if mode==RING and flags & RING_OCCUPIED_FLAG else None
            if sampling:
                print('[ARC sample last] map=%.1f signal=%.1f fit=%.1f ms' % (
                    sampling['map'],sampling['signal'],sampling['fit']),flush=True)
            batch=getattr(getattr(module,'occupied_sample_batch',None),'last_profile',None) if mode==RING and flags & RING_OCCUPIED_FLAG else None
            if batch:
                print('[ARC batch last] prep_map=%.1f ms rays=%d' % (batch['ms'],batch['rays']),flush=True)
            self.next_diagnostic = stamp + DIAGNOSTIC_PERIOD_S
        result = {"coordinates": coords, "id": identity, "scale": scale}
        if mode == RING:
            result.update(ring_count=min(len(groups),3),ring_count_trusted=count_trusted)
            if flags & RING_AXIS_FLAG:
                result['ring_axis']=ring_axis
        elif mode == PLATE:
            result.update(plate_scene=plate_scene,plate_scene_trusted=plate_healthy)
        return result

    def invalidate(self):
        self.view = None
        self.material_target = None
        self.status = "Visual unavailable"
        if self.config[0] == RING and hasattr(self, "ring_tracker"):
            self.ring_tracker.reset()
            self.alignment.count = 0
        elif self.config[0] == PLATE and hasattr(self, "plate_tracker"):
            self.plate_tracker.reset()


    def preview(self, config, observation):
        preview_start = time.monotonic()
        if self.view is None:
            self.stage = "display_status"
            self.show_message("X:-- Y:-- " + self.status)
            return
        if config[0]==MATERIAL and config[4] & MATERIAL_CALIBRATION_FLAG:
            self.show_frame(self.material_calibration.render(self.view,self.image))
            self.preview_count+=1
            self.last_preview_ms=1000*(time.monotonic()-preview_start)
            self.stage='preview_ok'
            return
        if config[0] == RING:
            self.stage = "ring_annotate"
            frame, gray, groups, observed, error, aligned = self.view
            rgb = self.module.annotate(gray, groups, observed, status=self.status, error=error, aligned=aligned, rgb=True)
            if config[4] & RING_OCCUPIED_FLAG:
                self.module.cv2.rectangle(rgb,(0,35),(rgb.shape[1]-1,53),(0,0,0),-1)
                self.module.cv2.putText(rgb,'FPS:%.1f D:%.0fms V:%.0fms TX:%s'%(self.fps,self.last_detect_ms,self.last_preview_ms,
                    'OK' if observation.get('coordinates') is not None else '--'),
                    (4,48),self.module.cv2.FONT_HERSHEY_SIMPLEX,.33,(255,255,255),1)
            self.stage = "ring_cv2image"
            # 预览独立持有图像内存，模式切换不再依赖借用numpy的生命周期。
            shown = self.image.cv2image(rgb, bgr=False, copy=True)
        else:
            shown = self.view
            if config[0] == MATERIAL:
                # Convert the binary preview to RGB before colored annotation.
                # Draw after detection and only at preview rate, not every frame.
                if shown.format() != self.image.Format.FMT_RGB888:
                    shown = shown.to_format(self.image.Format.FMT_RGB888)
                self.module.draw_pick_window(shown, self.material_target,
                                             self.selected_color, self.image)
            coords = observation["coordinates"]
            label = "X:-- Y:--" if coords is None else "X:%+.1f Y:%+.1f" % coords
            if config[0] == MATERIAL:
                target = self.material_target
                label = "PX dx:-- dy:--" if target is None else "PX dx:%+.1f dy:%+.1f" % (
                    target[0]-shown.width()/2, target[1]-shown.height()/2)
            if config[0] == PLATE and coords is not None:
                label="ERR X:%+.1f Y:%+.1f" % coords
            shown.draw_string(4, 4, label, self.image.COLOR_WHITE)
            shown.draw_string(4, 24, "%s FPS:%.1f" % (self.status, self.fps), self.image.COLOR_WHITE)
        self.show_frame(shown)
        self.preview_count += 1
        self.last_preview_ms = 1000 * (time.monotonic() - preview_start)
        self.stage = "preview_ok"

    def preview_failure(self):
        self.show_message("Preview error; check console")

    def close(self):
        self.release_camera()


def run_device():
    from maix import app, camera, display, image, err
    from maix.peripheral import uart, pinmap, gpio
    serial = disp = backend = light = None
    try:
        light = init_light_on(gpio, pinmap, err)
        # 等OpenCV/算法初始化完成后才声明服务就绪，避免首次SET加载库耗尽ACK重试。
        for name in ("material", "plate", "ring"):
            detector_module(name)
        for pin, function in (("A16", "UART0_TX"), ("A17", "UART0_RX")):
            err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping: " + pin)
        serial = uart.UART("/dev/ttyS0", 115200, databits=uart.BITS.BITS_8,
                           parity=uart.PARITY.PARITY_NONE, stopbits=uart.STOP.STOP_1)
        disp = display.Display()
        try:
            display.set_trans_image_quality(75)
        except (AttributeError, RuntimeError):
            pass
        backend = MaixBackend(camera, image, disp)
        backend.light = light
        standby = image.Image(PREVIEW_WIDTH, PREVIEW_HEIGHT, image.Format.FMT_RGB888)
        standby.draw_string(12, 90, "Waiting for STM32 SET", image.COLOR_WHITE)
        backend.show_frame(standby)
        service = VisionService(serial, backend)
        last_mode = STOP
        print("[SERVICE] build=%s ready; STOP=0 MAT=2 RING=3 PLATE=8; protocol8" % SERVICE_BUILD, flush=True)
        while not app.need_exit():
            observed = service.tick()
            if service.config[0] == STOP and last_mode != STOP:
                try:
                    backend.show_frame(standby)
                except Exception as exc:
                    report_fault(backend, "standby_show", exc)
            last_mode = service.config[0]
            if not observed:
                serial.available(timeout=10)  # 有界UART事件等待，不忙等或sleep。
    finally:
        light_off_on_exit(light)
        if backend is not None:
            try:
                backend.close()
            except Exception as exc:
                print("[SERVICE] camera close:", exc)
        for device in (serial, disp):
            if device is not None:
                try:
                    device.close()
                except Exception as exc:
                    print("[SERVICE] close:", exc)


if __name__ == "__main__":
    run_device()
