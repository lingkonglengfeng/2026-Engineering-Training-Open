"""MaixCAM：物料二值化定位与现场颜色取样。

屏幕中心为 (0,0)，右正、上正。物料模式默认黑白图；圆盘使用独立程序或统一服务。
Learn 进入彩色取样画面；分别取样 BLUE/LIGHTBLUE 后生成互斥阈值。
Color 只循环六种物料颜色，发送物料圆心。
"""
import json
import math
import os
import time
import traceback

CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 320, 240, 30
# Preview-only image-pixel window (dx right, dy down); shifted right10 by user.
# This is local configuration; UART SET/MAT_PX does not synchronize window edits.
PICK_WINDOW = (10, -30, 40, 40)


def pick_window_geometry(width, height, window=None):
    gx, gy, wx, wy = PICK_WINDOW if window is None else window
    sx, sy = width / CAM_WIDTH, height / CAM_HEIGHT
    cx, cy = (CAM_WIDTH / 2 + gx) * sx, (CAM_HEIGHT / 2 + gy) * sy
    return cx, cy, wx * sx, wy * sy


def pick_window_state(target, color_id, width, height, window=None):
    if target is None:
        return "NO TARGET"
    if color_id != 1:
        return "OTHER COLOR"
    cx, cy, wx, wy = pick_window_geometry(width, height, window)
    return "IN" if abs(target[0] - cx) <= wx and abs(target[1] - cy) <= wy else "OUT"


def draw_pick_window(source, target, color_id, image, label_y=None, window=None):
    """Draw only after detection; IN is a visual hint, not a grab/ACK event."""
    width, height = source.width(), source.height()
    local = PICK_WINDOW if window is None else window
    cx, cy, wx, wy = pick_window_geometry(width, height, local)
    state = pick_window_state(target, color_id, width, height, local)
    color = image.COLOR_GREEN if state == "IN" else image.COLOR_YELLOW
    # An inclusive pixel boundary at x=200 needs width=81, not width=80.
    left, right = max(0, round(cx-wx)), min(width-1, round(cx+wx))
    top, bottom = max(0, round(cy-wy)), min(height-1, round(cy+wy))
    if left <= right and top <= bottom:
        source.draw_rect(left, top, right-left+1, bottom-top+1, color, thickness=2)
    x, y = round(cx), round(cy)
    source.draw_line(x-6, y, x+6, y, color, thickness=2)
    source.draw_line(x, y-6, x, y+6, color, thickness=2)
    if target is not None:
        tx, ty = round(target[0]), round(target[1])
        source.draw_line(x, y, tx, ty, color)
        source.draw_circle(tx, ty, 4, color, thickness=2)
    if label_y is None:
        label_y = height - 54
    gx, gy, wx, wy = local
    source.draw_string(4, label_y, "WIN %g,%g +/-%g,%g LOCAL" % (gx, gy, wx, wy), color)
    source.draw_string(4, label_y+17, "RED %s (VIEW ONLY)" % state, color)
    return state


# MaixCAM 的 A16/TX、A17/RX（UART0），与主控采用 115200 8N1。
UART_ENABLED = True
UART_PORT = "/dev/ttyS0"
UART_BAUDRATE = 115200
UART_PIN_FUNCTIONS = {"A16": "UART0_TX", "A17": "UART0_RX"}
UART_SEND_HZ = 20
UART_TX_STALL_S = 0.20
COLORS = (
    (1, "RED", [15, 85, 25, 127, -10, 100]),
    (2, "YELLOW", [35, 100, -25, 25, 30, 127]),
    # 从用户彩色截图估算的调试起点；按 A 分开蓝紫/青蓝，不靠 L 下限。
    (3, "BLUE", [5, 100, 15, 90, -128, -18]),
    (4, "GREEN", [15, 90, -128, -20, -20, 80]),
    (5, "BLACK", [0, 25, -25, 25, -25, 25]),
    (6, "LIGHTBLUE", [5, 100, -60, 14, -90, -10]),
)
MIN_RADIUS_PX, MAX_RADIUS_PX = 5, 110
MIN_BLOB_PIXELS = 100
MIN_ASPECT_RATIO = 0.55
MIN_DENSITY, MAX_DENSITY = 0.35, 0.96
MAX_BLOB_AREA_FRACTION = 0.65
SHAPE_REFINEMENT_ENABLED = True
MAX_SHAPE_CANDIDATES = 8
MIN_CIRCULARITY, MIN_CONVEXITY = 0.48, 0.82
BUTTON_DEBOUNCE_S = 0.15
HEADER_HEIGHT, FOOTER_HEIGHT = 70, 40
SAMPLE_SIZE = 12
LIVE_LAB_INTERVAL_S = 0.25
MAX_SAMPLES = 6
CALIBRATION_PATH = os.path.expanduser("~/maixcam_material_colors.json")
PLATE_WHITE_LAB = [60, 100, -14, 14, -14, 14]  # 仅兼容旧标定存储；PLATE 检测不使用。
MODES = COLORS  # 物料程序只循环六种颜色；圆盘使用独立程序或统一服务模式8。
PLATE_EDGE_LOW = 4
PLATE_EDGE_HIGH = 12
PLATE_NEUTRAL_CHROMA_MAX = 35
PLATE_MAX_JOIN_GAP_FRACTION = 0.20
PLATE_MIN_INLIER_RATIO = 0.60
PLATE_MIN_ARC_DEG = 35.0
PLATE_CONFIRM_FRAMES = 3
PLATE_MAX_Y_STEP_PX = 24.0
PLATE_SEARCH_TOP_FRACTION = 0.22
PLATE_SEARCH_BOTTOM_FRACTION = 0.92




# LIGHT_GPIO_BEGIN
LIGHT_PIN = "A19"
LIGHT_GPIO = "GPIOA19"
LIGHT_OFF_LEVEL = 0  # 按高电平使能灯板；低电平使能板需改为1。
LIGHT_PWM_ENABLED = True  # 由各入口配置 GPIO/PWM；亮度为灯板占空比。
LIGHT_PWM_ID, LIGHT_PWM_FREQ_HZ = 7, 20000
LIGHT_BRIGHTNESS_PERCENT = 20.0


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
# LIGHT_GPIO_END# UART_PROTOCOL8_BEGIN
import math
import struct


PROTOCOL8_HEAD = 0x6B
MAT_PX, RING_PX, PLATE_PX = 0x02, 0x03, 0x09
# UI/比赛编号：5黑、6浅蓝；工程线协议：5浅蓝、6黑。
PROTOCOL_COLOR_IDS = (0, 1, 2, 3, 4, 6, 5)
PROTOCOL8_PARTIAL_TIMEOUT_S = 0.050
PROTOCOL8_RECOVERY_S = 0.055


def crc8(data):
    """CRC-8/SMBUS：poly=07 init=00 refin/refout=false xorout=00。"""
    result = 0
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = ((result << 1) ^ 0x07) & 0xFF if result & 0x80 else (result << 1) & 0xFF
    return result


def fixed10(value):
    if not math.isfinite(value):
        raise ValueError("Non-finite pixel measurement")
    scaled = value * 10
    if not math.isfinite(scaled):
        raise ValueError("Pixel measurement out of range")
    quantized = int(scaled + 0.5) if scaled >= 0 else -int(-scaled + 0.5)
    if not -32768 <= quantized <= 32767:
        raise ValueError("Pixel measurement out of range")
    return quantized


def make_frame(msg_type, sequence, target_id, valid, payload):
    if not (isinstance(sequence, int) and 0 <= sequence <= 15
            and isinstance(target_id, int) and 0 <= target_id <= 7
            and isinstance(msg_type, int) and 0 <= msg_type <= 255 and len(payload) == 4):
        raise ValueError("Invalid protocol8 fields")
    meta = (int(bool(valid)) << 7) | (sequence << 3) | target_id
    head = bytes((PROTOCOL8_HEAD, msg_type, meta)) + payload
    return head + bytes((crc8(head),))


def encode_pixel_frame(msg_type, target_id, screen_coordinates, sequence, screen_scale=1.0):
    """界面屏幕像素（右/上正）→ 相机图像像素（右/下正）。

    配置错误抛异常；测量超范围/非有限数按协议失效，避免冒充有效坐标。
    sequence 接受程序图像计数，线上只取低 4 位。
    """
    if not isinstance(sequence, int) or not 0 <= sequence <= 65535:
        raise ValueError("Invalid image sequence")
    if msg_type not in (MAT_PX, RING_PX, PLATE_PX):
        raise ValueError("Invalid position type")
    if ((msg_type == PLATE_PX and target_id != 0)
            or (msg_type != PLATE_PX and target_id not in range(1, 7))):
        raise ValueError("Invalid position target")
    if not math.isfinite(screen_scale) or screen_scale <= 0:
        raise ValueError("Invalid screen scale")
    valid = screen_coordinates is not None
    payload = bytes(4)
    if valid:
        try:
            x, y = screen_coordinates
            payload = struct.pack("<hh", fixed10(float(x) / screen_scale),
                                  fixed10(-float(y) / screen_scale))
        except (ValueError, OverflowError, TypeError):
            valid = False
    return make_frame(msg_type, sequence & 15, target_id, valid, payload if valid else bytes(4))


class Protocol8Sender:
    """有界短写缓存；失效优先；恢复时等待接收器半帧超时，不注入 CRLF。"""
    def __init__(self, serial, hz=20, screen_scale=1.0):
        if not math.isfinite(hz) or not 0 < hz <= 50:
            raise ValueError("Invalid UART send rate")
        if not math.isfinite(screen_scale) or screen_scale <= 0:
            raise ValueError("Invalid screen scale")
        self.serial, self.screen_scale = serial, screen_scale
        self.period, self.next_due = 1.0 / hz, 0.0
        self.sequence, self.sent, self.errors = 0, 0, 0
        self.last_state = None
        self.pending, self.pending_since, self.resume_at = b"", 0.0, 0.0

    def image_sequence(self):
        # 每次新图像调用 send，即使本帧被限频跳过，图像编号仍递增。
        result = self.sequence
        self.sequence = (self.sequence + 1) & 15
        return result

    def _abandon(self, now):
        self.pending = b""
        self.resume_at = now + PROTOCOL8_RECOVERY_S
        # 接收器不能把丢失前的半帧与新帧混合；不 sleep，不积压旧观测。
        self.last_state = None

    def _flush(self, now):
        try:
            if now - self.pending_since >= PROTOCOL8_PARTIAL_TIMEOUT_S:
                raise OSError("UART partial frame expired")
            written = self.serial.write(self.pending)
            if not isinstance(written, int) or not 0 <= written <= len(self.pending):
                raise OSError("Invalid UART write count")
            self.pending = self.pending[written:]
            if self.pending:
                return False
            self.sent += 1
            return True
        except Exception as exc:
            self.errors += 1
            self._abandon(now)
            if self.errors == 1:
                print("Protocol8 UART write failed:", exc)
            return False

    def send_packet(self, packet, state, now, force=False):
        if self.pending and (force or state != self.last_state):
            self._abandon(now)
        if now < self.resume_at:
            return False
        if self.pending and not self._flush(now):
            return False
        if not force and state == self.last_state and now < self.next_due:
            return False
        self.pending, self.pending_since, self.last_state = packet, now, state
        self.next_due = now + self.period
        return self._flush(now)
# UART_PROTOCOL8_END

def log_error(stage):
    """保留完整 Python 异常；MaixVision 控制台关闭后仍可在相机读取。"""
    detail = "%s [%s]\n%s" % (time.strftime("%Y-%m-%d %H:%M:%S"), stage,
                               traceback.format_exc())
    print(detail, flush=True)
    path = os.path.expanduser("~/maixcam_material_error.log")
    try:
        mode = "w" if os.path.exists(path) and os.path.getsize(path) > 262144 else "a"
        with open(path, mode, encoding="utf-8") as stream:
            stream.write(detail + "\n")
    except OSError as exc:
        print("Error log save failed:", exc, flush=True)


def encode_coordinate_frame(color_id, coordinates, sequence, screen_scale=1.0):
    """比赛/UI 颜色映射至工程 ID，发送 MAT_PX 图像像素误差。"""
    if color_id not in range(1, 7):
        raise ValueError("Invalid material color")
    return encode_pixel_frame(MAT_PX, PROTOCOL_COLOR_IDS[color_id], coordinates, sequence, screen_scale)


def encode_plate_frame(coordinates, sequence, screen_scale=1.0):
    return encode_pixel_frame(PLATE_PX, 0, coordinates, sequence, screen_scale)


class CoordinateSender(Protocol8Sender):
    def __init__(self, serial, hz=UART_SEND_HZ, screen_scale=1.0):
        super().__init__(serial, hz, screen_scale)
        self.last_color = 1

    def send(self, color_id, coordinates, now, force=False):
        self.last_color = color_id
        sequence = self.image_sequence()
        packet = (encode_plate_frame(coordinates, sequence, self.screen_scale) if color_id == 0
                  else encode_coordinate_frame(color_id, coordinates, sequence, self.screen_scale))
        state = color_id, bool(packet[2] & 0x80)
        return self.send_packet(packet, state, now, force)

    def send_plate(self, coordinates, now, force=False):
        return self.send(0, coordinates, now, force)

def fit_geometry(fw, fh, sw, sh):
    scale = min(sw / fw, sh / fh)
    width, height = fw * scale, fh * scale
    return (sw - width) / 2, (sh - height) / 2, width, height


def screen_point(x, y, fw, fh, geometry):
    left, top, width, height = geometry
    return left + x * width / fw, top + y * height / fh


def frame_point(x, y, fw, fh, geometry):
    left, top, width, height = geometry
    if not left <= x < left + width or not top <= y < top + height:
        return -1, -1
    return (x - left) * fw / width, (y - top) * fh / height


def centered_coordinates(x, y, sw, sh):
    return x - sw / 2, sh / 2 - y


def inside(x, y, rect):
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh


class TouchButtons:
    def __init__(self):
        self.pressed = False
        self.last_action = -math.inf

    def update(self, x, y, pressed, now, buttons):
        rising = bool(pressed) and not self.pressed
        self.pressed = bool(pressed)
        if not rising or now - self.last_action < BUTTON_DEBOUNCE_S:
            return None
        for name, rect in buttons.items():
            if inside(x, y, rect):
                self.last_action = now
                return name
        return None


def valid_samples(samples):
    result = {}
    if not isinstance(samples, dict):
        raise ValueError("Bad calibration data")
    for key, values in samples.items():
        if key not in ({str(c[0]) for c in COLORS} | {"0"}) or not isinstance(values, list):
            raise ValueError("Bad color samples")
        checked = []
        for value in values[-MAX_SAMPLES:]:
            if not isinstance(value, list) or len(value) != 3:
                raise ValueError("Bad LAB sample")
            if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       and math.isfinite(v) for v in value):
                raise ValueError("Non-finite LAB sample")
            if not (0 <= value[0] <= 100 and -128 <= value[1] <= 127
                    and -128 <= value[2] <= 127):
                raise ValueError("LAB sample outside range")
            checked.append(value)
        if checked:
            result[key] = checked
    return result


def load_samples(path=CALIBRATION_PATH):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            data = json.load(stream)
        if data.get("version") != 1:
            raise ValueError("Unsupported calibration version")
        return valid_samples(data["samples"])
    except FileNotFoundError:
        return {}
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print("Calibration not loaded:", exc)
        return {}


def save_samples(samples, path=CALIBRATION_PATH):
    data = {"version": 1, "samples": valid_samples(samples)}
    with open(path + ".tmp", "w", encoding="utf-8") as stream:
        json.dump(data, stream)
    os.replace(path + ".tmp", path)


def blue_separator(samples):
    """优先找 A/B 轴的样本间隙；色度不可分时才考虑 L 轴。"""
    blue, light = samples.get("3"), samples.get("6")
    if not blue or not light:
        return None, "NEED_PAIR"
    candidates = []
    for axis in (1, 2, 0):
        b = [s[axis] for s in blue]
        c = [s[axis] for s in light]
        if max(b) < min(c):
            gap, low_id, middle = min(c) - max(b), 3, (max(b) + min(c)) / 2
        elif max(c) < min(b):
            gap, low_id, middle = min(b) - max(c), 6, (max(c) + min(b)) / 2
        else:
            continue
        if gap >= (12 if axis == 0 else 6):
            candidates.append((axis, gap, low_id, middle))
    chroma = [c for c in candidates if c[0] != 0]
    choices = chroma or candidates
    if not choices:
        return None, "OVERLAP"
    axis, gap, low_id, middle = max(choices, key=lambda c: c[1])
    return (axis, low_id, math.floor(middle)), "LAB_" + "LAB"[axis]


def calibrated_threshold(color, samples):
    color_id, name, initial = color
    values = samples.get(str(color_id))
    threshold = list(initial)
    status = "DEFAULT"
    if values:
        # 蓝色族放开亮度范围，覆盖同一物料明暗面；色度由现场样本限定。
        threshold = []
        for axis, margin in enumerate((20, 18, 18)):
            if color_id == 0:
                margin = 10 if axis == 0 else 12
            lower, upper = (0, 100) if axis == 0 else (-128, 127)
            lo = max(lower, math.floor(min(v[axis] for v in values) - margin))
            hi = min(upper, math.ceil(max(v[axis] for v in values) + margin))
            if color_id in (3, 6) and axis == 0:
                lo, hi = 0, 100
            threshold.extend((lo, hi))
        status = "SAMPLED"
    if color_id in (3, 6):
        separator, status = blue_separator(samples)
        if separator is not None:
            axis, low_id, cut = separator
            slot = axis * 2
            if color_id == low_id:
                threshold[slot + 1] = min(threshold[slot + 1], cut)
            else:
                threshold[slot] = max(threshold[slot], cut + 1)
    return threshold, status


def sample_lab(frame, x, y):
    # 只在点击取样时统计小块原彩色图，不在每帧执行。
    left = max(0, min(frame.width() - SAMPLE_SIZE, round(x) - SAMPLE_SIZE // 2))
    top = max(0, min(frame.height() - SAMPLE_SIZE, round(y) - SAMPLE_SIZE // 2))
    stats = frame.get_statistics(roi=[left, top, SAMPLE_SIZE, SAMPLE_SIZE])
    return [stats.l_median(), stats.a_median(), stats.b_median()]


def blob_target(blob, width, height, details=None, allow_clipped=False):
    w, h, pixels = blob.w(), blob.h(), blob.pixels()
    radius = (w + h) / 4.0
    aspect = min(w, h) / max(w, h) if max(w, h) > 0 else 0
    density = pixels / float(w * h) if w > 0 and h > 0 else 0
    borders = sum((blob.x() <= 0, blob.y() <= 0,
                   blob.x() + w >= width, blob.y() + h >= height))
    reason = "OK"
    if w <= 0 or h <= 0 or pixels < MIN_BLOB_PIXELS:
        reason = "PIXELS"
    elif w * h > width * height * MAX_BLOB_AREA_FRACTION:
        reason = "AREA"
    elif borders >= 2 or (borders and not allow_clipped):
        reason = "EDGE"
    elif not MIN_RADIUS_PX <= radius <= MAX_RADIUS_PX:
        reason = "SIZE"
    elif aspect < (0.38 if borders else MIN_ASPECT_RATIO):
        reason = "ASPECT"
    elif not MIN_DENSITY <= density <= MAX_DENSITY:
        reason = "DENSITY"
    if details is not None:
        details.update(rect=(blob.x(), blob.y(), w, h), pixels=pixels,
                       radius=radius, aspect=aspect, density=density, borders=borders, reason=reason)
    # 不依赖 roundness()，避免明暗面/轻微椭圆导致硬拒绝。
    return (blob.cx(), blob.cy(), radius) if reason == "OK" else None


def refine_material_shape(binary, blob, target, details=None):
    """在已筛选色块的小ROI验证形状；单边半圆剔除裁切直线后补全。

    不对整图运行Hough/Canny，不改变LAB阈值，不使用SDK roundness的不同语义。
    """
    import cv2
    import numpy as np
    if cv2.getNumThreads() != 1:
        cv2.setNumThreads(1)
    h, w = binary.shape
    bx, by, bw, bh = blob.x(), blob.y(), blob.w(), blob.h()
    contours = cv2.findContours(binary[by:by+bh, bx:bx+bw], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    if not contours:
        return None, 0.0
    contour = max(contours, key=cv2.contourArea)
    area, perimeter = cv2.contourArea(contour), cv2.arcLength(contour, True)
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter else 0.0
    convexity = area / hull_area if hull_area else 0.0
    (_, _), radius = cv2.minEnclosingCircle(contour)
    fill = area / (math.pi * radius * radius) if radius else 0.0
    borders = sum((bx <= 0, by <= 0, bx + bw >= w, by + bh >= h))
    reason = "OK"
    if circularity < MIN_CIRCULARITY:
        reason = "CIRCULARITY"
    elif convexity < MIN_CONVEXITY:
        reason = "CONVEXITY"
    elif fill < (0.38 if borders else 0.50):
        reason = "FILL"
    elif borders:
        points = contour[:, 0, :].astype(np.float32)
        points += np.array((bx, by), np.float32)
        points = points[(points[:, 0] > 1) & (points[:, 0] < w-2)
                        & (points[:, 1] > 1) & (points[:, 1] < h-2)]
        points = points[::max(1, math.ceil(len(points) / 100))]
        if len(points) < 12:
            reason = "SHORT_ARC"
        else:
            # 中心化圆拟合只解2x2，避免嵌入式LAPACK调用；直裁切边不参与拟合。
            mean = points.mean(axis=0)
            normalized = points - mean
            px, py = normalized[:, 0], normalized[:, 1]
            xx, yy, xy = px*px, py*py, px*py
            zz = xx + yy
            aa, bb, cc = float(xx.sum()), float(xy.sum()), float(yy.sum())
            dd, ee = float((px*zz).sum())/2, float((py*zz).sum())/2
            determinant = aa*cc - bb*bb
            if aa <= 0 or cc <= 0 or determinant <= 1e-8*aa*cc:
                reason = "FLAT_ARC"
            else:
                dx, dy = (dd*cc-bb*ee)/determinant, (aa*ee-bb*dd)/determinant
                cx, cy = float(mean[0])+dx, float(mean[1])+dy
                radius = math.sqrt(dx*dx + dy*dy + float(zz.mean()))
                residual = np.abs(np.hypot(points[:, 0]-cx, points[:, 1]-cy) - radius) / max(radius, 1)
                angles = np.sort(np.mod(np.arctan2(points[:, 1]-cy, points[:, 0]-cx), 2*math.pi))
                span = 360 - math.degrees(float(np.diff(np.r_[angles, angles[0]+2*math.pi]).max()))
                fill = area / (math.pi*radius*radius) if radius else 0.0
                if (span < 100 or float(np.sqrt(np.mean(residual**2))) > .06
                        or float(np.percentile(residual, 95)) > .10 or fill < .38
                        or not MIN_RADIUS_PX <= radius <= MAX_RADIUS_PX
                        or not (-.2*radius <= cx <= w-1+.2*radius and -.2*radius <= cy <= h-1+.2*radius)):
                    reason = "BAD_ARC"
                else:
                    target = (cx, cy, radius)
                    reason = "HALF_OK"
    if details is not None:
        details.update(circularity=circularity, convexity=convexity, fill=fill, reason=reason)
    score = blob.pixels() * (.65+circularity) * (.65+min(fill, 1.0))
    return (target, score) if reason in ("OK", "HALF_OK") else (None, 0.0)


def detect_material_center(frame, threshold, image, diagnostics=None):
    frame.binary([threshold], copy=False)
    mask = frame.to_format(image.Format.FMT_GRAYSCALE)
    width, height = mask.width(), mask.height()
    blobs = mask.find_blobs([[128, 255]], x_stride=2, y_stride=2,
                           area_threshold=MIN_BLOB_PIXELS,
                           pixels_threshold=MIN_BLOB_PIXELS, merge=False)
    best = None
    best_score, largest_pixels, best_distance = -1, -1, math.inf
    # SDK设备提供image2cv；电脑的原生接口mock可独立验证原轻量分支。
    refine = SHAPE_REFINEMENT_ENABLED and hasattr(image, "image2cv")
    binary = None
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics.update(count=len(blobs), reason="NO_BLOBS")
    for blob in sorted(blobs, key=lambda b: b.pixels(), reverse=True)[:MAX_SHAPE_CANDIDATES]:
        details = {} if diagnostics is not None else None
        target = blob_target(blob, width, height, details, allow_clipped=refine)
        if diagnostics is not None and best is None and blob.pixels() > largest_pixels:
            diagnostics.update(details)
            largest_pixels = blob.pixels()
        if target is None:
            continue
        score = blob.pixels()
        if refine:
            if binary is None:
                binary = image.image2cv(mask, ensure_bgr=False, copy=False)
                if binary.ndim == 3 and binary.shape[2] == 1:
                    binary = binary[:, :, 0]
                if binary.ndim != 2 or binary.shape != (height, width):
                    raise ValueError("Unexpected material binary array geometry")
            target, score = refine_material_shape(binary, blob, target, details)
            if target is None:
                # 完整物料的LAB掩码会包含明暗侧面；形状低分仅降权，不用新硬阈值
                # 破坏已有效的原生定位。单边裁切必须真正通过弧拟合，不能用偏置质心。
                borders = sum((blob.x() <= 0, blob.y() <= 0,
                               blob.x()+blob.w() >= width, blob.y()+blob.h() >= height))
                if borders == 0:
                    target = (blob.cx(), blob.cy(), (blob.w()+blob.h())/4.0)
                    score = blob.pixels() * 0.50
                    if details is not None:
                        details["reason"] = "WEAK_SHAPE"
                else:
                    if diagnostics is not None and best is None:
                        diagnostics.update(details)
                    continue
        distance = (target[0] - width / 2) ** 2 + (target[1] - height / 2) ** 2
        if score > best_score or (score == best_score and distance < best_distance):
            best, best_score, best_distance = target, score, distance
            if diagnostics is not None:
                diagnostics.update(details)
    return best, mask


def fit_plate_color_array(rgb, diagnostics=None, order='RGB', _windowed=True, _prepared=None):
    """在彩色图上分析亮度梯度和中性色边缘，拟合可见盘沿；保留原图。

    返回 (圆弧结果, 调试边缘图)。只使用局部边缘，不按白色阈值分割盘面。
    """
    import itertools
    import cv2
    import numpy as np
    h, w = rgb.shape[:2]
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError('RGB uint8 required')
    if cv2.getNumThreads() != 1:
        cv2.setNumThreads(1)
    if _prepared is None:
        lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB if order == 'RGB' else cv2.COLOR_BGR2LAB)
        light = cv2.GaussianBlur(lab[:, :, 0], (5, 5), 0)
        edges = cv2.Canny(light, PLATE_EDGE_LOW, PLATE_EDGE_HIGH, L2gradient=True)
        # 仅在稀疏边缘点检查色度，不对整张图做浮点 hypot。
        edge_points = cv2.findNonZero(edges)
        if edge_points is not None:
            edge_points = edge_points.reshape(-1, 2)  # 兼容 OpenCV 4/5 返回的点数组形状。
            edge_x, edge_y = edge_points[:, 0], edge_points[:, 1]
            neutral = lab[edge_y, edge_x, 1:].astype(np.int32) - 128
            colored = np.sum(neutral * neutral, axis=1) > PLATE_NEUTRAL_CHROMA_MAX ** 2
            edges[edge_y[colored], edge_x[colored]] = 0
        gx = cv2.Sobel(light, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(light, cv2.CV_32F, 0, 1, ksize=3)
        edges[gy >= -2.0] = 0  # 下侧盘沿由亮向暗，剔除阴影回升边及杂线。
    else:
        edges, gx, gy = _prepared
    full_edges = edges
    if _windowed:
        # 优先处理盘沿区域，避免顶部横线/文字与底部地面阴影连入圆弧。
        # 无可靠候选时用同一套边缘回退全图，不改变ROI外真实圆盘的可识别范围。
        edges = edges.copy()
        edges[:round(h * PLATE_SEARCH_TOP_FRACTION)] = 0
        edges[round(h * PLATE_SEARCH_BOTTOM_FRACTION):] = 0
    contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[-2]
    arcs = []
    for contour in sorted(contours, key=len, reverse=True)[:50]:
        x, y, bw, bh = cv2.boundingRect(contour)
        if len(contour) < 30 or bw < w * 0.10 or bh < 6:
            continue
        points = contour[:, 0, :]
        points = points[(points[:, 0] > 2) & (points[:, 0] < w - 3) & (points[:, 1] > 2) & (points[:, 1] < h - 3)]
        if len(points) < 30:
            continue
        points = points[::max(1, len(points) // 180)]
        arcs.append(points.astype(np.float32))
        if len(arcs) >= 14:
            break
    proposals = list(arcs)
    bounds = [(float(points[:, 0].min()), float(points[:, 0].max())) for points in arcs]
    samples = [points[::max(1, len(points) // 24)] for points in arcs]
    join_distance_sq = max(12, w * PLATE_MAX_JOIN_GAP_FRACTION) ** 2
    for i, j in itertools.combinations(range(len(arcs)), 2):
        # 合并后横向跨度仍不够的候选，不会通过后续条件，提前剔除。
        if max(bounds[i][1], bounds[j][1]) - min(bounds[i][0], bounds[j][0]) < w * 0.35:
            continue
        left, right = arcs[i], arcs[j]
        a, b = samples[i], samples[j]
        distance_sq = np.sum((a[:, None] - b[None, :]) ** 2, axis=2).min()
        if distance_sq < join_distance_sq:
            proposals.append(np.vstack((left, right)))
    candidates = []
    for points in proposals:
        if np.ptp(points[:, 0]) < w * 0.35:
            continue
        support = points
        # 原候选点及对应梯度在迭代中不变，只取一次。
        px, py = points[:, 0], points[:, 1]
        xx = np.clip(np.rint(px).astype(int), 0, w - 1)
        yy = np.clip(np.rint(py).astype(int), 0, h - 1)
        point_gx, point_gy = gx[yy, xx], gy[yy, xx]
        good = True
        for iteration in range(3):
            if len(support) < 20:
                good = False
                break
            ellipse = cv2.fitEllipse(support.reshape(-1, 1, 2))
            (cx, cy), (ew, eh), angle = ellipse
            a, b = (ew / 2, eh / 2)
            if min(a, b) < w * 0.14 or max(a, b) > w * 3 or min(a, b) / max(a, b) < 0.35:
                good = False
                break
            theta = math.radians(angle)
            sn, cs = (math.sin(theta), math.cos(theta))
            ext = math.hypot(a * sn, b * cs)
            ty = cy + ext
            tx = cx + (a * a - b * b) * sn * cs / ext
            if not 3 < tx < w - 4 or not 3 < ty < h - 4 or cy > h * 0.6:
                good = False
                break
            dx, dy = (px - cx, py - cy)
            ex, ey = (dx * cs + dy * sn, -dx * sn + dy * cs)
            residual = np.abs(np.sqrt((ex / a) ** 2 + (ey / b) ** 2) - 1) * min(a, b)
            nx = ex / (a * a) * cs - ey / (b * b) * sn
            ny = ex / (a * a) * sn + ey / (b * b) * cs
            dot = (point_gx * nx + point_gy * ny) / np.maximum(np.hypot(nx, ny), 1e-06)
            if iteration == 0 and np.median(residual) > 4:
                if np.ptp(points[:, 0]) < w * 0.7:
                    good = False
                    break
                rng = np.random.default_rng(11)
                # 保持相同的 32 组随机种子，一次批量评估，减少 Python/NumPy 调用。
                trials = np.asarray([rng.choice(len(points), 3, replace=False) for _ in range(32)])
                seed = points[trials].astype(float)
                p0, u, v = seed[:, 0], seed[:, 1] - seed[:, 0], seed[:, 2] - seed[:, 0]
                det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
                usable = np.abs(det) >= 1e-05
                safe_det = np.where(usable, det, 1.0)
                su, sv = np.sum(u * u, axis=1) / 2, np.sum(v * v, axis=1) / 2
                ox = (su * v[:, 1] - u[:, 1] * sv) / safe_det
                oy = (u[:, 0] * sv - su * v[:, 0]) / safe_det
                rr = np.hypot(ox, oy)
                usable &= (rr > w * 0.2) & (rr < w * 3)
                cc = p0 + np.column_stack((ox, oy))
                res = np.abs(np.hypot(px[None, :] - cc[:, 0, None],
                                      py[None, :] - cc[:, 1, None]) - rr[:, None])
                selections = usable[:, None] & (res < 3.5) & (py[None, :] > cc[:, 1, None])
                counts = selections.sum(axis=1)
                best_index = int(counts.argmax())
                if counts[best_index] < len(points) * 0.6:
                    good = False
                    break
                support = points[selections[best_index]]
                continue
            selected = (residual < 3.5) & (dot < -3) & (py > cy)
            support = points[selected]
        if not good or len(support) < 30:
            continue
        ratio = float(selected.mean())
        if ratio < PLATE_MIN_INLIER_RATIO or np.ptp(support[:, 0]) < w * 0.35:
            continue
        if support[:, 0].min() > tx - w * 0.04 or support[:, 0].max() < tx + w * 0.04:
            continue
        dx, dy = (support[:, 0] - cx, support[:, 1] - cy)
        ex, ey = (dx * cs + dy * sn, -dx * sn + dy * cs)
        ordered = np.argsort(support[:, 0])
        angles = np.unwrap(np.arctan2(ey[ordered] / b, ex[ordered] / a))
        span = float(np.degrees(np.ptp(angles)))
        if not PLATE_MIN_ARC_DEG < span < 210:
            continue
        rms = float(np.sqrt(np.mean(residual[selected] ** 2)))
        score = 0.45 * ratio + 0.35 * min(span / 140, 1) + 0.2 * (1 - min(rms / 3.5, 1))
        candidates.append(dict(ellipse=ellipse, circle=(cx, cy, ext), tangent_x=tx, tangent_y=ty, arc_deg=span, inlier_ratio=ratio, rms=rms, score=score))
    candidates.sort(key=lambda x: x['score'], reverse=True)
    result = candidates[0] if candidates else None
    if len(candidates) > 1:
        first, second = candidates[:2]
        if first['score'] - second['score'] < 0.08 and (abs(first['tangent_x'] - second['tangent_x']) > w * 0.15 or abs(first['tangent_y'] - second['tangent_y']) > h * 0.06):
            result = None
    if _windowed and not candidates:
        # 复用LAB/Canny/Sobel结果；光照/形状不适合局部窗口时保留原全图路径。
        return fit_plate_color_array(rgb, diagnostics, order, False, (full_edges, gx, gy))
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics.update(search='ARC_WINDOW' if _windowed else 'FULL_FALLBACK',
                           contours=len(contours), arcs=len(arcs), proposals=len(proposals),
                           count=len(candidates), reason='EDGE_OK' if result else 'AMBIGUOUS_EDGE' if candidates else 'NO_EDGE_ARC')
        if result:
            diagnostics.update(arc_deg=result['arc_deg'], rms=result['rms'])
    return (result, edges)


def detect_plate_color(frame, image, diagnostics, edges_view=False):
    # RGB888 的 ndarray 借用原图内存，分析不修改原图；显示继续保留彩色。
    array = image.image2cv(frame, ensure_bgr=False, copy=False)
    result, edges = fit_plate_color_array(array, diagnostics, order="RGB")
    # 调试边缘图复制到 maix.Image，避免局部 ndarray 销毁后出现悬空引用。
    source = image.cv2image(edges, bgr=False, copy=True) if edges_view else frame
    return result, source


class PlateTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous_y = None
        self.count = 0

    def update(self, result):
        if result is None:
            self.reset()
            return None
        y = result["tangent_y"]
        self.count = (self.count + 1 if self.previous_y is not None
                      and abs(y - self.previous_y) <= PLATE_MAX_Y_STEP_PX else 1)
        self.previous_y = y
        return result if self.count >= PLATE_CONFIRM_FRAMES else None


def camera_lock(cam):
    # 在光照已稳定后读取当前值再冻结，不写未经标定的白平衡常量。
    exposure, gain, wb = cam.exposure(), cam.gain(), cam.set_wb_gain()
    if exposure <= 0 or gain <= 0 or len(wb) != 4 or not any(v > 0 for v in wb):
        raise ValueError("Camera settings unavailable")
    cam.set_wb_gain(wb)
    cam.exposure(exposure)
    cam.gain(gain)


def draw_screen(source, target, geometry, sw, sh, color, fps, buttons,
                image, diagnostics, learning, cal_status, sample_count, locked, message,
                live_lab=None, uart_status="OFF", plate_target=None, plate_edge_view=False):
    width, height = source.width(), source.height()
    if not learning:
        source.draw_line(0, height // 2, width - 1, height // 2, image.COLOR_WHITE)
        source.draw_line(width // 2, 0, width // 2, height - 1, image.COLOR_WHITE)
        source.draw_string(width // 2 + 4, height // 2 + 4, "(0,0)", image.COLOR_WHITE)
    else:
        # 框内才是实时 LAB 的统计区域；按 Sample 保存该区域。
        left, top = width // 2 - SAMPLE_SIZE // 2, height // 2 - SAMPLE_SIZE // 2
        source.draw_rect(left - 1, top - 1, SAMPLE_SIZE + 2, SAMPLE_SIZE + 2,
                         image.COLOR_BLACK, thickness=1)
        source.draw_rect(left, top, SAMPLE_SIZE, SAMPLE_SIZE, image.COLOR_WHITE, thickness=1)
    is_plate = color[0] == 0
    text = "ALIGN BOX / TAP OR SAMPLE" if learning else "X: -- Y: --"
    if plate_target is not None and not learning:
        (cx, cy), (axis_w, axis_h), angle = plate_target["ellipse"]
        tangent_x = plate_target["tangent_x"]
        line_y = plate_target["tangent_y"]
        sx, sy = screen_point(tangent_x, line_y, width, height, geometry)
        x_coordinate, y_coordinate = centered_coordinates(sx, sy, sw, sh)
        source.draw_ellipse(round(cx), round(cy), round(axis_w / 2), round(axis_h / 2),
                            angle, 0, 360, image.COLOR_YELLOW, thickness=1)
        source.draw_line(0, round(line_y), width - 1, round(line_y), image.COLOR_GREEN, thickness=2)
        source.draw_line(round(tangent_x) - 5, round(line_y), round(tangent_x) + 5, round(line_y), image.COLOR_RED)
        text = "X:%+.1f Y:%+.1f px" % (x_coordinate, y_coordinate)
    elif target is not None:
        cx, cy, radius = target
        sx, sy = screen_point(cx, cy, width, height, geometry)
        x, y = centered_coordinates(sx, sy, sw, sh)
        source.draw_circle(round(cx), round(cy), round(radius) + 2, image.COLOR_WHITE)
        source.draw_line(round(cx) - 5, round(cy), round(cx) + 5, round(cy), image.COLOR_BLACK)
        source.draw_line(round(cx), round(cy) - 5, round(cx), round(cy) + 5, image.COLOR_BLACK)
        text = "X:%+.1f Y:%+.1f px" % (x, y)
    elif not learning and "rect" in diagnostics:
        bx, by, bw, bh = diagnostics["rect"]
        source.draw_rect(bx, by, bw, bh, image.COLOR_WHITE)
    source.draw_rect(0, 0, width, HEADER_HEIGHT, image.COLOR_BLACK, thickness=-1)
    source.draw_string(4, 2, text, image.COLOR_WHITE)
    source.draw_string(4, 19, "%s FPS:%.1f %s TX:%s" % (
        color[1], fps, "LOCK" if locked else "AUTO", uart_status), image.COLOR_WHITE)
    if learning:
        source.draw_string(4, 36, "N:%d %s" % (sample_count, cal_status), image.COLOR_WHITE)
        detail = message
        if not detail and live_lab is not None:
            detail = "BOX LAB:%d,%d,%d" % tuple(live_lab)
        source.draw_string(4, 53, detail or "Align material inside box", image.COLOR_WHITE)
    else:
        source.draw_string(4, 36, ("A:%d %s %s" if is_plate else "B:%d %s %s") % (
            diagnostics["count"], diagnostics["reason"], cal_status), image.COLOR_WHITE)
        if message:
            source.draw_string(4, 53, message, image.COLOR_WHITE)
        elif "radius" in diagnostics:
            source.draw_string(4, 53, "R:%.0f A:%.2f D:%.2f" % (
                diagnostics["radius"], diagnostics["aspect"], diagnostics["density"]), image.COLOR_WHITE)
        elif "arc_deg" in diagnostics:
            source.draw_string(4, 53, "Arc:%.0f Err:%.1f" % (
                diagnostics["arc_deg"], diagnostics["rms"]), image.COLOR_WHITE)
    for name in ("color", "learn", "lock"):
        x, y, w, h = buttons[name]
        source.draw_rect(x, y, w, h, image.COLOR_BLACK, thickness=-1)
        source.draw_rect(x, y, w, h, image.COLOR_WHITE)
        label = {"color": "Mode" if is_plate else "Color",
                 "learn": ("Image" if plate_edge_view else "Edges") if is_plate else ("Done" if learning else "Learn"),
                 "lock": "Sample" if learning else ("Auto" if locked else "Lock")}[name]
        source.draw_string(x + 7, y + 8, label, image.COLOR_WHITE)


def main():
    from maix import app, camera, display, image, touchscreen, err
    from maix.peripheral import uart, pinmap, gpio
    cam = disp = touch = serial = sender = light = None
    try:
        light = init_light_on(gpio, pinmap, err)
        if UART_ENABLED:
            for pin, function in UART_PIN_FUNCTIONS.items():
                err.check_raise(pinmap.set_pin_function(pin, function),
                                "UART pin mapping failed: " + pin)
            serial = uart.UART(UART_PORT, UART_BAUDRATE,
                               databits=uart.BITS.BITS_8, parity=uart.PARITY.PARITY_NONE,
                               stopbits=uart.STOP.STOP_1, flow_ctrl=uart.FLOW_CTRL.FLOW_CTRL_NONE)
            sender = CoordinateSender(serial)
        disp = display.Display()
        sw, sh = disp.width(), disp.height()
        cam = camera.Camera(CAM_WIDTH, CAM_HEIGHT, image.Format.FMT_RGB888, fps=CAM_FPS)
        touch = touchscreen.TouchScreen()
        fw, fh = cam.width(), cam.height()
        geometry = fit_geometry(fw, fh, sw, sh)
        if sender is not None:
            sender.screen_scale = min(sw / fw, sh / fh)
        third = fw // 3
        buttons = {name: (i * third + 3, fh - FOOTER_HEIGHT, third - 6, 32)
                   for i, name in enumerate(("color", "learn", "lock"))}
        buttons["sample"] = (0, HEADER_HEIGHT, fw, fh - HEADER_HEIGHT - FOOTER_HEIGHT)
        controller = TouchButtons()
        samples = load_samples()
        color_index, learning, locked = 0, False, False
        reset_sample = False
        threshold, cal_status = calibrated_threshold(MODES[color_index], samples)
        fps, previous_time = 0.0, time.monotonic()
        diagnostics = {}
        message = ""
        live_lab, next_live_lab = None, 0.0
        while not app.need_exit():
            sample_point = None
            for _ in range(16):
                if not touch.available():
                    break
                tx, ty, pressed = touch.read0()
                fx, fy = frame_point(tx, ty, fw, fh, geometry)
                action = controller.update(fx, fy, pressed, time.monotonic(), buttons)
                if action == "color":
                    color_index = (color_index + 1) % len(MODES)
                    learning, sample_point, message = False, None, ""
                    reset_sample = False
                    threshold, cal_status = calibrated_threshold(MODES[color_index], samples)
                    if sender is not None:
                        # 换色先失效，不沿用上一颜色的测量。
                        sender.send(MODES[color_index][0], None, time.monotonic(), force=True)
                elif action == "learn":
                    learning = not learning
                    sample_point, message = None, ""
                    # 首次有效取样时才替换旧样本，直接 Done 不丢弃旧标定。
                    reset_sample = learning
                    live_lab, next_live_lab = None, 0.0
                    threshold, cal_status = calibrated_threshold(MODES[color_index], samples)
                    if learning and sender is not None:
                        sender.send(MODES[color_index][0], None, time.monotonic(), force=True)
                elif action == "sample" and learning:
                    sample_point = fx, fy
                elif action == "lock" and learning:
                    sample_point = fw / 2, fh / 2
                elif action == "lock":
                    try:
                        if locked:
                            cam.exp_mode(camera.AeMode.Auto)
                            cam.awb_mode(camera.AwbMode.Auto)
                        else:
                            camera_lock(cam)
                        locked = not locked
                        message = "Camera settings changed"
                    except Exception as exc:
                        print("Camera setting failed:", exc)
                        for restore in (lambda: cam.exp_mode(camera.AeMode.Auto),
                                        lambda: cam.awb_mode(camera.AwbMode.Auto)):
                            try:
                                restore()
                            except Exception as restore_exc:
                                print("Auto restore failed:", restore_exc)
                        locked = False
                        message = "LOCK FAIL: check camera"
            frame = cam.read()
            if frame is None:
                raise RuntimeError("Camera returned no frame")
            color = MODES[color_index]
            if learning and time.monotonic() >= next_live_lab:
                # 只在 Learn 模式低频统计，正常二值检测完全不增加此开销。
                live_lab = sample_lab(frame, fw / 2, fh / 2)
                next_live_lab = time.monotonic() + LIVE_LAB_INTERVAL_S
            if learning and sample_point is not None:
                value = sample_lab(frame, *sample_point)
                if color[0] in (3, 6) and (value[0] < 5 or value[0] > 95 or value[2] >= -3):
                    message = "Avoid dark/glare/background"
                else:
                    if reset_sample:
                        samples[str(color[0])] = []
                        reset_sample = False
                    values = samples.setdefault(str(color[0]), [])
                    values.append(value)
                    del values[:-MAX_SAMPLES]
                    threshold, cal_status = calibrated_threshold(color, samples)
                    message = "LAB:%d,%d,%d" % tuple(value)
                    try:
                        save_samples(samples)
                    except OSError as exc:
                        print("Calibration save failed:", exc)
                        message = "SAVE FAIL: RAM only"
            if learning:
                target, source = None, frame
            else:
                target, source = detect_material_center(frame, threshold, image, diagnostics)
                if cal_status == "OVERLAP":
                    target = None
                    diagnostics["reason"] = "CAL_OVERLAP"
            now = time.monotonic()
            if now > previous_time:
                measured = 1 / (now - previous_time)
                fps = measured if fps == 0 else fps * 0.8 + measured * 0.2
            previous_time = now
            uart_status = "OFF"
            if sender is not None:
                coordinates = None
                if target is not None and not learning:
                    sx, sy = screen_point(target[0], target[1], fw, fh, geometry)
                    coordinates = centered_coordinates(sx, sy, sw, sh)
                # 取样/目标丢失/标定重叠都发送 valid=0，x/y 清零。
                sender.send(color[0], coordinates, now)
                uart_status = "%d/%d" % (sender.sent % 10000, sender.errors % 1000)
            draw_screen(source, target, geometry, sw, sh, color, fps, buttons, image,
                        diagnostics, learning, cal_status,
                        0 if learning and reset_sample else len(samples.get(str(color[0]), [])),
                        locked, message, live_lab, uart_status)
            disp.show(source, fit=image.Fit.FIT_CONTAIN)
            del frame, source
    finally:
        light_off_on_exit(light)
        if sender is not None:
            try:
                # 正常退出尽力发送无效帧；主控仍须用接收超时兜底。
                sender.send(sender.last_color, None, time.monotonic(), force=True)
            except Exception as exc:
                print("Final UART invalidation failed:", exc)
        for device in (serial, touch, cam, disp):
            if device is not None:
                try:
                    device.close()
                except Exception as exc:
                    print("Device close failed:", exc)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        log_error("main")
        raise
