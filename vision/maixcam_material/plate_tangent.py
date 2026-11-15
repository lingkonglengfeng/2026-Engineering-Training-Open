"""MaixCAM 白色圆盘底部水平切线：单文件上传即运行。

MaixVision 打开本文件，直接运行，无需 main.py 或颜色标定文件。
电脑照片验证：python plate_tangent.py --image photo.png --output result.png
显示坐标以实际显示屏中心为原点，右正、上正；串口发送相对X中心/Y图像75%目标的纠偏误差。
"""
import math
import os
import time
import traceback

CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 320, 240, 30
PLATE_AIM_Y_FRACTION = 0.75  # 有效图像高度75%；320x240时目标y=180。
PROCESS_WIDTH = 224  # 拟合图宽度；原图/显示仍为 320×240。设 320 可恢复全分辨率分析。
UART_ENABLED = True
UART_PORT, UART_BAUDRATE = "/dev/ttyS0", 115200
UART_PIN_FUNCTIONS = {"A16": "UART0_TX", "A17": "UART0_RX"}
UART_SEND_HZ, UART_TX_STALL_S = 20, 0.20
PREVIEW_JPEG_QUALITY = 80  # 只影响传到 MaixVision 的预览，不影响检测原图。
SHOW_TIMING = True  # 详细耗时只输出到控制台，画面保持简洁。
PLATE_EDGE_LOW, PLATE_EDGE_HIGH = 4, 12
PLATE_NEUTRAL_CHROMA_MAX = 35
PLATE_MAX_JOIN_GAP_FRACTION = 0.20
PLATE_MIN_INLIER_RATIO = 0.60
PLATE_MIN_ARC_DEG = 35.0
PLATE_CONFIRM_FRAMES = 3
PLATE_MAX_Y_STEP_PX = 24.0
PLATE_SEARCH_TOP_FRACTION = 0.22
PLATE_SEARCH_BOTTOM_FRACTION = 0.92
ERROR_LOG_PATH = os.path.expanduser("~/maixcam_plate_error.log")
FATAL_LOG_PATH = os.path.expanduser("~/maixcam_plate_fatal.log")


# LIGHT_GPIO_BEGIN
LIGHT_PIN = "A19"
LIGHT_GPIO = "GPIOA19"
LIGHT_OFF_LEVEL = 0  # 按高电平使能灯板；低电平使能板需改为1。
LIGHT_PWM_ENABLED = True  # 由各入口配置 GPIO/PWM；亮度为灯板占空比。
LIGHT_PWM_ID, LIGHT_PWM_FREQ_HZ = 7, 20000
LIGHT_BRIGHTNESS_PERCENT = 30.0


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

# UART_PROTOCOL8_BEGIN
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
    detail = "%s [%s]\n%s" % (time.strftime("%Y-%m-%d %H:%M:%S"), stage,
                               traceback.format_exc())
    print(detail, flush=True)
    try:
        mode = "w" if os.path.exists(ERROR_LOG_PATH) and os.path.getsize(ERROR_LOG_PATH) > 262144 else "a"
        with open(ERROR_LOG_PATH, mode, encoding="utf-8") as stream:
            stream.write(detail + "\n")
    except OSError as exc:
        print("Error log save failed:", exc, flush=True)


# CORE_DETECTOR: copied from main.py to keep MaixVision single-file uploads usable.
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


def scale_plate_result(result, sx, sy):
    """把缩小图的拟合结果映射回原图，包含旋转椭圆的非等比缩放。"""
    if result is None:
        return None
    scaled = dict(result)
    (cx, cy), (ew, eh), angle = result["ellipse"]
    sn, cs = math.sin(math.radians(angle)), math.cos(math.radians(angle))
    a2, b2 = (ew / 2) ** 2, (eh / 2) ** 2
    qxx, qyy = sx * sx * (a2 * cs * cs + b2 * sn * sn), sy * sy * (a2 * sn * sn + b2 * cs * cs)
    qxy = sx * sy * (a2 - b2) * sn * cs
    delta = math.hypot(qxx - qyy, 2 * qxy)
    axis_a = 2 * math.sqrt(max(0.0, (qxx + qyy + delta) / 2))
    axis_b = 2 * math.sqrt(max(0.0, (qxx + qyy - delta) / 2))
    scaled["ellipse"] = ((cx * sx, cy * sy), (axis_a, axis_b),
                         math.degrees(0.5 * math.atan2(2 * qxy, qxx - qyy)))
    scaled["tangent_x"], scaled["tangent_y"] = result["tangent_x"] * sx, result["tangent_y"] * sy
    scaled["circle"] = (cx * sx, cy * sy, math.sqrt(qyy))
    scaled["rms"] = result["rms"] * max(sx, sy)
    return scaled


def fit_plate_preview_array(array, diagnostics=None, order="RGB", edges_view=False):
    """小图做拟合，输出恢复为相机原图像素，保持原屏幕坐标定义。"""
    import cv2
    fh, fw = array.shape[:2]
    pw = min(PROCESS_WIDTH, fw)
    if pw < fw:
        ph = max(1, round(fh * pw / fw))
        small = cv2.resize(array, (pw, ph), interpolation=cv2.INTER_AREA)
        result, edges = fit_plate_color_array(small, diagnostics, order)
        result = scale_plate_result(result, fw / pw, fh / ph)
        if result is not None and diagnostics is not None:
            diagnostics["rms"] = result["rms"]
        if edges_view:
            edges = cv2.resize(edges, (fw, fh), interpolation=cv2.INTER_NEAREST)
        return result, edges
    return fit_plate_color_array(array, diagnostics, order)


def detect_plate_color(frame, image, diagnostics, edges_view=False):
    # RGB888 的 ndarray 借用原图内存，分析不修改原图；显示继续保留彩色。
    array = image.image2cv(frame, ensure_bgr=False, copy=False)
    result, edges = fit_plate_preview_array(array, diagnostics, order="RGB", edges_view=edges_view)
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


# CORE_DETECTOR_END


def encode_plate_frame(coordinates, sequence, screen_scale=1.0):
    return encode_pixel_frame(PLATE_PX, 0, coordinates, sequence, screen_scale)


class PlateSender(Protocol8Sender):
    def __init__(self, serial, screen_scale=1.0):
        super().__init__(serial, UART_SEND_HZ, screen_scale)

    def send(self, coordinates, now, force=False):
        packet = encode_plate_frame(coordinates, self.image_sequence(), self.screen_scale)
        return self.send_packet(packet, bool(packet[2] & 0x80), now, force)

def tangent_screen_coordinates(result, fw, fh, sw, sh):
    """红色切点的屏幕中心相对坐标；与椭圆圆心分别计算。"""
    geometry = fit_geometry(fw, fh, sw, sh)
    sx, sy = screen_point(result["tangent_x"], result["tangent_y"], fw, fh, geometry)
    return sx - sw / 2, sh / 2 - sy


def tangent_alignment_error(result, fw, fh, sw, sh):
    """控制偏差：切线落在图像75%高度时Y=0，界面原点仍保留屏幕中心。"""
    x,y=tangent_screen_coordinates(result,fw,fh,sw,sh)
    scale=min(sw/fw,sh/fh)
    return x,y+(PLATE_AIM_Y_FRACTION-.5)*fh*scale


def draw_plate_overlay(source, result, image):
    aim_y=round(source.height()*PLATE_AIM_Y_FRACTION)
    source.draw_line(0,aim_y,source.width()-1,aim_y,image.COLOR_WHITE,thickness=1)
    if result is None:
        return
    (cx, cy), (ew, eh), angle = result["ellipse"]
    # MaixPy v4 要求显式传入 start_angle / end_angle；漏传会在首次检出时退出。
    source.draw_ellipse(round(cx), round(cy), round(ew / 2), round(eh / 2),
                        angle, 0, 360, image.COLOR_YELLOW, thickness=1)
    tx, ty = round(result["tangent_x"]), round(result["tangent_y"])
    source.draw_line(0, ty, source.width() - 1, ty, image.COLOR_GREEN, thickness=2)
    source.draw_line(tx - 5, ty, tx + 5, ty, image.COLOR_RED, thickness=2)


def draw_status_text(source, x, y, text, image, color=None):
    """只给字形描边，保留文字后方的相机画面。"""
    if not text:
        return
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        source.draw_string(x + dx, y + dy, text, image.COLOR_BLACK)
    source.draw_string(x, y, text, image.COLOR_WHITE if color is None else color)


def draw_ui(source, result, diagnostics, image, screen_size, fps, edges_view,
            locked, fault, uart_status, status):
    fw, fh = source.width(), source.height()
    draw_plate_overlay(source, result, image)
    coordinate = "X:-- Y:--" if result is None else "X:%+.1f Y:%+.1f px" % tangent_screen_coordinates(result, fw, fh, *screen_size)
    # 固定两行间距，避免 SDK 默认换行高度与额外状态行重叠。
    draw_status_text(source, 4, 4, coordinate, image)
    draw_status_text(source, 4, 28, "FPS:%.1f" % fps, image)
    warning = "ERROR: tap Retry" if fault else (status if "OFF" in status or "FAIL" in status else "")
    if warning:
        draw_status_text(source, 4, 52, warning, image, image.COLOR_RED)
    for i, label in enumerate(("Image" if edges_view else "Edges", "Retry", "Auto" if locked else "Lock")):
        x, width = i * (fw // 3) + 3, fw // 3 - 6
        source.draw_rect(x, fh - 37, width, 32, image.COLOR_BLACK, thickness=-1)
        source.draw_rect(x, fh - 37, width, 32, image.COLOR_WHITE)
        source.draw_string(x + 7, fh - 29, label, image.COLOR_WHITE)


def run_device():
    from maix import app, camera, display, image, touchscreen, err
    from maix.peripheral import gpio, pinmap
    cam = disp = touch = serial = sender = light = None
    stage = "display"
    try:
        stage = "A19 light GPIO"
        light = init_light_on(gpio, pinmap, err)
        stage = "display"
        print("[PLATE] starting standalone detector", flush=True)
        disp = display.Display()
        if hasattr(display, "set_trans_image_quality"):
            display.set_trans_image_quality(PREVIEW_JPEG_QUALITY)
        sw, sh = disp.width(), disp.height()
        boot = image.Image(CAM_WIDTH, CAM_HEIGHT, image.Format.FMT_RGB888)
        boot.draw_string(4, 10, "PLATE: loading OpenCV...", image.COLOR_WHITE)
        disp.show(boot, fit=image.Fit.FIT_CONTAIN)
        stage = "OpenCV / NumPy"
        import cv2
        import numpy as np
        cv2.setNumThreads(1)
        print("[PLATE] OpenCV %s, NumPy %s" % (cv2.__version__, np.__version__), flush=True)
        stage = "UART"
        status = ""
        if UART_ENABLED:
            try:
                from maix.peripheral import uart, pinmap
                for pin, function in UART_PIN_FUNCTIONS.items():
                    err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping: " + pin)
                serial = uart.UART(UART_PORT, UART_BAUDRATE, databits=uart.BITS.BITS_8,
                                   parity=uart.PARITY.PARITY_NONE, stopbits=uart.STOP.STOP_1,
                                   flow_ctrl=uart.FLOW_CTRL.FLOW_CTRL_NONE)
                sender = PlateSender(serial)
                sender.send(None, time.monotonic(), force=True)
            except Exception:
                log_error(stage)
                status = "UART OFF: check console/log"
        stage = "camera"
        cam = camera.Camera(CAM_WIDTH, CAM_HEIGHT, image.Format.FMT_RGB888, fps=CAM_FPS)
        fw, fh = cam.width(), cam.height()
        geometry = fit_geometry(fw, fh, sw, sh)
        if sender is not None:
            sender.screen_scale = min(sw / fw, sh / fh)
        stage = "touchscreen"
        try:
            touch = touchscreen.TouchScreen()
        except Exception:
            log_error(stage)
            status = "TOUCH OFF: check console/log"
        tracker = PlateTracker()
        edges_view = locked = fault = pressed_before = False
        last_action, previous_time, fps = -math.inf, time.monotonic(), 0.0
        timings = {"camera": 0.0, "detect": 0.0, "view": 0.0}
        next_timing_log = time.monotonic() + 2.0
        print("[PLATE] ready, camera %dx%d; screen %dx%d" % (fw, fh, sw, sh), flush=True)
        while not app.need_exit():
            stage = "touchscreen read"
            if touch is not None:
                for _ in range(16):
                    if not touch.available():
                        break
                    tx, ty, pressed = touch.read0()
                    fx, fy = frame_point(tx, ty, fw, fh, geometry)
                    now = time.monotonic()
                    rising = bool(pressed) and not pressed_before
                    pressed_before = bool(pressed)
                    if not rising or now - last_action < 0.15 or not (0 <= fx < fw and fh - 37 <= fy < fh - 5):
                        continue
                    last_action = now
                    button = min(2, int(fx / (fw // 3)))
                    if button == 0:
                        edges_view = not edges_view
                    elif button == 1:
                        fault = False
                        tracker.reset()
                        if sender is not None:
                            sender.send(None, now, force=True)
                    else:
                        try:
                            if locked:
                                cam.exp_mode(camera.AeMode.Auto)
                                cam.awb_mode(camera.AwbMode.Auto)
                            else:
                                camera_lock(cam)
                            locked = not locked
                            status = "Camera settings changed"
                        except Exception:
                            log_error("camera lock")
                            locked = False
                            for restore in (lambda: cam.exp_mode(camera.AeMode.Auto),
                                            lambda: cam.awb_mode(camera.AwbMode.Auto)):
                                try:
                                    restore()
                                except Exception:
                                    log_error("camera auto restore")
                            status = "LOCK FAIL: see console/log"
            stage = "camera read"
            read_start = time.monotonic()
            frame = cam.read()
            timings["camera"] = (time.monotonic() - read_start) * 1000
            if frame is None:
                raise RuntimeError("Camera returned no frame")
            source, target = frame, None
            diagnostics = {"count": 0, "reason": "PLATE_ERROR"}
            stage = "plate detection"
            detect_start = time.monotonic()
            if not fault:
                try:
                    raw, source = detect_plate_color(frame, image, diagnostics, edges_view)
                    target = tracker.update(raw)
                    if raw is not None and target is None:
                        diagnostics["reason"] = "PLATE:%d/%d" % (tracker.count, PLATE_CONFIRM_FRAMES)
                except Exception:
                    log_error(stage)
                    fault = True
                    tracker.reset()
                    source = frame
                    diagnostics = {"count": 0, "reason": "PLATE_ERROR"}
            timings["detect"] = (time.monotonic() - detect_start) * 1000
            now = time.monotonic()
            if now > previous_time:
                measured = 1.0 / (now - previous_time)
                fps = measured if fps == 0 else fps * 0.8 + measured * 0.2
            previous_time = now
            stage = "draw / display"
            uart_status = "OFF" if sender is None else "%d/%d" % (sender.sent, sender.errors)
            view_start = time.monotonic()
            draw_ui(source, target, diagnostics, image, (sw, sh), fps, edges_view,
                    locked, fault, uart_status, status)
            disp.show(source, fit=image.Fit.FIT_CONTAIN)
            timings["view"] = (time.monotonic() - view_start) * 1000
            # 检测和显示成功才发布有效坐标；任何异常均由退出路径发送失效帧。
            if sender is not None:
                sender.send(None if target is None else tangent_alignment_error(target, fw, fh, sw, sh), time.monotonic())
            if SHOW_TIMING and time.monotonic() >= next_timing_log:
                print("[PLATE] FPS:%.1f C:%.1f D:%.1f V:%.1f ms TX:%s %s %s" % (
                    fps, timings["camera"], timings["detect"], timings["view"],
                    uart_status, diagnostics.get("reason", ""), status), flush=True)
                next_timing_log = time.monotonic() + 2.0
            del frame, source
        print("[PLATE] exit requested by app / MaixVision", flush=True)
    except Exception:
        log_error(stage)
        if sender is not None:
            sender.send(None, time.monotonic(), force=True)
        # 启动/显示错误不重复重启摄像头；保留错误画面，等待 MaixVision 停止。
        if disp is not None:
            try:
                error_frame = image.Image(CAM_WIDTH, CAM_HEIGHT, image.Format.FMT_RGB888)
                error_frame.draw_string(4, 8, "PLATE ERROR", image.COLOR_RED)
                error_frame.draw_string(4, 32, stage, image.COLOR_WHITE)
                error_frame.draw_string(4, 62, "Check MaixVision console", image.COLOR_WHITE)
                error_frame.draw_string(4, 88, "~/maixcam_plate_error.log", image.COLOR_WHITE)
                disp.show(error_frame, fit=image.Fit.FIT_CONTAIN)
                while not app.need_exit():
                    time.sleep(0.1)
            except Exception:
                log_error("error display")
    finally:
        light_off_on_exit(light)
        if sender is not None:
            sender.send(None, time.monotonic(), force=True)
        for device in (serial, touch, cam, disp):
            if device is not None:
                try:
                    device.close()
                except Exception:
                    log_error("device close")


def run_photo(path, output, json_path=None):
    import cv2
    import numpy as np
    import json
    # imdecode / imencode 支持 Windows 中文路径。
    frame = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("Cannot read image: " + path)
    frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))
    diagnostics = {}
    result, edges = fit_plate_preview_array(frame, diagnostics, order="BGR")
    if result is not None:
        cv2.ellipse(frame, result["ellipse"], (0, 255, 255), 1)
        tx, ty = round(result["tangent_x"]), round(result["tangent_y"])
        cv2.line(frame, (0, ty), (CAM_WIDTH - 1, ty), (0, 255, 0), 2)
        cv2.line(frame, (tx - 5, ty), (tx + 5, ty), (0, 0, 255), 2)
    coordinates = None if result is None else tangent_screen_coordinates(result, CAM_WIDTH, CAM_HEIGHT, CAM_WIDTH, CAM_HEIGHT)
    # 单张图仅验证拟合，不绕过设备端连续 3 帧确认，不发送串口。
    record = {"input": str(path), "processed_size": [CAM_WIDTH, CAM_HEIGHT],
              "fit_size": [min(PROCESS_WIDTH, CAM_WIDTH), round(CAM_HEIGHT * min(PROCESS_WIDTH, CAM_WIDTH) / CAM_WIDTH)],
              "diagnostics": diagnostics, "tangent_x_image_px": None if result is None else result["tangent_x"],
              "tangent_y_image_px": None if result is None else result["tangent_y"],
              "x_centered_in_processed_image_px": None if coordinates is None else coordinates[0],
              "y_centered_in_processed_image_px": None if coordinates is None else coordinates[1]}
    cv2.putText(frame, diagnostics["reason"], (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    suffix = os.path.splitext(output)[1] or ".png"
    ok, encoded = cv2.imencode(suffix, frame)
    if not ok:
        raise OSError("Cannot encode preview: " + output)
    encoded.tofile(output)
    if json_path:
        with open(json_path, "w", encoding="utf-8") as stream:
            json.dump(record, stream, ensure_ascii=False, indent=2)
    print(json.dumps(record, ensure_ascii=False, indent=2), flush=True)


def main():
    import sys
    if len(sys.argv) > 1:
        import argparse
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--image", required=True)
        parser.add_argument("--output", default="plate_tangent_result.png")
        parser.add_argument("--json", default=None)
        args = parser.parse_args()
        run_photo(args.image, args.output, args.json)
        return
    # 原生崩溃不进入 Python except，额外保留 faulthandler 栈。
    fatal_stream = None
    try:
        import faulthandler
        fatal_stream = open(FATAL_LOG_PATH, "w", encoding="utf-8")
        faulthandler.enable(file=fatal_stream)
    except (ImportError, OSError):
        print("[PLATE] native crash log unavailable", flush=True)
    try:
        run_device()
    finally:
        if fatal_stream is not None:
            faulthandler.disable()
            fatal_stream.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        log_error("entry")
        raise
