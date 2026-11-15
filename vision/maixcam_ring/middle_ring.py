"""MaixCAM 俯视三组同心圆：识别/锁定中间组，串口发送视觉纠偏误差。

独立上传本文件即可运行，需要 MaixPy v4 的 cv2/numpy。
电脑照片验证：python middle_ring.py --image photo.jpg --output result.png
"""
import argparse
import itertools
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 480, 360, 30
RING_BUILD = '2026-10-15-occupied-resources-v18'
ADAPTIVE_BLOCK = 25
ADAPTIVE_C = 7
WEAK_MIDDLE_RECOVERY_ENABLED = True
OCCUPIED_RING_RECOVERY_ENABLED = True
MIN_ELLIPSE_AXIS = 10.0
MIN_ELLIPSE_ASPECT = 0.35
MAX_FIT_ERROR = 0.10
MIN_CONTOUR_FILL = 0.78
MIN_RING_LEVELS = 3
MIN_RADIUS_SPAN = 0.18
MIN_PARTIAL_ARC_DEG = 100.0
MAX_PARTIAL_FIT_ERROR = 0.04
MAX_PARTIAL_FIT_P95 = 0.08
PARTIAL_BORDER_MARGIN = 2
MAX_CONTOURS = 350
MOMENT_ELLIPSES_ENABLED = True  # 闭合轮廓面积矩，避免逐环通用椭圆求解；False 可对比旧路径。
ACQUIRE_FRAMES = 3
TRACK_MEMORY_S = 0.35
TRACK_MEMORY_MAX_S = 1.0  # 仅用于身份关联，绝不复用旧坐标输出。
PREVIEW_ENABLED = True
PREVIEW_FPS = 5
PREVIEW_JPEG_QUALITY = 75
PERF_LOG_INTERVAL_S = 2.0
ROI_TRACKING_ENABLED = True
GLOBAL_VERIFY_INTERVAL_S = 1.0
ROI_TEMPLATE_MIN_SCORE = 0.86
MOTION_PREDICTION_ENABLED = True
POSITION_SMOOTHING_ENABLED = True
PREDICTION_HORIZON_S = 0.15
SMOOTH_MAX_LAG_PX = 1.0  # 滤波偏离当前实测值最多1图像像素。
ALIGN_TOLERANCE_SCREEN_PX = 5.0
ALIGN_STABLE_FRAMES = 5
# 用户指定屏幕中心；可配置中心的偏置，但当前为零。
TARGET_OFFSET_SCREEN_PX = (0.0, 0.0)
UART_ENABLED = True
UART_PORT = "/dev/ttyS0"
UART_PIN_FUNCTIONS = {"A16": "UART0_TX", "A17": "UART0_RX"}
UART_BAUDRATE, UART_SEND_HZ = 115200, 20




# LIGHT_GPIO_BEGIN
LIGHT_PIN = "A19"
LIGHT_GPIO = "GPIOA19"
LIGHT_OFF_LEVEL = 0  # 按高电平使能灯板；低电平使能板需改为1。
LIGHT_PWM_ENABLED = False  # 由各入口配置 GPIO/PWM；亮度为灯板占空比。
LIGHT_PWM_ID, LIGHT_PWM_FREQ_HZ = 7, 20000
LIGHT_BRIGHTNESS_PERCENT = 50.0


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

def image_fit_geometry(fw, fh, sw, sh):
    scale = min(sw / fw, sh / fh)
    return (sw - fw * scale) / 2, (sh - fh * scale) / 2, scale


def screen_error(center, frame_size, screen_size):
    fw, fh = frame_size
    sw, sh = screen_size
    left, top, scale = image_fit_geometry(fw, fh, sw, sh)
    sx, sy = left + center[0] * scale, top + center[1] * scale
    return (sx - sw / 2 - TARGET_OFFSET_SCREEN_PX[0],
            sh / 2 - sy - TARGET_OFFSET_SCREEN_PX[1])


def moment_ellipse(moments):
    """闭合椭圆内部协方差的特征值为 a²/4、b²/4，标量解 2×2。

    使用面积矩而非边界采样密度；边界缺损/非椭圆仍须通过后续 fill/residual 检查。
    """
    area = moments["m00"]
    if area <= 0:
        return None
    xx, xy, yy = moments["mu20"] / area, moments["mu11"] / area, moments["mu02"] / area
    gap = math.hypot(xx - yy, 2 * xy)
    major, minor = (xx + yy + gap) / 2, (xx + yy - gap) / 2
    if minor <= 0 or not all(math.isfinite(v) for v in (major, minor)):
        return None
    center = moments["m10"] / area, moments["m01"] / area
    axes = 4 * math.sqrt(major), 4 * math.sqrt(minor)
    angle = math.degrees(0.5 * math.atan2(2 * xy, xx - yy))
    return center, axes, angle


def ellipse_candidate(contour, shape, bounds=None, area=None):
    h, w = shape
    if len(contour) < 12:
        return None
    x, y, bw, bh = bounds if bounds is not None else cv2.boundingRect(contour)
    if min(bw, bh) < MIN_ELLIPSE_AXIS or max(bw, bh) > min(h, w) * 0.85:
        return None
    if x <= 0 or y <= 0 or x + bw >= w or y + bh >= h:
        return None
    area = abs(cv2.contourArea(contour)) if area is None else area
    # 允许的椭圆轴比 >=0.35，即使旋转后 bbox 填充率也足够；先排除稀疏杂线。
    if area < 60 or area < bw * bh * 0.40:
        return None
    ellipse = moment_ellipse(cv2.moments(contour)) if MOMENT_ELLIPSES_ENABLED else cv2.fitEllipse(contour)
    if ellipse is None:
        return None
    (cx, cy), (ew, eh), angle = ellipse
    if min(ew, eh) < MIN_ELLIPSE_AXIS or min(ew, eh) / max(ew, eh) < MIN_ELLIPSE_ASPECT:
        return None
    fill = area / (math.pi * ew * eh / 4)
    if not MIN_CONTOUR_FILL <= fill <= 1.12:
        return None
    points = contour[::max(1, math.ceil(len(contour) / 64)), 0, :].astype(np.float32)
    theta = math.radians(angle)
    dx, dy = points[:, 0] - cx, points[:, 1] - cy
    ex = dx * math.cos(theta) + dy * math.sin(theta)
    ey = -dx * math.sin(theta) + dy * math.cos(theta)
    residual = np.abs(np.sqrt((ex / (ew / 2)) ** 2 + (ey / (eh / 2)) ** 2) - 1)
    error = float(np.mean(residual))
    if not math.isfinite(error) or error > MAX_FIT_ERROR:
        return None
    return {"center": (float(cx), float(cy)), "radius": (ew + eh) / 4,
            "ellipse": ellipse, "error": error, "partial": False, "arc_deg": 360.0}


def partial_normalization(reference=None):
    if reference is None:
        ratios, angle = (1.0, 1.0), 0.0
    else:
        _, (ew, eh), angle = reference["ellipse"]
        average_radius = (ew + eh) / 4
        ratios = ew / (2 * average_radius), eh / (2 * average_radius)
    theta = math.radians(angle)
    cs, sn = math.cos(theta), math.sin(theta)
    # 旋转与轴缩放的逆可直接写出，无需在每条弧上调用 LAPACK。
    transform = (cs / ratios[0], sn / ratios[0], -sn / ratios[1], cs / ratios[1])
    inverse = (ratios[0] * cs, -ratios[1] * sn, ratios[0] * sn, ratios[1] * cs)
    extents = math.hypot(ratios[0] * cs, ratios[1] * sn), math.hypot(ratios[0] * sn, ratios[1] * cs)
    return transform, inverse, ratios, angle, extents


def arc_coverage_degrees(dx, dy):
    """连续、单向弧用两端求覆盖角；有回折时回退到完整角排序。"""
    cross = dx[:-1] * dy[1:] - dy[:-1] * dx[1:]
    positive, negative = int(np.count_nonzero(cross > 0)), int(np.count_nonzero(cross < 0))
    nonzero = positive + negative
    if nonzero >= 3 and max(positive, negative) >= 0.90 * nonzero:
        direction = 1 if positive > negative else -1
        start, end = math.atan2(float(dy[0]), float(dx[0])), math.atan2(float(dy[-1]), float(dx[-1]))
        coverage = ((end - start) * direction) % (2 * math.pi)
        if coverage > 1e-3:
            return math.degrees(coverage)
    angles = np.sort(np.mod(np.arctan2(dy, dx), 2 * math.pi))
    gaps = np.diff(np.r_[angles, angles[0] + 2 * math.pi])
    return math.degrees(2 * math.pi - float(gaps.max()))


def clipped_arc_candidates(contour, shape, reference=None, normalization=None, bounds=None):
    """删掉裁切直边，利用可见弧拟合；完整组提供共同的椭圆比例。

    在参考椭圆的归一化平面中拟合圆，避免对半环直接自由拟合 5 参数椭圆。
    没有完整参考组时按用户的俯视条件使用圆形模型。
    """
    h, w = shape
    raw = contour[:, 0, :]
    if len(raw) < 24:
        return []
    margin = PARTIAL_BORDER_MARGIN
    x, y, bw, bh = bounds if bounds is not None else cv2.boundingRect(contour)
    borders = (x <= margin, x + bw >= w - margin,
               y <= margin, y + bh >= h - margin)
    if not any(borders):
        return []
    # 大背景/极细直线无法形成当前尺寸范围的单个圆环，先阻止昂贵的分段计算。
    if min(bw, bh) < MIN_ELLIPSE_AXIS / 2 or max(bw, bh) > min(h, w) * 0.95:
        return []
    if reference is not None and max(bw, bh) > max(reference["ellipse"][1]) * 1.8:
        return []
    keep = ((raw[:, 0] > margin) & (raw[:, 0] < w - 1 - margin) &
            (raw[:, 1] > margin) & (raw[:, 1] < h - 1 - margin))
    transitions = np.flatnonzero(np.diff(np.r_[False, keep, False].astype(np.int8)))
    runs = [raw[start:end] for start, end in zip(transitions[::2], transitions[1::2])]
    if len(runs) > 1 and keep[0] and keep[-1]:
        runs = [np.vstack((runs[-1], runs[0]))] + runs[1:-1]
    runs = sorted((run for run in runs if len(run) >= 24), key=len, reverse=True)[:4]
    if not runs:
        return []
    transform, inverse, axis_ratios, angle, extents = normalization or partial_normalization(reference)
    results = []
    for run in runs:
        if len(run) < 24:
            continue
        # ceil 才能真正保持 <=100；此前 100~199 点会全部进入计算。
        # RV64 嵌入式上避免整条弧用 float64 做逐点超越函数；关键解保留标量精度。
        points = run[::max(1, math.ceil(len(run) / 100))].astype(np.float32)
        nx = points[:, 0] * transform[0] + points[:, 1] * transform[1]
        ny = points[:, 0] * transform[2] + points[:, 1] * transform[3]
        mean_x, mean_y = float(nx.mean()), float(ny.mean())
        scale = max(float(np.ptp(nx)), float(np.ptp(ny)), 1)
        px, py = (nx - mean_x) / scale, (ny - mean_y) / scale
        xx, yy, xy = px * px, py * py, px * py
        zz = xx + yy
        aa, bb, cc = float(xx.sum()), float(xy.sum()), float(yy.sum())
        dd, ee = float((px * zz).sum()) / 2, float((py * zz).sum()) / 2
        determinant = aa * cc - bb * bb
        if aa <= 0 or cc <= 0 or determinant <= 1e-8 * aa * cc:
            continue
        # 与原来中心化后的圆最小二乘同一解，仅求解 2×2 正规方程。
        local_x = (dd * cc - bb * ee) / determinant
        local_y = (aa * ee - bb * dd) / determinant
        radius_sq = local_x * local_x + local_y * local_y + float(zz.mean())
        center_nx, center_ny = local_x * scale + mean_x, local_y * scale + mean_y
        radius = math.sqrt(float(radius_sq)) * scale
        center = (inverse[0] * center_nx + inverse[1] * center_ny,
                  inverse[2] * center_nx + inverse[3] * center_ny)
        if not all(math.isfinite(v) for v in center) or not MIN_ELLIPSE_AXIS / 2 <= radius <= min(h, w) * 0.425:
            continue
        if not (-0.65 * radius <= center[0] <= w - 1 + 0.65 * radius and
                -0.65 * radius <= center[1] <= h - 1 + 0.65 * radius):
            continue
        dx, dy = nx - center_nx, ny - center_ny
        residual = np.abs(np.hypot(dx, dy) / radius - 1)
        error = float(np.mean(residual))
        p95_index = math.ceil(0.95 * (len(residual) - 1))
        if error > MAX_PARTIAL_FIT_ERROR or np.partition(residual, p95_index)[p95_index] > MAX_PARTIAL_FIT_P95:
            continue
        coverage = arc_coverage_degrees(dx, dy)
        if coverage < MIN_PARTIAL_ARC_DEG:
            continue
        a, b = radius * axis_ratios[0], radius * axis_ratios[1]
        extent_x, extent_y = radius * extents[0], radius * extents[1]
        # 拟合出的虚拟圆也必须越过相应画面边界，排除边框/直线误拟合。
        crosses = (center[0] - extent_x <= margin + 1,
                   center[0] + extent_x >= w - 2 - margin,
                   center[1] - extent_y <= margin + 1,
                   center[1] + extent_y >= h - 2 - margin)
        if not any(touched and crossed for touched, crossed in zip(borders, crosses)):
            continue
        center = tuple(float(v) for v in center)
        results.append({"center": center, "radius": radius,
                        "ellipse": (center, (2 * a, 2 * b), angle), "error": error,
                        "partial": True, "arc_deg": coverage})
    return results


def group_concentric(ellipses):
    clusters = []
    for candidate in sorted(ellipses, key=lambda c: c["radius"], reverse=True):
        chosen = None
        for cluster in clusters:
            anchor = cluster[0]
            distance = math.dist(candidate["center"], anchor["center"])
            ratio = candidate["radius"] / anchor["radius"]
            tolerance = 0.10 if anchor.get("partial") or candidate.get("partial") else 0.07
            if distance <= max(2.0, anchor["radius"] * tolerance) and 0.42 <= ratio <= 1.05:
                chosen = cluster
                break
        if chosen is None:
            clusters.append([candidate])
        else:
            chosen.append(candidate)
    groups = []
    for cluster in clusters:
        largest = cluster[0]
        radii = sorted(c["radius"] for c in cluster)
        levels = []
        tolerance = max(0.7, largest["radius"] * 0.025)
        for radius in radii:
            if not levels or radius - levels[-1] > tolerance:
                levels.append(radius)
        span = (radii[-1] - radii[0]) / radii[-1]
        if len(levels) < MIN_RING_LEVELS or span < MIN_RADIUS_SPAN:
            continue
        partial = any(c.get("partial", False) for c in cluster)
        # 同一被裁切轮廓可能产生多段弧，不允许它伪装成三层同心环。
        sources = {c["source"] for c in cluster if "source" in c}
        if partial and len(sources) < MIN_RING_LEVELS:
            continue
        weights = [c["radius"] ** 2 * min(c.get("arc_deg", 360) / 360, 1) for c in cluster]
        total_weight = sum(weights)
        center = (sum(weight * c["center"][0] for weight, c in zip(weights, cluster)) / total_weight,
                  sum(weight * c["center"][1] for weight, c in zip(weights, cluster)) / total_weight)
        mean_error = sum(c["error"] for c in cluster) / len(cluster)
        score = min(1.0, 0.55 + 0.05 * len(levels) + 0.15 * (1 - mean_error / MAX_FIT_ERROR))
        score -= 0.06 * sum(c.get("partial", False) for c in cluster) / len(cluster)
        groups.append({"center": tuple(float(v) for v in center),
                       "radius": float(largest["radius"]), "levels": len(levels),
                       "ellipse": (tuple(center), largest["ellipse"][1], largest["ellipse"][2]),
                       "score": score, "partial": partial})
    return sorted(groups, key=lambda g: g["score"], reverse=True)[:8]


def choose_middle(groups):
    """用三组（可含残弧补全）的等距/共线关系确定中间组。"""
    choices = []
    for trio in itertools.combinations(groups, 3):
        pairs = [(math.dist(trio[i]["center"], trio[j]["center"]), i, j)
                 for i, j in ((0, 1), (0, 2), (1, 2))]
        distance, first, last = max(pairs)
        middle = 3 - first - last
        p, q, m = trio[first]["center"], trio[last]["center"], trio[middle]["center"]
        direction = q[0] - p[0], q[1] - p[1]
        if distance <= 1:
            continue
        t = ((m[0] - p[0]) * direction[0] + (m[1] - p[1]) * direction[1]) / (distance * distance)
        perpendicular = math.hypot(m[0] - p[0] - t * direction[0], m[1] - p[1] - t * direction[1])
        d1, d2 = math.dist(m, p), math.dist(m, q)
        radii = [g["radius"] for g in trio]
        if not 0.34 <= t <= 0.66 or perpendicular > distance * 0.055:
            continue
        if min(radii) / max(radii) < 0.60 or min(d1, d2) / max(d1, d2) < 0.60:
            continue
        mean_radius = sum(radii) / 3
        # 规则中心距 150 mm，外圈直径约 95 mm；容忍不同可见环层。
        if not 2.2 <= (d1 + d2) / (2 * mean_radius) <= 5.2:
            continue
        angle = math.degrees(math.atan2(-float(direction[1]), float(direction[0])))
        angle = (angle + 90) % 180 - 90
        score = sum(g["score"] for g in trio) / 3 - 0.5 * abs(t - 0.5) - perpendicular / distance
        choices.append({"ring": trio[middle], "triple": trio, "angle": angle, "score": score})
    choices.sort(key=lambda choice: choice["score"], reverse=True)
    if not choices:
        return None, "NEED_THREE"
    if len(choices) > 1:
        a, b = choices[:2]
        if a["score"] - b["score"] < 0.08 and math.dist(a["ring"]["center"], b["ring"]["center"]) > a["ring"]["radius"]:
            return None, "AMBIGUOUS"
    return choices[0], "THREE_FOUND"


def ring_axis_measurement(groups, triple, observation):
    """本帧三圆心的图像行方向，x右/y上，180度周期；不是车体航向误差。

    零位与车体方向/比例由主控停车标定。仅复用现有圆心，无新轮廓或图像算子。
    """
    if len(groups)!=3 or triple is None or observation is None or not observation.get('angle_valid'):
        return None
    if any(g.get('boundary_verified') or g.get('levels',0)<3 for g in groups):
        return None  # 两层边界补证可定位，但不用于精确角度控制。
    try:
        points=[(float(g['center'][0]),-float(g['center'][1])) for g in groups]
        scores=[float(g['score']) for g in groups]
        if not all(math.isfinite(v) for p in points for v in p) or not all(math.isfinite(q) and q>=.75 for q in scores):
            return None
        span=max(math.dist(a,b) for a,b in itertools.combinations(points,2))
        if span<80:
            return None
        mx,my=sum(p[0] for p in points)/3,sum(p[1] for p in points)/3
        xx=sum((p[0]-mx)**2 for p in points)
        yy=sum((p[1]-my)**2 for p in points)
        xy=sum((p[0]-mx)*(p[1]-my) for p in points)
        if xx+yy<=0:
            return None
        angle=.5*math.atan2(2*xy,xx-yy)
        cs,sn=math.cos(angle),math.sin(angle)
        deviation=max(abs(-(p[0]-mx)*sn+(p[1]-my)*cs) for p in points)
        if deviation>max(1.25,.0125*span):
            return None
        degrees=(math.degrees(angle)+90)%180-90
        confidence=max(0,min(100,round(100*min(scores)-500*deviation/span)))
        if confidence<75:
            return None
        return dict(degrees=degrees,confidence=confidence,source=3)
    except (TypeError,KeyError,ValueError,OverflowError):
        return None


def detect_rings(gray, diagnostics=None, roi=None, reference=None, hints=None, occupied=None):
    if gray.ndim != 2 or gray.dtype != np.uint8:
        raise ValueError("Expected uint8 grayscale image")
    start = time.monotonic() if diagnostics is not None else 0.0
    # 规则线宽 1.5 mm，低分辨率下很细；默认不模糊，避免相邻环层粘连。
    h, w = gray.shape
    left, top, right, bottom = (0, 0, w, h) if roi is None else roi
    region = gray[top:bottom, left:right]
    local_binary = cv2.adaptiveThreshold(region, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                         cv2.THRESH_BINARY_INV, ADAPTIVE_BLOCK, ADAPTIVE_C)
    if roi is None:
        binary = local_binary
    else:
        binary = np.zeros_like(gray)
        binary[top:bottom, left:right] = local_binary
    threshold_end = time.monotonic() if diagnostics is not None else 0.0
    raw_contours = cv2.findContours(local_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE,
                                  offset=(left, top))[-2]
    # 有界候选量，避免脏背景中的成千上万个轮廓拖低帧率。
    # 这些小碎片原本也无法通过 len>=12，排序前过滤，避免大量无效 C++ 调用。
    def metadata(contour):
        if len(contour) < 12:
            return None
        bounds = cv2.boundingRect(contour)
        x, y, bw, bh = bounds
        if roi is not None:
            # ROI 裁切不是相机裁切：不要补全窗口人为切断的对象。
            if ((left > 0 and x <= left + 2) or (top > 0 and y <= top + 2) or
                (right < w and x + bw >= right - 2) or (bottom < h and y + bh >= bottom - 2)):
                return None
        return contour, bounds, abs(cv2.contourArea(contour))
    contours = sorted((m for c in raw_contours for m in [metadata(c)] if m is not None),
                      key=lambda m: m[2], reverse=True)[:MAX_CONTOURS]
    contour_end = time.monotonic() if diagnostics is not None else 0.0
    ellipses = []
    border_contours = []
    for source, (contour, bounds, area) in enumerate(contours):
        x, y, bw, bh = bounds
        margin = PARTIAL_BORDER_MARGIN
        if x <= margin or y <= margin or x + bw >= w - margin or y + bh >= h - margin:
            border_contours.append((source, contour, bounds))
        candidate = ellipse_candidate(contour, gray.shape, bounds, area)
        if candidate is not None:
            candidate["source"] = source
            ellipses.append(candidate)
    intact = group_concentric(ellipses)
    # 场地只有三组：三组完整几何证据已成立时，不再拟合边框/地面残弧。
    intact_middle, intact_status = choose_middle(intact) if len(intact) == 3 else (None, None)
    reference = intact[0] if intact else reference
    normalization = partial_normalization(reference) if border_contours and intact_middle is None else None
    full_end = time.monotonic() if diagnostics is not None else 0.0
    partial_count = 0
    # 只对触及边界的轮廓补全，不扩大为全图 Hough 搜索。
    for source, contour, bounds in (border_contours if intact_middle is None else []):
        for candidate in clipped_arc_candidates(contour, gray.shape, reference, normalization, bounds):
            candidate["source"] = source
            ellipses.append(candidate)
            partial_count += 1
    partial_end = time.monotonic() if diagnostics is not None else 0.0
    groups = group_concentric(ellipses) if partial_count else intact
    middle, status = (intact_middle, intact_status) if intact_middle is not None else choose_middle(groups)
    if WEAK_MIDDLE_RECOVERY_ENABLED and len(groups) == 2 and middle is None:
        recovered = recover_weak_middle(gray, groups)
        if recovered is not None:
            trial = groups + [recovered]
            selection, selection_status = choose_middle(trial)
            if selection is not None and selection['ring'] is recovered:
                groups, middle, status = trial, selection, selection_status
    if OCCUPIED_RING_RECOVERY_ENABLED and occupied is not False and middle is None and status != 'AMBIGUOUS':
        groups = recover_occupied_rings(gray, contours, groups, partial_normalization,
            group_concentric, fit_occupied_circle, clipped_arc_candidates, roi, hints)
        middle, status = choose_middle(groups)
    if diagnostics is not None:
        group_end = time.monotonic()
        diagnostics.clear()
        diagnostics.update(B=1000*(threshold_end-start), K=1000*(contour_end-threshold_end),
                           E=1000*(full_end-contour_end), P=1000*(partial_end-full_end),
                           G=1000*(group_end-partial_end), raw=len(raw_contours),
                           kept=len(contours), arcs=partial_count, weak=sum(g.get('weak_recovered',False) for g in groups),
                           occupied=sum(g.get('occluded',False) for g in groups),
                           mode="FULL" if roi is None else "ROI")
    return groups, middle, binary, status


def recover_weak_middle(gray, groups, _seed=None, _refined=False):
    """只在两侧环尺度/间距符合规则时局部测弱中环；中点仅作搜索初值。

    必须在当前图像重新测到>=3层、各层>=75%方向有足够暗线对比，
    通过残差/同心/半径跨度/三环布局后才使用，不把几何中点当测量。
    """
    if len(groups) != 2:
        return None
    first, last = groups
    distance = math.dist(first['center'], last['center'])
    mean_radius = (first['radius'] + last['radius']) / 2
    if not 5.0 <= distance / max(mean_radius, 1) <= 8.0:
        return None  # 相邻两环不可把中点误作第三环。
    cx, cy = tuple((first['center'][i]+last['center'][i])/2 for i in (0,1))
    if _seed is not None:
        cx,cy=_seed  # 重测初值只来自上一轮合格的当前环线，不来自历史缓存。
    # 两端间距300mm、外径95mm。半径初值来自规则尺度，不要求强环外层都分离。
    radius = distance * 95.0 / 600.0
    if not 12 <= radius <= min(gray.shape)*.35:
        return None
    reference = first if not first.get('partial') else last
    transform, inverse, ratios, angle, extents = partial_normalization(reference)
    h,w = gray.shape
    if cx-radius*extents[0] < 3 or cy-radius*extents[1] < 3 or cx+radius*extents[0] >= w-3 or cy+radius*extents[1] >= h-3:
        return None  # 本分支只恢复画面内弱中环；边界中环沿用原残弧逻辑。
    angles = np.arange(0,360,4,dtype=np.float32)*(math.pi/180)
    radial = np.arange(.62*radius,1.07*radius,max(.3,radius/110),dtype=np.float32)
    nx,ny = np.cos(angles)[:,None]*radial, np.sin(angles)[:,None]*radial
    mx = (cx+inverse[0]*nx+inverse[1]*ny).astype(np.float32)
    my = (cy+inverse[2]*nx+inverse[3]*ny).astype(np.float32)
    values = cv2.remap(gray,mx,my,cv2.INTER_LINEAR).astype(np.float32)
    dip = cv2.blur(values,(13,1))-values  # 仅处理小采样阵列，不降低全图二值化阈值。
    candidates=[]
    rows=np.arange(len(angles))
    for layer,fraction in enumerate((65/95,75/95,85/95,1.0)):
        band=np.flatnonzero(np.abs(radial-fraction*radius) <= max(1.5,.045*radius))
        if len(band)<3:continue
        indices=band[np.argmax(dip[:,band],axis=1)]
        strength=dip[rows,indices]
        selected=strength>=5.0
        if float(selected.mean()) < .75 or float(np.median(strength[selected])) < 8.0:
            continue
        sx,sy=nx[rows,indices][selected],ny[rows,indices][selected]
        active=np.ones(len(sx),dtype=bool)
        fitted=None
        for iteration in range(3):
            fitted=fit_sample_circle(sx[active],sy[active])
            if fitted is None:break
            fx,fy,fitted_radius=fitted
            absolute=np.abs(np.hypot(sx-fx,sy-fy)-fitted_radius)
            if iteration == 2:break
            updated=absolute<=max(.7,min(1.5,3*float(np.median(absolute[active]))))
            if int(updated.sum()) < len(angles)*.70:break
            active=updated
        if fitted is None or int(active.sum()) < len(angles)*.70:continue
        fx,fy,fitted_radius=fitted
        residual=np.abs(np.hypot(sx[active]-fx,sy[active]-fy)/fitted_radius-1)
        center=(cx+inverse[0]*fx+inverse[1]*fy,cy+inverse[2]*fx+inverse[3]*fy)
        error=float(residual.mean())
        if (error>.025 or float(np.percentile(residual,95))>.055
                or math.dist(center,(cx,cy))>max(4,radius*.12)
                or abs(fitted_radius-fraction*radius)>radius*.07):
            continue
        candidates.append(dict(center=center,radius=fitted_radius,error=error,source=layer,
            ellipse=(center,(2*fitted_radius*ratios[0],2*fitted_radius*ratios[1]),angle)))
    recovered=group_concentric(candidates)
    if len(recovered)!=1:
        # 初值偏几像素时可能选中相邻线；以本帧可靠外层中心最多重测一次。
        if candidates and not _refined:
            outer=max(candidates,key=lambda c:c['radius'])
            return recover_weak_middle(gray,groups,outer['center'],True)
        return None
    result=recovered[0]
    result['weak_recovered']=True
    return result


def fit_sample_circle(x,y):
    """OpenCV原生归约/乘法形成2x2方程，标量求解；不调用SVD/LAPACK。"""
    if len(x)<12:return None
    points=np.column_stack((x,y)).astype(np.float32,copy=False)
    mean_x,mean_y=cv2.mean(points.reshape(-1,1,2))[:2]
    centered=points-np.asarray((mean_x,mean_y),dtype=np.float32)
    zz=cv2.reduce(centered*centered,1,cv2.REDUCE_SUM)
    gram=cv2.gemm(centered,centered,1.,None,0.,flags=cv2.GEMM_1_T)
    rhs=cv2.gemm(centered,zz,.5,None,0.,flags=cv2.GEMM_1_T)
    aa,bb,cc=float(gram[0,0]),float(gram[0,1]),float(gram[1,1])
    dd,ee=float(rhs[0,0]),float(rhs[1,0])
    determinant=aa*cc-bb*bb
    if aa<=0 or cc<=0 or determinant<=1e-8*aa*cc:return None
    ox,oy=(dd*cc-bb*ee)/determinant,(aa*ee-bb*dd)/determinant
    radius=math.sqrt(ox*ox+oy*oy+cv2.mean(zz)[0])
    return mean_x+ox,mean_y+oy,radius


# BEGIN OCCUPIED_RING
def occupied_sample_group(gray, seed, normalize, group_layers, fit_circle, refined=False, sampled=None):
    started=time.monotonic()
    occupied_sample_group.last_profile=None
    cx, cy = seed['center']
    radius = seed.get('sampling_radius', seed['radius'])
    h, w = gray.shape
    if not 12 <= radius <= min(h, w)*.425:
        return None
    if not (-.65*radius <= cx <= w-1+.65*radius and -.65*radius <= cy <= h-1+.65*radius):
        return None
    _, inverse, ratios, angle, extents = normalize(seed)
    cropped = cx-radius*extents[0] < 0 or cx+radius*extents[0] >= w or cy-radius*extents[1] < 0 or cy+radius*extents[1] >= h
    boundary_context=bool(seed.get('boundary_verified')) and sum((cx-radius*extents[0]<0,cx+radius*extents[0]>=w,cy-radius*extents[1]<0,cy+radius*extents[1]>=h))==1
    if sampled is None:
        angles,radial,nx,ny,mx,my=occupied_sampling_grid(seed,normalize)
        values=cv2.remap(gray,mx,my,cv2.INTER_LINEAR).astype(np.float32)
    else:
        angles,radial,nx,ny,mx,my,values=sampled
    ray_scale=len(angles)/90
    mapped=time.monotonic()
    # 保证暗线对比窗口全在相机内，禁止把 remap 黑边当成圆弧。
    inside = ((mx > 2) & (mx < w-3) & (my > 2) & (my < h-3)).astype(np.uint8)
    kernel=getattr(occupied_sample_group,'valid_kernel',None)
    if kernel is None:
        kernel=np.ones((1,13),np.uint8);occupied_sample_group.valid_kernel=kernel
    valid = cv2.erode(inside, kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    smooth = cv2.blur(np.vstack((values[-1:], values, values[:1])), (1, 3))[1:-1]
    noise = occupied_median(np.abs(values-smooth)[valid])/.55 if valid.any() else 0.0
    values = smooth
    dip = cv2.blur(values, (13, 1))-values
    dip[~valid] = -1000
    signaled=time.monotonic()
    candidates = []
    row_cache=getattr(occupied_sample_group,'row_cache',None)
    if row_cache is None:
        row_cache={};occupied_sample_group.row_cache=row_cache
    if len(angles) not in row_cache:row_cache[len(angles)]=np.arange(len(angles))
    rows=row_cache[len(angles)]
    for layer, fraction in enumerate((65/95, 75/95, 85/95, 1.0)):
        band = np.flatnonzero(np.abs(radial-fraction*radius) <= max(1.5, .045*radius))
        if len(band) < 3:
            continue
        available = valid[:, band].any(axis=1)
        possible = int(available.sum())
        if possible < math.ceil(((24 if boundary_context else 28) if cropped else 90)*ray_scale):
            continue
        indices = band[np.argmax(dip[:, band], axis=1)]
        strength = dip[rows, indices]
        selected = (strength >= max(5, 2*noise)) & available
        support = .70 if cropped else .60
        minimum = max(math.ceil((22 if boundary_context else 26)*ray_scale), math.ceil(possible*(.65 if cropped else .55)))
        if int(selected.sum()) < possible*support or occupied_median(strength[selected]) < max(8, 3*noise):
            continue
        sx, sy = nx[rows, indices][selected], ny[rows, indices][selected]
        active = np.ones(len(sx), dtype=bool)
        fitted = None
        for iteration in range(3):
            fitted = fit_circle(sx[active], sy[active])
            if fitted is None:
                break
            fx, fy, fitted_radius = fitted
            absolute = np.abs(np.hypot(sx-fx, sy-fy)-fitted_radius)
            if iteration == 2:
                break
            updated = absolute <= max(.7, min(1.5, 3*occupied_median(absolute[active])))
            if int(updated.sum()) < minimum:
                break
            if np.array_equal(updated,active):
                break  # 内点集合不再变化；重复解相同方程不会改变结果。
            active = updated
        if fitted is None or int(active.sum()) < minimum:
            continue
        fx, fy, fitted_radius = fitted
        residual = np.abs(np.hypot(sx[active]-fx, sy[active]-fy)/fitted_radius-1)
        center = (cx+inverse[0]*fx+inverse[1]*fy, cy+inverse[2]*fx+inverse[3]*fy)
        error = float(residual.mean())
        # 覆盖角按实际有效点最大缺口计算，而非用离散杂点冒充一圈。
        # Sampling angles and boolean indexing already preserve ascending order.
        supported_angles = angles[selected][active]
        gap=float((supported_angles[1:]-supported_angles[:-1]).max()) if len(supported_angles)>1 else 0.
        wrap=float(supported_angles[0]+2*math.pi-supported_angles[-1])
        coverage = math.degrees(2*math.pi-max(gap,wrap))
        measured_crop = (center[0]-fitted_radius*extents[0] < 0 or center[0]+fitted_radius*extents[0] >= w
                         or center[1]-fitted_radius*extents[1] < 0 or center[1]+fitted_radius*extents[1] >= h)
        p95=float(np.partition(residual,math.ceil(.95*(len(residual)-1)))[math.ceil(.95*(len(residual)-1))])
        if (error > .025 or p95 > .055
                or math.dist(center, (cx, cy)) > max(4, radius*(.25 if seed.get('coarse') else .12))
                or abs(fitted_radius-fraction*radius) > radius*.07
                or coverage < ((84 if boundary_context else 100) if measured_crop else 210)
                or (not measured_crop and (int(selected.sum()) < math.ceil(54*ray_scale) or int(active.sum()) < math.ceil(50*ray_scale)))):
            continue
        candidates.append(dict(center=center, radius=fitted_radius, error=error, source=layer, sample_fraction=fraction,
            partial=measured_crop, arc_deg=coverage,
            ellipse=(center, (2*fitted_radius*ratios[0], 2*fitted_radius*ratios[1]), angle)))
    fitted_at=time.monotonic()
    occupied_sample_group.last_profile=dict(map=1000*(mapped-started),signal=1000*(signaled-mapped),fit=1000*(fitted_at-signaled))
    recovered = group_layers(candidates)
    if len(recovered)!=1 and boundary_context:
        # 仅已由两个当前完整环确定第三组位置时，允许边界外圈85/95两层证明该组。
        # 一条粗线的内外边缘不能冒充两层；普通独立单双圈仍由原三层门槛拒绝。
        outer_layers=[c for c in candidates if c['source'] in (2,3) and c['partial']]
        if len(outer_layers)==2:
            inner,outer=sorted(outer_layers,key=lambda c:c['radius'])
            if (.86 <= inner['radius']/outer['radius'] <= .93
                    and math.dist(inner['center'],outer['center']) <= max(1.5,.035*radius)
                    and math.dist(outer['center'],(cx,cy)) <= max(3,.06*radius)
                    and .93 <= outer['radius']/radius <= 1.07):
                center=tuple((inner['center'][i]+outer['center'][i])/2 for i in (0,1))
                recovered=[dict(center=center,radius=outer['radius'],levels=2,
                    ellipse=(center,outer['ellipse'][1],outer['ellipse'][2]),score=.80,
                    partial=True,boundary_verified=True)]
    if len(recovered) != 1:
        if candidates and not refined:
            new = dict(seed)
            new['center'] = max(candidates, key=lambda c: c['radius'])['center']
            return occupied_sample_group(gray, new, normalize, group_layers, fit_circle, True)
        return None
    result = recovered[0]
    rx, ry = result['center']
    outer = result['radius']
    if not (-.65*outer <= rx <= w-1+.65*outer and -.65*outer <= ry <= h-1+.65*outer):
        return None
    result['occluded'] = True
    # 外层暂时失去对比时，不能把下一层误作新的物理外径，逐帧缩小搜索圈。
    result['sampling_radius'] = occupied_median([c['radius']/c['sample_fraction'] for c in candidates])
    return result


def fit_occupied_circle(x,y):
    """Same centered2x2 least-squares circle; one3x3 Gram also supplies RHS/meanZ.

    Occupied path only. Retains all samples/iterations, avoids a second GEMM
    and final mean call; no SVD or changed geometric thresholds.
    """
    if len(x)<12:return None
    points=np.column_stack((x,y)).astype(np.float32,copy=False)
    mean_x,mean_y=cv2.mean(points.reshape(-1,1,2))[:2]
    centered=points-np.asarray((mean_x,mean_y),dtype=np.float32)
    zz=cv2.reduce(centered*centered,1,cv2.REDUCE_SUM)
    basis=np.column_stack((centered,zz))
    gram=cv2.gemm(basis,basis,1.,None,0.,flags=cv2.GEMM_1_T)
    aa,bb,cc=float(gram[0,0]),float(gram[0,1]),float(gram[1,1])
    dd,ee=.5*float(gram[0,2]),.5*float(gram[1,2])
    determinant=aa*cc-bb*bb
    if aa<=0 or cc<=0 or determinant<=1e-8*aa*cc:return None
    ox,oy=(dd*cc-bb*ee)/determinant,(aa*ee-bb*dd)/determinant
    radius=math.sqrt(ox*ox+oy*oy+(aa+cc)/len(x))
    return mean_x+ox,mean_y+oy,radius


def recover_occupied_rings(gray, contours, groups, normalize, group_layers, fit_circle, clipped_arcs, roi=None, hints=None):
    """仅在空环路径不能确认三环时调用；拟合/采样次数有界。

    输入已经提取的轮廓，不重复全图二值化。粗椭圆不具有目标资格。
    """
    h, w = gray.shape
    if hints:
        measured = list(groups)
        for hint in hints[:3]:
            if any(math.dist(hint['center'], g['center']) < .3*hint['radius'] for g in measured):
                continue
            current = occupied_sample_group(gray, hint, normalize, group_layers, fit_circle)
            if current is not None:
                measured.append(current)
        if len(measured) == 3:
            return measured  # 仍需调用方检查三组共线/等距/唯一身份。
    seeds = []
    border = []
    # C形薄轮廓面积小，但长度足够；最长轮廓优先且最多32次自由椭圆拟合。
    for contour, bounds, _ in sorted(contours, key=lambda m: len(m[0]), reverse=True)[:32]:
        x, y, bw, bh = bounds
        if len(contour) < 70 or min(bw, bh) < 18 or max(bw, bh) > min(h, w)*.85:
            continue
        if x <= 2 or y <= 2 or x+bw >= w-2 or y+bh >= h-2:
            border.append((contour,bounds))
            continue
        points = contour[::max(1, math.ceil(len(contour)/100))].astype(np.float32)
        ellipse = cv2.fitEllipse(points)
        center, axes, _ = ellipse
        if (min(axes) < 24 or max(axes) > min(h, w)*.85 or min(axes)/max(axes) < .72
                or not all(math.isfinite(v) for v in (*center, *axes))):
            continue
        seeds.append(dict(center=center, radius=sum(axes)/4, ellipse=ellipse))
    # 当前完整外圈提供椭圆比例；画面缩放/相机投影不必强行当成正圆。
    round_seeds=[s for s in seeds if min(s['ellipse'][1])/max(s['ellipse'][1]) >= .9]
    reference = groups[0] if groups else max(round_seeds or seeds,key=lambda s:s['radius']) if seeds else None
    board_reference=reference
    for contour,bounds in border:
        for arc in clipped_arcs(contour, gray.shape, reference, bounds=bounds):
            if arc['radius'] >= 12:
                seeds.append(arc)
    clusters = []
    for seed in sorted(seeds, key=lambda s: s['radius'], reverse=True):
        cluster = next((c for c in clusters if math.dist(c[0]['center'], seed['center']) < .30*max(c[0]['radius'], seed['radius'])), None)
        if cluster is None:
            clusters.append([seed])
        else:
            cluster.append(seed)
    recovered = list(groups)
    for cluster in clusters[:6]:
        maximum = max(s['radius'] for s in cluster)
        # 防止较大的物料轮廓拉偏初值：优先采用圆形的同心外环候选。
        circular = [s for s in cluster if s['radius'] >= maximum*.65 and min(s['ellipse'][1])/max(s['ellipse'][1]) >= .90]
        good = circular or cluster
        cx = occupied_median([s['center'][0] for s in good])
        cy = occupied_median([s['center'][1] for s in good])
        if any(math.dist((cx, cy), g['center']) < .30*maximum and g['radius'] >= maximum*.9 for g in groups):
            continue
        reference = board_reference or max(good, key=lambda s: s['radius'])
        trial = []
        for factor in (1.0, .88, 1.08, 1.16):
            seed = dict(reference, center=(cx, cy), radius=maximum*factor, coarse=True)
            # 搜索ROI人为切开的圆不能作为相机残弧。
            if roi is not None:
                left, top, right, bottom = roi
                _, _, _, _, extents = normalize(seed)
                rx, ry = seed['radius']*extents[0], seed['radius']*extents[1]
                if ((left > 0 and cx-rx <= left+2) or (top > 0 and cy-ry <= top+2)
                        or (right < w and cx+rx >= right-2) or (bottom < h and cy+ry >= bottom-2)):
                    continue
            current = occupied_sample_group(gray, seed, normalize, group_layers, fit_circle)
            if current is not None:
                trial.append(current)
                break  # 已由本帧测齐多层，无需再试所有尺度寻找更大的圈。
        if trial:
            best = max(trial, key=lambda g: g['radius'])
            matches = [g for g in recovered if math.dist(g['center'], best['center']) < .30*max(g['radius'], best['radius'])]
            if not matches:
                recovered.append(best)
            elif all(best['radius'] > g['radius']*1.05 for g in matches):
                recovered = [g for g in recovered if g not in matches]+[best]
    # 透过原始边界弧给搜索初值，当前中环必须实际测到三层，绝不输出中点。
    if len(recovered)==1:
        anchor=recovered[0]
        for edge in (s for s in seeds if s.get('partial')):
            distance=math.dist(anchor['center'],edge['center'])
            if not 4.5 <= distance/max(anchor['radius'],1) <= 9:
                continue
            center=tuple((anchor['center'][i]+edge['center'][i])/2 for i in (0,1))
            seed=dict(anchor,center=center,radius=distance*95/600)
            seed.pop('sampling_radius',None)
            measured=occupied_sample_group(gray,seed,normalize,group_layers,fit_circle)
            if measured is not None:
                recovered.append(measured)
                break
    if len(recovered)==2:
        first,last=recovered
        distance=math.dist(first['center'],last['center'])
        ratio=distance/max((first['radius']+last['radius'])/2,1)
        targets=[]
        if 5<=ratio<=8:
            targets.append((tuple((first['center'][i]+last['center'][i])/2 for i in (0,1)),distance*95/600,False))
        elif 2.2<=ratio<=4.8 and not any(g.get('partial') for g in recovered):
            for a,b in ((first,last),(last,first)):
                targets.append((tuple(2*a['center'][i]-b['center'][i] for i in (0,1)),distance*95/300,True))
        for center,radius,boundary in targets:
            if boundary and not (center[0]<0 or center[0]>=w or center[1]<0 or center[1]>=h):
                continue
            seed=dict(first,center=center,radius=radius,boundary_verified=boundary)
            seed.pop('sampling_radius',None)
            measured=occupied_sample_group(gray,seed,normalize,group_layers,fit_circle)
            if measured is not None:
                recovered.append(measured)
    return sorted(recovered, key=lambda g: g['score'], reverse=True)[:8]


def occupied_median(values):
    """小阵列用直接partition求同样的中位数，省去通用统计函数的归约调度。"""
    array=np.asarray(values).reshape(-1)
    count=len(array)
    if count==0:
        return 0.0
    middle=count//2
    if count%2:
        return float(np.partition(array,middle)[middle])
    split=np.partition(array,(middle-1,middle))
    return (float(split[middle-1])+float(split[middle]))/2


def occupied_sampling_grid(seed,normalize,step=4):
    """相同角度只计算一次；采样阵列保持float32。"""
    cache=getattr(occupied_sampling_grid,'polar',None)
    if cache is None:
        cache={}
        occupied_sampling_grid.polar=cache
    if step not in cache:
        angles=np.arange(0,360,step,dtype=np.float32)*(math.pi/180)
        cache[step]=(angles,np.cos(angles)[:,None],np.sin(angles)[:,None])
    angles,unit_x,unit_y=cache[step]
    cx,cy=seed['center']
    radius=seed.get('sampling_radius',seed['radius'])
    inverse=normalize(seed)[1]
    radial=np.arange(.62*radius,1.07*radius,max(.3,radius/110),dtype=np.float32)
    nx,ny=unit_x*radial,unit_y*radial
    mx=(cx+inverse[0]*nx+inverse[1]*ny).astype(np.float32)
    my=(cy+inverse[2]*nx+inverse[3]*ny).astype(np.float32)
    return angles,radial,nx,ny,mx,my


def occupied_sample_batch(gray,seeds,normalize):
    """将三组的采样映射拼成一个窄矩阵，只调用一次remap。

    已确认、画面内的环先用60方向；边界仍90方向。失败时由调用方回到90方向。
    """
    started=time.monotonic()
    grids=[]
    width=0
    for seed in seeds:
        cx,cy=seed['center']
        r=seed.get('sampling_radius',seed['radius'])
        extents=normalize(seed)[4]
        h,w=gray.shape
        inside=cx-r*extents[0]>=3 and cx+r*extents[0]<w-3 and cy-r*extents[1]>=3 and cy+r*extents[1]<h-3
        grid=occupied_sampling_grid(seed,normalize,6 if inside else 4)
        grids.append(grid)
        width=max(width,grid[-1].shape[1])
    height=sum(len(g[0]) for g in grids)
    mx=np.full((height,width),-1000,dtype=np.float32)
    my=np.full_like(mx,-1000)
    offset=0
    for grid in grids:
        count,cols=grid[-1].shape
        mx[offset:offset+count,:cols]=grid[-2]
        my[offset:offset+count,:cols]=grid[-1]
        offset+=count
    values=cv2.remap(gray,mx,my,cv2.INTER_LINEAR).astype(np.float32)
    result=[]
    offset=0
    for grid in grids:
        count,cols=grid[-1].shape
        result.append((*grid,values[offset:offset+count,:cols]))
        offset+=count
    occupied_sample_batch.last_profile=dict(ms=1000*(time.monotonic()-started),rays=height)
    return result
# END OCCUPIED_RING


class MiddleTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.lock = None
        self.pending = None
        self.count = 0
        self.last_seen = -math.inf
        self.status = "SEARCH_THREE"
        self.last_update = None
        self.frame_interval = 0.0
        self.velocity = (0.0, 0.0)
        self.filtered_center = None
        self.filter_time = None
        self.motion_center = None
        self.motion_time = None
        self.motion_delta = (0.0, 0.0)

    def clear_motion(self):
        # 丢失当帧立即清滤波/速度，不让旧预测参与重新检出的测量。
        self.velocity = (0.0, 0.0)
        self.filtered_center = self.filter_time = None
        self.motion_center = self.motion_time = None
        self.motion_delta = (0.0, 0.0)

    def predicted_center(self, now):
        if self.lock is None:
            return None
        x, y = self.lock["center"]
        if not MOTION_PREDICTION_ENABLED or self.motion_time is None:
            return x, y
        dt = max(0.0, min(PREDICTION_HORIZON_S, now - self.motion_time))
        dx, dy = self.velocity[0] * dt, self.velocity[1] * dt
        limit = min(24.0, self.lock["radius"] * 0.35)
        distance = math.hypot(dx, dy)
        factor = min(1.0, limit / distance) if distance else 1.0
        return x + dx * factor, y + dy * factor

    def fresh_position(self, ring, now):
        """只滤波当前有效实测点；快速位移直通，微抖动轻滤波，不外推位置。"""
        center = tuple(ring["center"])
        dt = now - self.motion_time if self.motion_time is not None else 0.0
        sustained = False
        if self.motion_center is not None and 0 < dt <= PREDICTION_HORIZON_S * 2:
            delta = tuple(center[i] - self.motion_center[i] for i in (0, 1))
            length, old_length = math.hypot(*delta), math.hypot(*self.motion_delta)
            dot = sum(delta[i]*self.motion_delta[i] for i in (0, 1))
            sustained = (length > .75 and old_length > .75 and length/dt > 12
                         and dot > .75 * length * old_length)
            measured = tuple((center[i] - self.motion_center[i]) / dt for i in (0, 1))
            self.velocity = tuple(.5 * self.velocity[i] + .5 * measured[i] for i in (0, 1))
            self.motion_delta = delta
        else:
            self.velocity = (0.0, 0.0)
            self.motion_delta = (0.0, 0.0)
        self.motion_center, self.motion_time = center, now
        previous, previous_time = self.filtered_center, self.filter_time
        filtered = center
        if (POSITION_SMOOTHING_ENABLED and not sustained and previous is not None and previous_time is not None
                and 0 < now - previous_time <= .2):
            distance = math.dist(center, previous)
            # 快速位移、丢帧间隔较大时减少延迟；每轴同一系数避免改变运动方向。
            alpha = max(.55, min(1.0, (now - previous_time) / .12), min(1.0, distance / 3.0))
            filtered = tuple(previous[i] + alpha * (center[i] - previous[i]) for i in (0, 1))
            lag = math.dist(center, filtered)
            if lag > SMOOTH_MAX_LAG_PX:
                ratio = SMOOTH_MAX_LAG_PX / lag
                filtered = tuple(center[i] + ratio * (filtered[i] - center[i]) for i in (0, 1))
        self.filtered_center, self.filter_time = filtered, now
        return filtered

    @staticmethod
    def matches(a, b):
        return (math.dist(a["center"], b["center"]) <= max(8, a["radius"] * 0.65)
                and 0.60 <= b["radius"] / a["radius"] <= 1.50)

    def update(self, groups, triple, now, status="NEED_THREE"):
        if self.last_update is not None:
            interval = now - self.last_update
            if 0 < interval <= TRACK_MEMORY_MAX_S:
                self.frame_interval = (interval if self.frame_interval == 0 else
                                       0.8 * self.frame_interval + 0.2 * interval)
        self.last_update = now
        memory = max(TRACK_MEMORY_S, min(TRACK_MEMORY_MAX_S, 2 * self.frame_interval))
        fresh_confirmation = (triple is not None and self.lock is not None and
                              now - self.last_seen <= TRACK_MEMORY_MAX_S and
                              self.matches(self.lock, triple["ring"]))
        if self.lock is not None and now - self.last_seen > memory and not fresh_confirmation:
            self.reset()
            self.last_update = now
        if triple is not None:
            ring = triple["ring"]
            if self.lock is not None and not self.matches(self.lock, ring):
                self.reset()
            if self.lock is None:
                self.count = self.count + 1 if self.pending is not None and self.matches(self.pending, ring) else 1
                self.pending = ring
                self.status = "ACQUIRE:%d/%d" % (self.count, ACQUIRE_FRAMES)
                if self.count < ACQUIRE_FRAMES:
                    return None
            self.lock, self.last_seen = ring, now
            partial_triple = any(g.get("partial", False) for g in triple["triple"])
            self.status = "MIDDLE_PARTIAL" if partial_triple else "MIDDLE_LOCKED"
            return {"ring": ring, "angle": triple["angle"], "angle_valid": True,
                    "score": triple["score"], "mode": "THREE_PARTIAL" if partial_triple else "THREE",
                    "position": self.fresh_position(ring, now)}
        self.pending, self.count = None, 0
        if status == "AMBIGUOUS":
            self.status = status
            self.clear_motion()
            return None
        if self.lock is not None:
            matches = [g for g in groups if self.matches(self.lock, g)]
            if len(matches) == 1:
                self.lock, self.last_seen = matches[0], now
                partial = matches[0].get("partial", False)
                self.status = "TRACK_PARTIAL" if partial else "TRACK_MIDDLE"
                return {"ring": matches[0], "angle": 0.0, "angle_valid": False,
                        "score": matches[0]["score"], "mode": "TRACK_PARTIAL" if partial else "TRACK",
                        "position": self.fresh_position(matches[0], now)}
            self.status = "LOST_STOP" if not matches else "AMBIGUOUS"
        else:
            self.status = ("SHOW_ALL_3" if groups and len(groups) < 3
                           else "CHECK_ROW" if len(groups) >= 3 else "SEARCH_THREE")
        self.clear_motion()
        return None


class RingDetector:
    """全三环以本帧排列确认；单环局部跟踪仍须核对印字身份。"""
    aperture_mask = None
    def __init__(self):
        # Standalone/library compatibility; the unified service explicitly selects via SET bit1.
        self.allow_occupied = True
        self.template = None
        self.last_global = -math.inf
        self.used_roi = False
        self.identity_rejected = False
        self.require_scene = False
        self.scene_layout = None
        self.scene_trusted = False
        self.last_scene = -math.inf

    def fast_occupied_scene(self, gray, tracker, now, diagnostics=None):
        """先测本帧三个外环小采样阵列；历史只定位搜索窗，成功后不再跑轮廓。

        未完成初始3帧确认也可用上一帧几何作初值，但每帧必须重新测齐三组。
        """
        if (not self.allow_occupied or not ROI_TRACKING_ENABLED or self.scene_layout is None
                or not any(g.get('occluded') for g in self.scene_layout)
                or now-self.last_scene > TRACK_MEMORY_MAX_S):
            return None
        start=time.monotonic()
        delta=(0.,0.)
        if tracker.lock is not None:
            prediction=tracker.predicted_center(now)
            delta=(prediction[0]-tracker.lock['center'][0],prediction[1]-tracker.lock['center'][1])
        hints=[]
        for old in self.scene_layout:
            center=(old['center'][0]+delta[0],old['center'][1]+delta[1])
            hint=dict(old,center=center)
            hint['ellipse']=(center,old['ellipse'][1],old['ellipse'][2])
            hints.append(hint)
        samples=occupied_sample_batch(gray,hints,partial_normalization)
        current=[]
        for old,hint,sampled in zip(self.scene_layout,hints,samples):
            center=hint['center']
            measured=occupied_sample_group(gray,hint,partial_normalization,group_concentric,fit_occupied_circle,sampled=sampled)
            if measured is None:
                # 小阵列失败当帧按原90方向重测，不靠降低证据门槛补足目标。
                measured=occupied_sample_group(gray,hint,partial_normalization,group_concentric,fit_occupied_circle)
            if measured is None:
                base_radius=hint.get('sampling_radius',hint['radius'])
                for factor in (.96,1.04):
                    retry=dict(hint,sampling_radius=base_radius*factor)
                    measured=occupied_sample_group(gray,retry,partial_normalization,group_concentric,fit_occupied_circle)
                    if measured is not None:
                        break
            if measured is None and old.get('boundary_verified') and len(current)==2 and not any(g.get('partial') for g in current):
                near,far=sorted(current,key=lambda g:math.dist(g['center'],center))
                center=tuple(2*near['center'][i]-far['center'][i] for i in (0,1))
                hint['center']=center
                hint['radius']=math.dist(near['center'],far['center'])*95/300
                hint.pop('sampling_radius',None)
                hint['ellipse']=(center,old['ellipse'][1],old['ellipse'][2])
                measured=occupied_sample_group(gray,hint,partial_normalization,group_concentric,fit_occupied_circle)
            if measured is None or not tracker.matches(old,measured):
                return None
            current.append(measured)
        triple,status=choose_middle(current)
        if len(current)!=3 or triple is None:
            return None
        if not all(sum(tracker.matches(old,g) for g in current)==1 for old in self.scene_layout):
            return None
        if tracker.lock is not None and not tracker.matches(tracker.lock,triple['ring']):
            return None
        self.used_roi,self.scene_trusted=True,True
        if diagnostics is not None:
            diagnostics.clear()
            diagnostics.update(B=0.,K=0.,E=0.,P=0.,G=1000*(time.monotonic()-start),
                retry=0.,match=-1.,raw=0,kept=0,arcs=0,weak=0,occupied=3,mode='ARC3_FAST')
        return current,triple,None,'ARC3_FAST'

    def process_scene(self, gray, tracker, now, diagnostics=None):
        """计数模式：覆盖整组三环的窗口重测；缺任一组/排列不符则全图重检。

        历史位置只给搜索窗口，只有当前重新测到三组才能报告3。少于3必须全图。
        """
        self.used_roi, self.identity_rejected, self.scene_trusted = False, False, False
        start = time.monotonic()
        try_fast=self.allow_occupied and now-self.last_global<2.0
        fast=self.fast_occupied_scene(gray,tracker,now,diagnostics) if try_fast else None
        if fast is not None and now-self.last_global < 2.0:
            return fast
        self.used_roi=False
        memory = max(TRACK_MEMORY_S, min(TRACK_MEMORY_MAX_S, 2 * tracker.frame_interval))
        if (ROI_TRACKING_ENABLED and self.scene_layout is not None and tracker.lock is not None
                and now-tracker.last_seen <= memory
                and now-self.last_global < GLOBAL_VERIFY_INTERVAL_S):
            predicted = tracker.predicted_center(now)
            delta = (predicted[0]-tracker.lock['center'][0], predicted[1]-tracker.lock['center'][1])
            h,w=gray.shape
            bounds=[]
            hints=[]
            for old in self.scene_layout:
                x,y=old['center'][0]+delta[0],old['center'][1]+delta[1]
                hint=dict(old, center=(x,y))
                hint['ellipse']=((x,y),old['ellipse'][1],old['ellipse'][2])
                hints.append(hint)
                margin=max(old['ellipse'][1])*.70+max(16,old['radius']*.25)
                bounds.append((x-margin,y-margin,x+margin,y+margin))
            # 一个联合窗口覆盖三环与间隔，避免重复三次阈值/图像分配；ROI边界仍不补全。
            roi=(max(0,math.floor(min(b[0] for b in bounds))),max(0,math.floor(min(b[1] for b in bounds))),
                 min(w,math.ceil(max(b[2] for b in bounds))),min(h,math.ceil(max(b[3] for b in bounds))))
            current,_,combined,_=detect_rings(gray,diagnostics,roi,reference=tracker.lock,hints=hints,occupied=self.allow_occupied)
            triple,status=choose_middle(current)
            all_matches = all(sum(tracker.matches(old,g) for g in current)==1 for old in self.scene_layout)
            if len(current)==3 and triple is not None and all_matches and tracker.matches(tracker.lock,triple['ring']):
                # 全组三环已由本帧外圈独立确认；放料遮住数字不能迫使反复全图。
                self.used_roi,self.scene_trusted=True,True
                if diagnostics is not None:
                    diagnostics.update(mode='SCENE_ROI',retry=0.,match=-1.)
                return current,triple,combined,'SCENE_ROI'
        groups,triple,binary,status=detect_rings(gray,diagnostics,occupied=self.allow_occupied)
        if not try_fast and self.allow_occupied and len(groups)<3 and triple is None and status!='AMBIGUOUS':
            fast=self.fast_occupied_scene(gray,tracker,now)
        self.last_global,self.scene_trusted=now,True
        if fast is not None and len(groups)<3 and triple is None and status!='AMBIGUOUS':
            groups,triple,binary,status=fast
            self.used_roi=True  # 同一帧三组外环实测可补偿轮廓检出不足，不沿用旧目标。
        elif status=='AMBIGUOUS' or len(groups)>3:
            self.scene_layout=None
        if tracker.lock is not None and triple is None and self.template is not None:
            matches=[g for g in groups if tracker.matches(tracker.lock,g)]
            if len(matches)==1:
                score=self.template_score(gray,matches[0])
                self.identity_rejected=score is not None and score<ROI_TEMPLATE_MIN_SCORE
        if diagnostics is not None:
            diagnostics.update(mode='SCENE_FULL',retry=max(0.,1000*(time.monotonic()-start)-sum(diagnostics[k] for k in ('B','K','E','P','G'))),match=-1.)
        return groups,triple,binary,status

    @staticmethod
    def center_patch(gray, group):
        x, y = group["center"]
        half = max(5, round(group["radius"] * 0.48))
        h, w = gray.shape
        if x - half < 1 or y - half < 1 or x + half >= w - 1 or y + half >= h - 1:
            return None
        patch = cv2.getRectSubPix(gray, (2 * half + 1, 2 * half + 1), (float(x), float(y)))
        patch = cv2.resize(patch, (32, 32), interpolation=cv2.INTER_LINEAR)
        if RingDetector.aperture_mask is None:
            coordinates = np.arange(32, dtype=np.float32) - 15.5
            RingDetector.aperture_mask = (coordinates[:, None]**2 + coordinates[None, :]**2) <= 14**2
        aperture = RingDetector.aperture_mask
        # 去掉角落共同的环线，避免相同外圈掩盖数字 1/2/3 的差异。
        patch[~aperture] = int(np.median(patch[aperture]))
        if float(cv2.meanStdDev(patch)[1][0, 0]) < 8:
            return None
        return patch

    def template_score(self, gray, group):
        patch = self.center_patch(gray, group)
        if self.template is None or patch is None:
            return None
        score = float(cv2.matchTemplate(patch, self.template, cv2.TM_CCOEFF_NORMED)[0, 0])
        return score if math.isfinite(score) else None

    def process(self, gray, tracker, now, diagnostics=None):
        if self.require_scene:
            return self.process_scene(gray,tracker,now,diagnostics)
        self.used_roi, self.identity_rejected = False, False
        try_fast=self.allow_occupied and now-self.last_global<2.0
        fast=self.fast_occupied_scene(gray,tracker,now,diagnostics) if try_fast else None
        if fast is not None and now-self.last_global < 2.0:
            return fast
        self.used_roi=False
        retry_ms, match_score, attempted_roi = 0.0, -1.0, False
        memory = max(TRACK_MEMORY_S, min(TRACK_MEMORY_MAX_S, 2 * tracker.frame_interval))
        if (ROI_TRACKING_ENABLED and tracker.lock is not None and self.template is not None
                and now - tracker.last_seen <= memory
                and now - self.last_global < GLOBAL_VERIFY_INTERVAL_S):
            roi_start = time.monotonic()
            attempted_roi = True
            ring = tracker.lock
            x, y = tracker.predicted_center(now)
            margin = max(ring["ellipse"][1]) * 0.70 + max(24, ring["radius"] * 0.35)
            h, w = gray.shape
            roi = (max(0, math.floor(x - margin)), max(0, math.floor(y - margin)),
                   min(w, math.ceil(x + margin)), min(h, math.ceil(y + margin)))
            groups, _, binary, _ = detect_rings(gray, diagnostics, roi, reference=ring, occupied=self.allow_occupied)
            matches = [g for g in groups if tracker.matches(ring, g)]
            if len(matches) == 1:
                score = self.template_score(gray, matches[0])
                match_score = score if score is not None else -1.0
                if score is not None and score >= ROI_TEMPLATE_MIN_SCORE:
                    self.used_roi = True
                    if diagnostics is not None:
                        diagnostics.update(retry=0.0, match=match_score)
                    return groups, None, binary, "ROI"
                self.identity_rejected = score is not None
            retry_ms = 1000 * (time.monotonic() - roi_start)
        groups, triple, binary, status = detect_rings(gray, diagnostics, occupied=self.allow_occupied)
        if not try_fast and self.allow_occupied and len(groups)<3 and triple is None and status!='AMBIGUOUS':
            fast=self.fast_occupied_scene(gray,tracker,now)
        if fast is not None and len(groups)<3 and triple is None and status!='AMBIGUOUS':
            groups,triple,binary,status=fast
            self.used_roi=True
        elif status=='AMBIGUOUS' or len(groups)>3:
            self.scene_layout=None
        if diagnostics is not None:
            diagnostics.update(retry=retry_ms, match=match_score)
            if attempted_roi:
                diagnostics["mode"] = "FULL_R"
        self.last_global = now
        if tracker.lock is not None and triple is None and self.template is not None:
            matches = [g for g in groups if tracker.matches(tracker.lock, g)]
            if len(matches) == 1:
                score = self.template_score(gray, matches[0])
                if score is not None and score < ROI_TEMPLATE_MIN_SCORE:
                    self.identity_rejected = True
        return groups, triple, binary, status

    def accept(self, observation, triple, gray, now):
        if not self.used_roi:
            self.last_global = now
        # 模板只能由全图三组重新确认，防止局部跟踪把模板漂移到邻圈。
        if triple is not None:
            if observation is not None and not self.used_roi:
                self.template = self.center_patch(gray, observation["ring"])
            self.scene_layout = triple['triple']
            self.last_scene = now


class AlignmentState:
    def __init__(self):
        self.count = 0

    def update(self, error):
        if error is None or max(abs(error[0]), abs(error[1])) > ALIGN_TOLERANCE_SCREEN_PX:
            self.count = 0
        else:
            self.count += 1
        return self.count >= ALIGN_STABLE_FRAMES


class LoopProfiler:
    """分段计时；V 只统计实际绘制/显示的帧，不用跳过的零值稀释图传耗时。"""
    def __init__(self):
        self.last = dict(fps=0.0, C=0.0, D=0.0, U=0.0, V=0.0, O=0.0, previews=0)
        self.reset_window()

    def reset_window(self):
        self.frames, self.previews, self.total = 0, 0, 0.0
        self.sums = dict(C=0.0, D=0.0, U=0.0, V=0.0, O=0.0)

    def record(self, total, camera, detect, uart, view=0.0, previewed=False):
        self.frames += 1
        self.total += max(total, 0.0)
        self.previews += int(previewed)
        for name, duration in zip(("C", "D", "U", "V"), (camera, detect, uart, view)):
            self.sums[name] += max(duration, 0.0)
        self.sums["O"] += max(total - camera - detect - uart - view, 0.0)

    def snapshot(self):
        if self.frames:
            self.last = dict(fps=self.frames / self.total if self.total > 0 else 0.0,
                             C=1000 * self.sums["C"] / self.frames,
                             D=1000 * self.sums["D"] / self.frames,
                             U=1000 * self.sums["U"] / self.frames,
                             V=1000 * self.sums["V"] / self.previews if self.previews else 0.0,
                             O=1000 * self.sums["O"] / self.frames,
                             previews=self.previews)
        return self.last.copy()


def correction_velocity(error, pixel_to_body_mm, kp=0.6, max_speed_mm_s=30.0):
    """仅给出已标定安装关系下的速度建议；未标定/无目标始终返回零。"""
    if error is None or pixel_to_body_mm is None:
        return (0.0, 0.0), False
    matrix = np.asarray(pixel_to_body_mm, dtype=float)
    if matrix.shape != (2, 2) or not np.all(np.isfinite(matrix)) or abs(np.linalg.det(matrix)) < 1e-9:
        raise ValueError("Invalid pixel-to-body calibration")
    if not np.all(np.isfinite(error)) or kp <= 0 or max_speed_mm_s <= 0:
        raise ValueError("Invalid correction input")
    velocity = kp * matrix.dot(np.asarray(error, dtype=float))
    norm = float(np.linalg.norm(velocity))
    if norm > max_speed_mm_s:
        velocity *= max_speed_mm_s / norm
    return tuple(float(v) for v in velocity), True


def encode_rloc(error, angle, angle_valid, aligned, quality, sequence, screen_scale=1.0):
    """RING_PX：仅发送位置。图像行方向、对齐计数、形状评分留在本地。

    图像连线角度未经车体标定，不能冒充 ALIGN_YAW 航向误差。
    """
    return encode_pixel_frame(RING_PX, 2, error, sequence, screen_scale)


class RlocSender(Protocol8Sender):
    def __init__(self, serial, screen_scale=1.0):
        super().__init__(serial, UART_SEND_HZ, screen_scale)

    def send(self, error, observation, aligned, now, force=False):
        packet = encode_rloc(error, 0, False, False, 0, self.image_sequence(), self.screen_scale)
        return self.send_packet(packet, bool(packet[2] & 0x80), now, force)

def annotate(gray, groups, observation=None, binary=None, status="", error=None, aligned=False, rgb=False):
    canvas = cv2.cvtColor(binary if binary is not None else gray, cv2.COLOR_GRAY2RGB if rgb else cv2.COLOR_GRAY2BGR)
    def color(bgr):
        return bgr[::-1] if rgb else bgr
    h, w = gray.shape
    cv2.line(canvas, (w // 2, 0), (w // 2, h - 1), (90, 90, 90), 1)
    cv2.line(canvas, (0, h // 2), (w - 1, h // 2), (90, 90, 90), 1)
    for i, group in enumerate(groups):
        cv2.ellipse(canvas, group["ellipse"], color((200, 100, 255) if group.get("partial") else (0, 190, 255)), 1)
        x, y = (round(v) for v in group["center"])
        cv2.putText(canvas, "G%d %sL%d" % (i, "O " if group.get('occluded') else "W " if group.get('weak_recovered') else "P " if group.get("partial") else "", group["levels"]), (x + 3, y - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, color((0, 190, 255)), 1)
    if observation is not None:
        group = observation["ring"]
        x, y = (round(v) for v in group["center"])
        cv2.ellipse(canvas, group["ellipse"], (0, 255, 0), 2)
        cv2.drawMarker(canvas, (x, y), color((0, 0, 255)), cv2.MARKER_CROSS, 12, 1)
        cv2.putText(canvas, "MIDDLE", (x + 7, y + 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (0, 255, 0), 1)
    cv2.rectangle(canvas, (0, 0), (w - 1, 35), (0, 0, 0), -1)
    cv2.putText(canvas, status, (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 255, 255), 1)
    text = "X: -- Y: --" if error is None else "X:%+.1f Y:%+.1f %s" % (*error, "ALIGNED" if aligned else "ALIGN")
    cv2.putText(canvas, text, (4, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (255, 255, 255), 1)
    return canvas


def run_maixcam():
    from maix import app, camera, display, image, touchscreen, err
    from maix.peripheral import uart, pinmap, gpio
    cam = disp = touch = serial = sender = light = None
    displayed = None
    try:
        light = init_light_on(gpio, pinmap, err)
        cv2.setNumThreads(1)
        print("[RING] build=%s native moments=%s + ROI; cv=%s numpy=%s" % (
            RING_BUILD, MOMENT_ELLIPSES_ENABLED, cv2.__version__, getattr(np, "__version__", "?")))
        disp = display.Display()
        if PREVIEW_ENABLED and PREVIEW_FPS <= 0:
            raise ValueError("PREVIEW_FPS must be positive")
        try:
            display.set_trans_image_quality(PREVIEW_JPEG_QUALITY)
        except (AttributeError, RuntimeError) as exc:
            print("JPEG preview quality setting unavailable:", exc)
        sw, sh = disp.width(), disp.height()
        cam = camera.Camera(CAM_WIDTH, CAM_HEIGHT, image.Format.FMT_GRAYSCALE, fps=CAM_FPS)
        touch = touchscreen.TouchScreen()
        if UART_ENABLED:
            for pin, function in UART_PIN_FUNCTIONS.items():
                err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping: " + pin)
            serial = uart.UART(UART_PORT, UART_BAUDRATE, databits=uart.BITS.BITS_8,
                               parity=uart.PARITY.PARITY_NONE, stopbits=uart.STOP.STOP_1)
            sender = RlocSender(serial, min(sw / cam.width(), sh / cam.height()))
        tracker, alignment, detector = MiddleTracker(), AlignmentState(), RingDetector()
        pressed_before, binary_view = False, False
        profile = LoopProfiler()
        detector_stats = {}
        next_preview, next_log = 0.0, time.monotonic() + PERF_LOG_INTERVAL_S
        previous_complete = time.monotonic()
        while not app.need_exit():
            for _ in range(16):
                if not touch.available():
                    break
                tx, ty, pressed = touch.read0()
                left, top, scale = image_fit_geometry(cam.width(), cam.height(), sw, sh)
                fx, fy = (tx - left) / scale, (ty - top) / scale
                if pressed and not pressed_before and cam.height() - 25 <= fy < cam.height():
                    if 0 <= fx < cam.width() / 2:
                        binary_view = not binary_view
                    elif cam.width() / 2 <= fx < cam.width():
                        tracker.reset()
                        alignment.count = 0
                pressed_before = bool(pressed)
            camera_start = time.monotonic()
            frame = cam.read()
            camera_end = time.monotonic()
            if frame is None:
                raise RuntimeError("Camera returned no frame")
            gray = image.image2cv(frame, ensure_bgr=False, copy=False)
            if gray.ndim == 3 and gray.shape[2] == 1:
                gray = gray[:, :, 0]
            groups, triple, binary, detector_status = detector.process(gray, tracker, time.monotonic(), detector_stats)
            now = time.monotonic()
            if detector.identity_rejected and triple is None:
                tracker.reset()
            observation = tracker.update(groups, triple, now, detector_status)
            detector.accept(observation, triple, gray, now)
            if detector.used_roi and observation is not None:
                tracker.status = "ROI_PARTIAL" if observation["ring"].get("partial") else "ROI_MIDDLE"
            error = screen_error(observation["position"], (frame.width(), frame.height()), (sw, sh)) if observation else None
            raw_error = screen_error(observation["ring"]["center"], (frame.width(), frame.height()), (sw, sh)) if observation else None
            aligned = alignment.update(raw_error)
            detect_end = time.monotonic()
            if sender is not None:
                sender.send(error, observation, aligned, detect_end)
            uart_end = time.monotonic()
            previewed = PREVIEW_ENABLED and uart_end >= next_preview
            view_time = 0.0
            if previewed:
                stats = profile.snapshot()
                status = "%s G:%d F:%.1f" % (tracker.status, len(groups), stats["fps"])
                if sender is not None:
                    status += " TX:%d/%d" % (sender.sent % 10000, sender.errors % 1000)
                # 原生 RGB 显示，避免 BGR 图交给 SDK 后又逐像素交换通道。
                rendered = annotate(gray, groups, observation, binary if binary_view else None,
                                     status, error, aligned, rgb=True)
                h, w = gray.shape
                cv2.rectangle(rendered, (0, 35), (w - 1, 52), (0, 0, 0), -1)
                cv2.putText(rendered, "C:%.0f D:%.0f U:%.0f V:%.0f O:%.0f ms" % (
                    stats["C"], stats["D"], stats["U"], stats["V"], stats["O"]), (4, 47),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 255, 255), 1)
                cv2.rectangle(rendered, (0, h - 23), (w - 1, h - 1), (0, 0, 0), -1)
                cv2.putText(rendered, "View: %s" % ("BIN" if binary_view else "GRAY"),
                            (5, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
                cv2.putText(rendered, "Reset lock", (w // 2 + 5, h - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
                shown = image.cv2image(rendered, bgr=False, copy=False)
                disp.show(shown, fit=image.Fit.FIT_CONTAIN)
                displayed = shown, rendered
                view_end = time.monotonic()
                view_time = view_end - uart_end
                # 从显示完成后计间隔，慢图传时仍给识别/串口留出运行机会。
                next_preview = view_end + (1.0 if detect_end-camera_end >= .15 else 1 / PREVIEW_FPS)
            loop_end = time.monotonic()
            # 上一帧结束后的日志/调度也计入总帧周期，避免把 FPS 显示偏高。
            profile.record(loop_end - previous_complete, camera_end - camera_start,
                           detect_end - camera_end, uart_end - detect_end,
                           view_time, previewed)
            previous_complete = loop_end
            if PERF_LOG_INTERVAL_S > 0 and loop_end >= next_log:
                stats = profile.snapshot()
                try:
                    exposure = cam.exposure()
                except Exception:
                    exposure = -1
                print("[PERF] F=%.1f C=%.1f D=%.1f U=%.1f V=%.1f O=%.1f ms PV=%d exp_us=%s" % (
                    stats["fps"], stats["C"], stats["D"], stats["U"], stats["V"],
                    stats["O"], stats["previews"], exposure))
                print("[DET last] %s B=%.1f K=%.1f E=%.1f P=%.1f G=%.1f R=%.1f ms M=%.2f N=%d>%d A=%d W=%d" % (
                    detector_stats["mode"],
                    detector_stats["B"], detector_stats["K"], detector_stats["E"],
                    detector_stats["P"], detector_stats["G"], detector_stats["retry"],
                    detector_stats["match"], detector_stats["raw"],
                    detector_stats["kept"], detector_stats["arcs"],detector_stats.get('weak',0)))
                sampling=getattr(occupied_sample_group,'last_profile',None)
                if sampling:
                    print('[ARC sample last] map=%.1f signal=%.1f fit=%.1f ms' % (
                        sampling['map'],sampling['signal'],sampling['fit']))
                batch=getattr(occupied_sample_batch,'last_profile',None)
                if batch:
                    print('[ARC batch last] prep_map=%.1f ms rays=%d' % (batch['ms'],batch['rays']))
                profile.reset_window()
                next_log = time.monotonic() + PERF_LOG_INTERVAL_S
            del frame, gray, binary
    finally:
        light_off_on_exit(light)
        if sender is not None:
            sender.send(None, None, False, time.monotonic(), force=True)
        for device in (serial, touch, cam, disp):
            if device is not None:
                try:
                    device.close()
                except Exception as exc:
                    print("Device close failed:", exc)


def check_photo(path, output):
    cv2.setNumThreads(1)
    # imdecode 支持 Windows 中文路径。
    original = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if original is None:
        raise ValueError("Cannot read photo")
    h, w = original.shape
    scale = min(CAM_WIDTH / w, CAM_HEIGHT / h)
    resized = cv2.resize(original, (round(w * scale), round(h * scale)), interpolation=cv2.INTER_AREA)
    canvas = np.full((CAM_HEIGHT, CAM_WIDTH), 255, np.uint8)
    left, top = (CAM_WIDTH - resized.shape[1]) // 2, (CAM_HEIGHT - resized.shape[0]) // 2
    canvas[top:top + resized.shape[0], left:left + resized.shape[1]] = resized
    start = time.perf_counter()
    groups, triple, binary, status = detect_rings(canvas)
    elapsed = (time.perf_counter() - start) * 1000
    observation = None if triple is None else {"ring": triple["ring"], "angle": triple["angle"],
                                              "angle_valid": True, "score": triple["score"]}
    error = screen_error(observation["ring"]["center"], (CAM_WIDTH, CAM_HEIGHT),
                         (CAM_WIDTH, CAM_HEIGHT)) if observation else None
    preview = annotate(canvas, groups, observation, None, status, error)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(".png", preview)
    if not ok:
        raise RuntimeError("Preview encoding failed")
    encoded.tofile(str(output))
    report = {"status": status, "processing_size": [CAM_WIDTH, CAM_HEIGHT],
              "desktop_opencv": cv2.__version__,
              "desktop_ms": elapsed, "groups": groups,
              "middle_original_px": None if observation is None else [
                  (observation["ring"]["center"][0] - left) / scale,
                  (observation["ring"]["center"][1] - top) / scale]}
    with output.with_suffix(".json").open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps({k: v for k, v in report.items() if k != "groups"}, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--image")
    parser.add_argument("--output", default="ring_detection.png")
    args = parser.parse_args()
    if args.image:
        check_photo(args.image, args.output)
    else:
        run_maixcam()
