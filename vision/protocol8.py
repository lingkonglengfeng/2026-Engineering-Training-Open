"""项目固定 8 字节像素观测协议；由 sync_protocol8.py 嵌入单文件相机程序。"""
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
