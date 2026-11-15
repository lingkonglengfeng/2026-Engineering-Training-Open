"""Best-effort LCD mirror: separate socket, one latest frame, no UART."""
import socket
import struct
import threading
import time
import cv2

SCREEN_HEADER = struct.Struct('!4sIIQHHI')
SCREEN_MAGIC = b'LCD1'
SCREEN_MAX_JPEG = 65536

class ScreenPreviewClient:
    def __init__(self, host, hello):
        port = hello.get('screen_port', 0)
        self.port = port if type(port) is int and 1 <= port <= 65535 and hello.get('screen_preview') == 'jpeg-v1' else 0
        self.host, self.enabled = host, True
        self.ready = threading.Condition(); self.stop = threading.Event()
        self.latest = self.sock = None
        self.next_frame = 0.0; self.sent = 0; self.error = None; self.thread = None
        if self.port:
            self.thread = threading.Thread(target=self.run, daemon=True); self.thread.start()

    def submit(self, bgr, meta):
        now = time.monotonic()
        if not self.port or not self.enabled or bgr is None or meta is None or now < self.next_frame: return
        self.next_frame = now + .1
        try: small = cv2.resize(bgr, (320, 240), interpolation=cv2.INTER_AREA)
        except cv2.error as exc: self.error = str(exc); return
        with self.ready:
            self.latest = (small, tuple(meta[:3]), now); self.ready.notify()

    def run(self):
        while not self.stop.is_set():
            try:
                self.sock = socket.create_connection((self.host, self.port), timeout=.5)
                self.sock.settimeout(.5); self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                while not self.stop.is_set():
                    with self.ready:
                        item, self.latest = self.latest, None
                        if item is None: self.ready.wait(.1); continue
                    frame, meta, submitted = item
                    if not self.enabled or time.monotonic()-submitted > .5: continue
                    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ok or len(encoded) > SCREEN_MAX_JPEG: continue
                    self.sock.sendall(SCREEN_HEADER.pack(SCREEN_MAGIC, *meta, 320, 240, len(encoded))+encoded.tobytes())
                    self.sent += 1; self.error = None
            except (OSError, ValueError, cv2.error) as exc:
                self.error = str(exc); self.stop.wait(1)
            finally:
                if self.sock is not None: self.sock.close(); self.sock = None

    def close(self):
        self.stop.set()
        with self.ready: self.latest = None; self.ready.notify_all()
        sock = self.sock
        if sock is not None:
            try: sock.shutdown(socket.SHUT_RDWR)
            except OSError: pass
        if self.thread is not None: self.thread.join(timeout=2)
