"""Two TCP connections: pull JPEG frames, and duplex protocol8 control."""
import collections
import json
import select
import socket
import struct
import threading
import time

from wire import HELLO, REQUEST, RESULT, GET_FRAME, FRAME, FRAME_V2, ERROR, RESULT_META, MAX_CONTROL, encode, receive, unpack_frame


class StreamContextChanged(RuntimeError):
    """A new SET superseded a frame wait; the video connection remains healthy."""


def connect(host, port):
    sock = socket.create_connection((host, port), timeout=2)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return sock


class ControlClient:
    def __init__(self, host, port):
        self.sock = connect(host, port)
        try:
            kind, payload = receive(self.sock, 2, MAX_CONTROL)
            if kind != HELLO:
                raise ValueError("Not a PC-vision bridge; upload maixcam_bridge/main.py")
            self.hello = json.loads(payload)
            self.epoch = int(self.hello["epoch"])
        except Exception:
            self.sock.close()
            raise
        self.events = collections.deque()
        self.lock = threading.Lock()
        self.tx_lock = threading.Lock()
        self.closed = threading.Event()
        self.error = None
        self.frame = (self.epoch, 0, 0)
        self.sent = 0
        self.observation_groups = 0
        self.last_observation = None
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def _receive(self):
        try:
            while not self.closed.is_set():
                if not select.select([self.sock], [], [], .1)[0]:
                    continue
                kind, payload = receive(self.sock, .75, MAX_CONTROL)
                if kind != REQUEST or len(payload) != 12:
                    raise ValueError("Invalid SET envelope")
                with self.lock:
                    if len(self.events) >= 32:
                        raise ConnectionError("Control queue overflow")
                    self.events.append((struct.unpack("!I", payload[:4])[0], payload[4:]))
        except (OSError, ValueError, ConnectionError) as exc:
            if not self.closed.is_set():
                self.error = exc

    def check(self):
        if self.error is not None:
            raise ConnectionError(str(self.error))

    def read(self, count, timeout=0):
        self.check()
        with self.lock:
            if not self.events:
                return b""
            self.epoch, packet = self.events.popleft()
        return packet

    def has_new_context(self, epoch):
        self.check()
        with self.lock:
            return any(context != epoch for context, _ in self.events)

    def write(self, data):
        self.check()
        if data[1] in (0x81, 0x82):
            meta = RESULT_META.pack(self.epoch, 0, 0)
        else:
            epoch, image_id, stamp = self.frame
            if epoch != self.epoch:
                return len(data)  # discarded old observation; bridge also checks epoch
            meta = RESULT_META.pack(epoch, image_id, stamp)
        with self.tx_lock:
            self.sock.sendall(encode(RESULT, meta + data))
        self.sent += 1
        if data[1] in (2, 3, 9):
            valid = bool(data[2] & 0x80)
            x, y = struct.unpack_from('<hh', data, 3)
            self.observation_groups += 1
            self.last_observation = {
                'epoch': epoch, 'frame': image_id, 'type': data[1], 'id': data[2] & 7,
                'valid': valid, 'dx_px': x / 10 if valid else None,
                'dy_px': y / 10 if valid else None, 'sent_at': time.perf_counter()}
        elif data[1] == 7:
            self.observation_groups += 1
            self.last_observation = {'epoch':epoch,'frame':image_id,'type':7,
                                     'valid':True,'request_seq':(data[2]>>3)&15,
                                     'sent_at':time.perf_counter()}
        return len(data)

    def observation_status(self):
        last = self.last_observation
        if last is not None and last['epoch'] != self.epoch:
            last = None
        return {'groups': self.observation_groups, 'last': last,
                'age_ms': (time.perf_counter() - last['sent_at']) * 1000 if last else None}

    def close(self):
        self.closed.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        self.thread.join(timeout=1)


class VideoClient:
    def __init__(self, host, port, mode="pull-v1", control=None, expected_size=(480,360)):
        self.sock = connect(host, port)
        self.control = control
        self.expected_size = tuple(expected_size)
        self.last = None
        self.error = None
        self.mode = mode
        self.thread = None
        self.ready = threading.Condition()
        self.latest = None
        self.closed = threading.Event()
        self.received = self.skipped = 0
        self.last_receive_age_ms = 0.

    def get(self, epoch):
        try:
            if self.mode == "push-v2":
                return self._get_stream(epoch)
            return self._get(epoch)
        except (OSError, ValueError, ConnectionError) as exc:
            self.error = exc
            raise

    def _get(self, epoch):
        start = time.perf_counter()
        self.sock.sendall(encode(GET_FRAME, struct.pack("!I", epoch)))
        kind, payload = receive(self.sock, 2)
        elapsed_ms = (time.perf_counter()-start)*1000
        if kind == ERROR:
            raise RuntimeError(payload.decode("utf-8", errors="replace"))
        if kind != FRAME:
            raise ValueError("Expected a JPEG frame")
        meta, jpeg = unpack_frame(payload,expected_size=self.expected_size)
        if meta[0] != epoch:
            raise StreamContextChanged("Frame context changed")
        if self.last is not None and meta[1] <= self.last:
            raise ValueError("Repeated/out-of-order bridge frame")
        self.last = meta[1]
        return meta, jpeg, elapsed_ms

    def _read_stream(self):
        try:
            while not self.closed.is_set():
                kind,payload = receive(self.sock,2)
                if kind != FRAME_V2:
                    raise ValueError("Expected v2 continuous frame")
                meta,jpeg = unpack_frame(payload,version=2,expected_size=self.expected_size)
                with self.ready:
                    if self.latest is not None and meta[1] <= self.latest[0][1]:
                        raise ValueError("Out-of-order stream frame")
                    if self.latest is not None and self.latest[0][1] != self.last:
                        self.skipped += 1
                    self.latest = meta,jpeg,time.perf_counter()
                    self.received += 1
                    self.ready.notify_all()
        except (OSError,ValueError,ConnectionError) as exc:
            if not self.closed.is_set():
                with self.ready:
                    self.error = exc
                    self.ready.notify_all()

    def _get_stream(self,epoch):
        start = time.perf_counter()
        if self.thread is None:
            self.sock.sendall(encode(GET_FRAME,struct.pack("!I",epoch)+b"\x01"))
            self.thread = threading.Thread(target=self._read_stream,daemon=True)
            self.thread.start()
        with self.ready:
            while True:
                if self.error is not None:
                    raise ConnectionError(str(self.error))
                if self.control is not None and self.control.has_new_context(epoch):
                    raise StreamContextChanged("New SET pending while waiting for a frame")
                if self.latest is not None:
                    meta,jpeg,received_at = self.latest
                    # Older cached frames may precede a just-applied SET. Only a
                    # newer uint32 epoch supersedes this wait (including wrap).
                    if 0 < ((meta[0] - epoch) & 0xFFFFFFFF) < 0x80000000:
                        raise StreamContextChanged("New stream context superseded frame wait")
                    if meta[0] == epoch and meta[1] != self.last:
                        self.last = meta[1]
                        now = time.perf_counter()
                        self.last_receive_age_ms = (now-received_at)*1000
                        return meta,jpeg,(now-start)*1000
                remaining = 2-(time.perf_counter()-start)
                if remaining <= 0 or self.closed.is_set():
                    raise TimeoutError("No new image in current stream context")
                # SET is received on a separate socket; don't wait two seconds
                # for another JPEG notification before giving it to the service.
                self.ready.wait(min(remaining,.01) if self.control is not None else remaining)

    def close(self):
        self.closed.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        if self.thread is not None:
            self.thread.join(timeout=1)
