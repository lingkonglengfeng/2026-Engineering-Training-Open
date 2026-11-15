"""MaixCAM RGB/JPEG + UART bridge. Build main.py before single-file upload."""
import collections
import json
import math
import os
import queue
import select
import socket
import struct
import threading
import time
import traceback

from pc_wire import HELLO, REQUEST, RESULT, GET_FRAME, FRAME, FRAME_V2, ERROR, FRAME_META, FRAME_META_V2, RESULT_META, MAX_CONTROL, encode, receive, json_bytes, DISCOVERY_PORT, DISCOVERY_QUERY, DISCOVERY_RESPONSE

BUILD = "pc-vision-bridge-1.3.6"
QR_BUILD = "pc-vision-bridge-1.4.1"
LOCAL_PREVIEW_FPS = 10
SCREEN_PORT = 9003
LIGHT_PORT = 9004
LIGHT_MAX_LINE = 256
SCREEN_HEADER = struct.Struct('!4sIIQHHI')
SCREEN_MAGIC = b'LCD1'
SCREEN_MAX_JPEG = 65536
OBSTACLE_TYPE = 0x86
OBSTACLE_MAX_AGE_NS = 200_000_000
BRIDGE_ROLE = "material"
VIDEO_PORT, CONTROL_PORT = 9000, 9001
JPEG_QUALITY = 85
UART_PORT, UART_BAUD = "/dev/ttyS0", 115200
CALIBRATION = os.path.expanduser("~/maixcam_material_colors.json")
CAM1_LIGHT_PIN, CAM1_LIGHT_GPIO = "A18", "GPIOA18"
CAM1_LIGHT_PWM, CAM1_LIGHT_HZ = 6, 20000
# Auto brightness per requested recognition mode: material=25%, plate=60%, ring=30%.
CAM1_LIGHT_MODE_PERCENT = {2: 25, 8: 60, 3: 30}
CAM1_LIGHT_OTHER_PERCENT = 100


class FillLight:
    def __init__(self, gpio, pinmap, err, pwm):
        self.gpio,self.pinmap,self.err = gpio,pinmap,err
        self.output = None
        self.percent = None
        self.mode = None
        self.manual = None  # None = follow the current camera mode; else 0..100
        try:
            self.output = pwm.PWM(CAM1_LIGHT_PWM, freq=CAM1_LIGHT_HZ, duty=100, enable=False)
            err.check_raise(pinmap.set_pin_function(CAM1_LIGHT_PIN, 'PWM6'), 'CAM1 light PWM mapping')
            err.check_raise(self.output.enable(), 'CAM1 light PWM enable')
            self.percent = 100
        except Exception:
            self.close()
            raise

    def apply(self):
        if self.output is None:return
        percent = self.manual if self.manual is not None else CAM1_LIGHT_MODE_PERCENT.get(self.mode, CAM1_LIGHT_OTHER_PERCENT)
        percent = int(percent)
        if self.percent != percent:
            self.output.duty(percent)
            self.percent = percent
            source = 'manual' if self.manual is not None else ('mode=%s' % self.mode)
            print('[BRIDGE LIGHT] A18/PWM6 duty=%d%% %s freq=20000Hz' % (percent, source), flush=True)

    def set_mode(self, mode):
        self.mode = mode
        self.apply()

    def set_manual(self, percent):
        """None returns to the mode default; otherwise clamp 0..100 in RAM."""
        if percent is None:
            self.manual = None
        else:
            try:value = float(percent)
            except (TypeError, ValueError):
                raise ValueError('brightness must be a number or auto') from None
            if not math.isfinite(value):raise ValueError('brightness must be finite')
            self.manual = max(0, min(100, int(round(value))))
        self.apply()

    def status(self):
        return {'percent': self.percent, 'manual': self.manual, 'mode': self.mode}

    def close(self):
        try:
            if self.output is not None:
                output,self.output = self.output,None
                try:
                    output.duty(0)
                finally:
                    output.disable()
        finally:
            self.err.check_raise(self.pinmap.set_pin_function(CAM1_LIGHT_PIN, CAM1_LIGHT_GPIO), 'CAM1 light OFF mapping')
            off = self.gpio.GPIO(CAM1_LIGHT_GPIO, self.gpio.Mode.OUT, self.gpio.Pull.PULL_DOWN)
            off.value(0)
            self.percent = 0


def init_fill_light(role):
    """CAM1 only: material25%, plate60%, ring30%, other100%; CAM2 owns no light pins."""
    if role != 'material':
        return None
    from maix import err
    from maix.peripheral import gpio, pinmap, pwm
    light = FillLight(gpio, pinmap, err, pwm)
    print('[BRIDGE LIGHT] A18/PWM6 20000Hz; startup100%%, material25%%, plate60%%, ring30%%, other100%%; '
          'light_port=%d accepts {"light":0..100} or {"light":"auto"}' % LIGHT_PORT, flush=True)
    return light


def close_fill_light(light):
    if light is not None:
        try:
            light.close()
            print('[BRIDGE LIGHT] A18/GPIOA18 LOW/OFF; PWM disabled', flush=True)
        except Exception as exc:
            print('[BRIDGE LIGHT] OFF failed:', exc, flush=True)


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 7) & 255 if value & 128 else (value << 1) & 255
    return value


def valid_group(data,role="material"):
    lengths=(8,32) if role=='qr' else (8,16,24)
    types=(7,0x81,0x82,OBSTACLE_TYPE) if role=='qr' else (2,3,9,10,0x81,0x82,0x83,0x85)
    if len(data) not in lengths:return False
    if role=='qr':
        if len(data)==8 and data[1] not in (0x81,0x82,OBSTACLE_TYPE):return False
        # Age is filled only by the camera bridge at actual UART start.
        if len(data)==8 and data[1]==OBSTACLE_TYPE and data[2]!=0:return False
        if len(data)==32 and not all(data[i+1]==7 and data[i+2]&0x80 and
                                    data[i+2]&7==i//8+1 and data[i+6]==4 and
                                    (data[i+2]>>3)&15==(data[2]>>3)&15 for i in range(0,32,8)):return False
    return all(
        data[i] == 0x6B and data[i+1] in types
        and crc8(data[i:i+7]) == data[i+7] for i in range(0, len(data), 8))


def qr_rule_text(data):
    inverse=(0,1,2,3,4,6,5);groups=[]
    for index in range(4):
        values=list(data[index*8+3:index*8+6])
        if any(v<1 or v>6 for v in values):return None
        if index in (0,2):
            if len(set(values))!=3:return None
            values=[inverse[v] for v in values]
        elif set(values)!={1,2,3}:return None
        groups.append(''.join(str(v) for v in values))
    return '+'.join(groups)


class BridgeState:
    def __init__(self,role="material"):
        if role not in ('material','qr'):raise ValueError('Unknown bridge role')
        self.role=role
        self.build=QR_BUILD if role=='qr' else BUILD
        self.control_port,self.video_port=CONTROL_PORT,VIDEO_PORT
        self.frame_size=(640,480) if role=='qr' else (480,360)
        self.rotation=180 if role=='qr' else 0
        self.display_task=None
        # UART's first obstacle write revalidates age/context while holding this
        # lock; result_current also holds it, so the lock must be re-entrant.
        self.lock = threading.RLock()
        self.frame_ready = threading.Condition(self.lock)
        self.latest = None
        self.capture_active = False
        self.capture_error = None
        self.screen_enabled = False  # run() enables it only with the device LCD owner.
        self.screen_latest = None
        self.screen_next = 0.0
        self.screen_frames = self.screen_copy_us = self.screen_show_us = 0
        self.screen_port = 0
        self.screen_annotated = None
        self.epoch = 0
        self.request = None
        self.last_request_time = -100.0
        self.revision = 0
        self.connection = 0
        self.results = queue.Queue(maxsize=16)
        self.stop = threading.Event()
        self.frames = 0
        self.last_roundtrip_ms = 0.0
        self.dropped = 0
        self.frame_times = collections.OrderedDict()
        self.fill_light = None

    def accept_request(self, packet, now):
        with self.lock:
            if self.fill_light is not None:
                try:
                    self.fill_light.set_mode(packet[3])
                except Exception as exc:
                    print('[BRIDGE LIGHT] mode update failed; video/UART continue:', exc, flush=True)
            if packet != self.request or now - self.last_request_time > 2.0:
                self.epoch = (self.epoch + 1) & 0xFFFFFFFF
                self.frame_times.clear()
                if self.role=='qr' and packet[3]==6:self.display_task=None
            self.request, self.last_request_time = packet, now
            self.revision += 1

    def remember_frame(self, epoch, image_id, stamp):
        with self.lock:
            if epoch == self.epoch:
                self.frame_times[image_id] = stamp
                while len(self.frame_times) > 32:
                    self.frame_times.popitem(last=False)

    def result_current(self, connection, payload, now_ns):
        if len(payload) < RESULT_META.size:
            return None
        epoch, image_id, stamp = RESULT_META.unpack_from(payload)
        data = payload[RESULT_META.size:]
        if not valid_group(data,self.role):
            return None
        with self.lock:
            if connection != self.connection or epoch != self.epoch:
                return None
            observation = data[1] not in (0x81, 0x82)
            obstacle = self.role=='qr' and len(data)==8 and data[1]==OBSTACLE_TYPE
            if self.role=='qr' and observation and not obstacle:
                if self.request is None or self.request[3]!=6 or self.request[4]!=4 or \
                   (data[2]>>3)&15!=(self.request[2]>>3)&15:return None
                task=qr_rule_text(data)
                if task is None:return None
            if observation:
                age_ns = now_ns - stamp
                limit = OBSTACLE_MAX_AGE_NS if obstacle else 500_000_000
                if self.frame_times.get(image_id) != stamp or not 0 <= age_ns <= limit:
                    self.dropped += 1
                    return None
                self.last_roundtrip_ms = age_ns/1e6
                if obstacle:
                    age10 = (age_ns + 9_999_999)//10_000_000
                    data = data[:2] + bytes((age10,)) + data[3:7]
                    data += bytes((crc8(data),))
                elif self.role=='qr':self.display_task=task
            return data

    def hello(self):
        result={'build':self.build,'role':self.role,'frame_size':self.frame_size,
                'rotation':self.rotation,'epoch':self.epoch,'video_mode':'push-v2',
                'video_port':self.video_port,'calibration':load_samples()}
        if self.role=='qr':
            result.update(obstacle_uart_type=OBSTACLE_TYPE,obstacle_max_age_ms=200)
        else:
            result.update(light_port=LIGHT_PORT,
                          light=(self.fill_light.status() if self.fill_light is not None else None))
        if self.screen_port: result.update(screen_preview='jpeg-v1',screen_port=self.screen_port)
        return result


def write_obstacle_first(state, serial, connection, payload, now_ns=None):
    """Recheck original frame and stamp its real age immediately before UART.

    Returns (packet, count), or (None, 0) for a stale/disconnected result.
    The caller retries a zero-byte write with the original payload so age is
    recomputed. Once a byte has been written, normal partial-frame handling
    owns the remaining suffix and MCU accounts for its arrival duration.
    """
    with state.lock:
        if now_ns is None:
            now_ns = time.monotonic_ns()
        packet = state.result_current(connection, payload, now_ns)
        if packet is None:
            return None, 0
        written = serial.write(packet)
        if not isinstance(written, int) or not 0 <= written <= len(packet):
            raise OSError("UART short-write count invalid")
        return packet, written


def server_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.listen(1)
    sock.settimeout(0.5)
    return sock


def discovery_socket(port=DISCOVERY_PORT):
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
        sock.bind(('0.0.0.0',port));sock.settimeout(.2)
    except Exception:
        sock.close();raise
    return sock


def discovery_server(state,listener):
    name=socket.gethostname()[:64]
    while not state.stop.is_set():
        try:data,address=listener.recvfrom(256)
        except socket.timeout:continue
        except OSError:break
        if not data.startswith(DISCOVERY_QUERY):continue
        nonce=data[len(DISCOVERY_QUERY):]
        if len(nonce)!=16 or any(c not in b'0123456789abcdef' for c in nonce):continue
        payload={'protocol':DISCOVERY_RESPONSE,'nonce':nonce.decode('ascii'),'name':name,
                 'role':state.role,'build':state.build,'control_port':state.control_port,'video_port':state.video_port}
        if state.role=='material':payload['light_port']=LIGHT_PORT
        try:listener.sendto(json_bytes(payload),address)
        except OSError:continue


def load_samples():
    try:
        with open(CALIBRATION, "r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, ValueError):
        return {"version": 1, "samples": {}}


def control_server(state, listener):
    while not state.stop.is_set():
        try:
            conn, address = listener.accept()
        except socket.timeout:
            continue
        print("[BRIDGE] control PC", address, flush=True)
        with conn:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with state.lock:
                state.connection += 1
                connection = state.connection
            revision = -1
            try:
                conn.settimeout(.5)
                with state.lock:
                    hello = state.hello()
                conn.sendall(encode(HELLO, json_bytes(hello)))
                while not state.stop.is_set():
                    with state.lock:
                        current, request, epoch = state.revision, state.request, state.epoch
                    if revision != current:
                        revision = current
                        if request is not None:
                            conn.settimeout(.5)
                            conn.sendall(encode(REQUEST, struct.pack("!I", epoch) + request))
                    if not select.select([conn], [], [], .005)[0]:
                        continue
                    kind, payload = receive(conn, .5, MAX_CONTROL)
                    if kind != RESULT:
                        raise ValueError("Expected a protocol result")
                    state.results.put_nowait((connection, payload))
            except (OSError, ValueError, ConnectionError, queue.Full) as exc:
                print("[BRIDGE] control closed:", exc, flush=True)
            finally:
                with state.lock:
                    state.connection += 1  # queued data from this connection cannot outlive it


def light_command(state, line):
    """One newline-delimited JSON light command -> JSON-able reply dict.

    {"light": 0..100} sets a manual duty, {"light": null} or
    {"light": "auto"} returns to the mode default (disc/material 50%,
    ring/other 100%). {"status": true} only reads back the current state.
    """
    try:
        command = json.loads(line)
    except (TypeError, ValueError):
        return {'ok': False, 'error': 'bad json'}
    if not isinstance(command, dict):
        return {'ok': False, 'error': 'bad command'}
    light = getattr(state, 'fill_light', None)
    if light is None:
        return {'ok': False, 'error': 'light unavailable'}
    if 'light' in command:
        value = command['light']
        try:
            if value is None or (isinstance(value, str) and value.strip().lower() == 'auto'):
                light.set_manual(None)
            else:
                light.set_manual(value)
        except (TypeError, ValueError) as exc:
            return {'ok': False, 'error': str(exc) or 'bad brightness'}
        reply = {'ok': True}
        reply.update(light.status())
        return reply
    if command.get('status') is True:
        reply = {'ok': True}
        reply.update(light.status())
        return reply
    return {'ok': False, 'error': 'unknown command'}


def command_server(state, listener):
    """Dedicated RAM-only light channel; never touches the UART/video path."""
    while not state.stop.is_set():
        try:
            conn, address = listener.accept()
        except socket.timeout:
            continue
        except OSError:
            if state.stop.is_set(): break
            raise
        print("[BRIDGE] command PC", address, flush=True)
        with conn:
            conn.settimeout(.5); buffer = bytearray()
            try:
                while not state.stop.is_set():
                    if not select.select([conn], [], [], .2)[0]:
                        continue
                    chunk = conn.recv(256)
                    if not chunk: break
                    buffer.extend(chunk)
                    while b'\n' in buffer:
                        raw, _, rest = bytes(buffer).partition(b'\n')
                        buffer = bytearray(rest)
                        raw = raw.strip()
                        if not raw: continue
                        reply = ({'ok': False, 'error': 'command too long'}
                                 if len(raw) > LIGHT_MAX_LINE else light_command(state, raw))
                        conn.sendall(json.dumps(reply, separators=(',', ':')).encode('utf-8') + b'\n')
                    if len(buffer) > LIGHT_MAX_LINE:
                        buffer.clear()
                        conn.sendall(b'{"ok":false,"error":"command too long"}\n')
            except (OSError, ValueError, ConnectionError) as exc:
                if not state.stop.is_set():
                    print('[BRIDGE] command closed:', exc, flush=True)


def publish_screen_frame(state, frame):
    now = time.monotonic()
    if not state.screen_enabled or now < state.screen_next: return
    with state.lock:
        if screen_annotation_current(state, state.screen_annotated): return
    state.screen_next = now + 1.0/LOCAL_PREVIEW_FPS
    started = time.perf_counter_ns()
    try:
        preview = frame.copy()  # LCD overlay/driver must never mutate the camera/JPEG frame.
        if preview is None: raise RuntimeError('LCD frame copy returned no image')
    except Exception as exc:
        state.screen_enabled = False
        print('[BRIDGE] LCD copy disabled; video/UART continue:', exc, flush=True)
        return
    with state.frame_ready:
        state.screen_copy_us = (time.perf_counter_ns()-started)//1000
        state.screen_latest = preview  # One replaceable slot, never a growing queue.
        state.frame_ready.notify_all()


def screen_annotation_current(state, item):
    return bool(item and item[0] == state.epoch and 0 <= time.monotonic_ns()-item[2] <= 1_000_000_000)


def screen_server(state, listener):
    """Display-only inbound JPEG. No result queue or UART access."""
    while not state.stop.is_set():
        try: conn, _ = listener.accept()
        except socket.timeout: continue
        except OSError:
            if state.stop.is_set(): break
            raise
        with conn:
            try:
                while not state.stop.is_set():
                    header = read_screen_exact(conn, SCREEN_HEADER.size)
                    magic, epoch, frame_id, stamp, width, height, size = SCREEN_HEADER.unpack(header)
                    if magic != SCREEN_MAGIC or (width,height)!=(320,240) or not 0 < size <= SCREEN_MAX_JPEG:
                        raise ValueError('invalid LCD image envelope')
                    jpeg = read_screen_exact(conn, size)
                    with state.frame_ready:
                        item = (epoch,frame_id,stamp,jpeg)
                        if state.screen_enabled and state.frame_times.get(frame_id)==stamp and screen_annotation_current(state,item):
                            state.screen_annotated = item; state.screen_latest = None
                            state.frame_ready.notify_all()
            except (OSError,ValueError,ConnectionError) as exc:
                if not state.stop.is_set(): print('[BRIDGE] LCD peer closed:',exc,flush=True)


def read_screen_exact(conn, size):
    # A short timeout bounds shutdown; display reconnects independently.
    deadline = time.monotonic()+1
    data = bytearray()
    while len(data) < size:
        remaining = deadline-time.monotonic()
        if remaining <= 0: raise TimeoutError('LCD packet timeout')
        conn.settimeout(remaining)
        chunk = conn.recv(size-len(data))
        if not chunk: raise ConnectionError('LCD peer disconnected')
        data.extend(chunk)
    return bytes(data)


def display_frames(state, disp, image):
    """Only this thread owns Display; LCD failures do not become capture errors."""
    try:
        next_show, shown_annotation = 0.0, None
        while not state.stop.is_set() and state.screen_enabled:
            if time.monotonic() < next_show:
                state.stop.wait(next_show-time.monotonic()); continue
            with state.frame_ready:
                annotation = state.screen_annotated
                if screen_annotation_current(state,annotation):
                    state.screen_latest = None
                    if annotation[:3] == shown_annotation:
                        state.frame_ready.wait(.05); continue
                    frame = None
                else:
                    annotation = None; shown_annotation = None
                    frame, state.screen_latest = state.screen_latest, None
                if frame is None and annotation is None:
                    state.frame_ready.wait(.1)
                    continue
                mode = state.request[3] if state.request else 0
                task = state.display_task
            started = time.perf_counter_ns()
            if annotation is not None:
                try:
                    encoded = image.from_bytes(320,240,image.Format.FMT_JPEG,annotation[3])
                    frame = encoded.to_format(image.Format.FMT_RGB888)
                except Exception as exc:
                    with state.lock:
                        if state.screen_annotated is annotation: state.screen_annotated = None
                    print('[BRIDGE] LCD JPEG dropped:',exc,flush=True); continue
                shown_annotation = annotation[:3]
            elif state.role == 'qr':
                status = 'STOP / LAST TASK' if mode == 0 and task else 'LATCHED' if task else 'WAIT PC / SCAN'
                frame.draw_string(8, 12, 'CAM2 PC QR '+status, image.COLOR_WHITE)
                frame.draw_string(8, 32, task or 'Waiting for confirmed task', image.COLOR_WHITE)
            disp.show(frame)
            next_show = time.monotonic()+1.0/LOCAL_PREVIEW_FPS
            with state.lock:
                state.screen_frames += 1
                state.screen_show_us = (time.perf_counter_ns()-started)//1000
    except Exception as exc:
        state.screen_enabled = False
        print('[BRIDGE] LCD disabled; video/UART continue:', exc, flush=True)
    finally:
        with state.frame_ready: state.screen_latest = None
        if disp is not None:
            try: disp.close()
            except Exception as exc: print('[BRIDGE] LCD close:', exc, flush=True)


def capture_frames(state, camera, image):
    """One device thread owns camera; one latest JPEG slot, independent of network/UI."""
    cam = None
    opened_epoch = None
    try:
        cam = camera.Camera(*state.frame_size, image.Format.FMT_RGB888, fps=30)
        while not state.stop.is_set():
            with state.frame_ready:
                active = state.capture_active
                if not active and not state.screen_enabled:
                    opened_epoch = None  # resumed capture must discard driver frames from the idle period
                    state.frame_ready.wait(.1)
                    continue
                epoch = state.epoch
            if active and opened_epoch != epoch:
                clear = getattr(cam, "clear_buff", None)
                if clear is not None:
                    clear()
                opened_epoch = epoch
            if not active: opened_epoch = None
            before = time.perf_counter_ns()
            frame = cam.read(block=True, block_ms=250)
            read_finished = time.perf_counter_ns()
            # Use the same monotonic clock for frame IDs and UART age; keep
            # perf_counter_ns only for measuring local capture/encode durations.
            stamp = time.monotonic_ns()
            if frame is None:
                raise RuntimeError("camera returned no frame")
            if not active:
                publish_screen_frame(state, frame)
                del frame
                state.stop.wait(1.0/LOCAL_PREVIEW_FPS)
                continue
            jpeg = frame.to_jpeg(quality=JPEG_QUALITY).to_bytes()
            encoded = time.perf_counter_ns()
            with state.frame_ready:
                state.frames += 1
                if state.epoch == epoch and state.capture_active:
                    state.latest = (epoch, state.frames, stamp, frame.width(), frame.height(),
                                    min(0xFFFFFFFF,(read_finished-before)//1000),
                                    min(0xFFFFFFFF,(encoded-read_finished)//1000), jpeg)
                    state.frame_ready.notify_all()
            publish_screen_frame(state, frame)
            del frame, jpeg
    except Exception as exc:
        traceback.print_exc()
        with state.frame_ready:
            state.capture_error = str(exc)
            state.frame_ready.notify_all()
    finally:
        if cam is not None:
            cam.close()


def video_server(state, listener, camera, image):
    capture = threading.Thread(target=capture_frames,args=(state,camera,image),daemon=True)
    capture.start()
    try:
        while not state.stop.is_set():
            try:
                conn, address = listener.accept()
            except socket.timeout:
                continue
            print("[BRIDGE] video PC", address, flush=True)
            with conn:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
                last_image, previous_send_us, streaming, waiting_request = 0, 0, False, False
                with state.frame_ready:
                    state.capture_active = True
                    state.latest = None
                    state.frame_ready.notify_all()
                try:
                    while not state.stop.is_set():
                        if not streaming and not waiting_request:
                            if not select.select([conn], [], [], .1)[0]:
                                continue
                            kind, payload = receive(conn, 1.0, MAX_CONTROL)
                            if kind != GET_FRAME or len(payload) not in (4,5):
                                raise ValueError("Expected frame request")
                            epoch = struct.unpack("!I", payload[:4])[0]
                            streaming = len(payload) == 5 and payload[4] == 1
                            waiting_request = True
                        with state.frame_ready:
                            slot = state.latest
                            if slot is None or slot[1] <= last_image or slot[0] != state.epoch:
                                if state.capture_error:
                                    raise RuntimeError(state.capture_error)
                                state.frame_ready.wait(.1)
                                continue
                            if not streaming and epoch != state.epoch:
                                slot = None
                        if slot is None:
                            conn.sendall(encode(ERROR, b"context changed"))
                            waiting_request = False
                            continue
                        last_image = slot[1]
                        state.remember_frame(slot[0],slot[1],slot[2])
                        if streaming:
                            meta = FRAME_META_V2.pack(*slot[:7],previous_send_us)
                            kind = FRAME_V2
                        else:
                            meta = FRAME_META.pack(*slot[:6])
                            kind = FRAME
                        conn.settimeout(1.0)
                        started = time.perf_counter_ns()
                        conn.sendall(encode(kind, meta + slot[7]))
                        waiting_request = False
                        previous_send_us = min(0xFFFFFFFF,(time.perf_counter_ns()-started)//1000)
                except (OSError, ValueError, RuntimeError, ConnectionError) as exc:
                    print("[BRIDGE] video closed:", exc, flush=True)
                finally:
                    with state.frame_ready:
                        state.capture_active = False
    finally:
        capture.join(timeout=1)


def run():
    from maix import app, camera, image, err, network, display
    from maix.peripheral import uart, pinmap
    state = BridgeState(BRIDGE_ROLE)
    listeners, threads, serial, light = [], [], None, None
    try:
        try:
            light = init_fill_light(BRIDGE_ROLE)
            state.fill_light = light
        except Exception as exc:
            print('[BRIDGE LIGHT] unavailable; video/UART continue:', exc, flush=True)
        for pin, function in (("A16", "UART0_TX"), ("A17", "UART0_RX")):
            err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping")
        serial = uart.UART(UART_PORT, UART_BAUD, databits=uart.BITS.BITS_8,
                           parity=uart.PARITY.PARITY_NONE, stopbits=uart.STOP.STOP_1)
        print("[BRIDGE]", state.build, "IP:", network.wifi.Wifi().get_ip(), flush=True)
        if BRIDGE_ROLE == 'qr':
            print("[BRIDGE] CAM2 has no light GPIO/PWM initialization.", flush=True)
        try:
            disp=display.Display()  # Finish device initialization before Camera starts.
            state.screen_enabled = True
            lcd = threading.Thread(target=display_frames, args=(state,disp,image), daemon=True)
            lcd.start(); threads.append(lcd)
        except Exception as exc: print('[BRIDGE] LCD unavailable; video/UART continue:',exc,flush=True)
        video, control = server_socket(VIDEO_PORT), server_socket(CONTROL_PORT)
        listeners = [video, control]
        targets=[(control_server,(state,control)),(video_server,(state,video,camera,image))]
        if BRIDGE_ROLE == 'material':
            try:
                light_server = server_socket(LIGHT_PORT)
                listeners.append(light_server)
                targets.append((command_server, (state, light_server)))
            except OSError as exc:
                print('[BRIDGE] light command port unavailable; GUI brightness unavailable:', exc, flush=True)
        if state.screen_enabled:
            try:
                screen=server_socket(SCREEN_PORT);listeners.append(screen);state.screen_port=SCREEN_PORT
                targets.append((screen_server,(state,screen)))
            except OSError as exc: print('[BRIDGE] annotated LCD unavailable; local preview remains:',exc,flush=True)
        try:
            discovery=discovery_socket();listeners.append(discovery)
            targets.append((discovery_server,(state,discovery)))
        except OSError as exc:print('[BRIDGE] discovery unavailable; direct IP remains available:',exc,flush=True)
        for target, args in targets:
            thread = threading.Thread(target=target, args=args, daemon=True)
            thread.start()
            threads.append(thread)
        rx, pending = bytearray(), b""
        pending_obstacle_payload = None
        pending_started = False
        rx_started = tx_started = resume = 0.0
        pending_epoch = pending_connection = -1
        log_due = time.monotonic() + 2
        while not app.need_exit() and not state.stop.is_set():
            now = time.monotonic()
            if rx and now-rx_started >= .05:
                rx.clear()
            data = serial.read(256, timeout=0)
            if data:
                if not rx:
                    rx_started = now
                rx.extend(data)
                while len(rx) >= 8:
                    if rx[0] != 0x6B or crc8(rx[:7]) != rx[7]:
                        del rx[0]
                        continue
                    packet = bytes(rx[:8]); del rx[:8]
                    if packet[1] == 0x80:
                        state.accept_request(packet, now)
                if rx:
                    rx_started = now
            with state.lock:
                epoch, connection = state.epoch, state.connection
            if pending and (pending_epoch != epoch or pending_connection != connection or now-tx_started >= .05):
                pending = b""
                pending_obstacle_payload = None
                pending_started = False
                resume = now + .055  # existing protocol8 partial-frame recovery gap
            if not pending and now >= resume:
                for _ in range(16):
                    try:
                        item_connection, payload = state.results.get_nowait()
                    except queue.Empty:
                        break
                    accepted = state.result_current(item_connection, payload, time.monotonic_ns())
                    if accepted is not None:
                        pending, tx_started = accepted, now
                        pending_epoch, pending_connection = epoch, item_connection
                        pending_obstacle_payload = payload if accepted[1] == OBSTACLE_TYPE else None
                        pending_started = False
                        break
            if pending:
                if pending_obstacle_payload is not None and not pending_started:
                    # A nonblocking UART may write zero bytes. Until the FIRST
                    # byte is accepted, revalidate and refresh the age/CRC on
                    # every attempt. Hold the context lock across that one
                    # small write so a disconnected client cannot race it.
                    refreshed, written = write_obstacle_first(state, serial,
                        pending_connection, pending_obstacle_payload)
                    if refreshed is None:
                        pending = b""
                        pending_obstacle_payload = None
                        continue
                    pending = refreshed
                else:
                    written = serial.write(pending)
                if not isinstance(written, int) or not 0 <= written <= len(pending):
                    raise OSError("UART short-write count invalid")
                if written:
                    pending_started = True
                pending = pending[written:]
                if not pending:
                    pending_obstacle_payload = None
                    pending_started = False
            if now >= log_due:
                print("[BRIDGE] epoch=%d frames=%d sample_to_return_ms=%.1f old_groups=%d LCD=%s lcd_frames=%d copy_us=%d show_us=%d" %
                      (epoch, state.frames, state.last_roundtrip_ms, state.dropped, state.screen_enabled,
                       state.screen_frames,state.screen_copy_us,state.screen_show_us), flush=True)
                log_due = now + 2
            serial.available(timeout=2)
    finally:
        close_fill_light(light)
        state.stop.set()
        for listener in listeners:
            listener.close()
        for thread in threads:
            thread.join(timeout=2)
        if serial is not None:
            serial.close()


if __name__ == "__main__":
    run()
