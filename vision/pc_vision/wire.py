"""Bounded PC/bridge envelopes; UART protocol8 remains unchanged inside them."""
import json
import struct
import time

MAGIC = b"PCV1"
HELLO, REQUEST, RESULT, GET_FRAME, FRAME, ERROR = range(1, 7)
FRAME_V2 = 7
HEADER = struct.Struct("!4sBI")
FRAME_META = struct.Struct("!IQQHHI")  # epoch, image ID, read-finished ns, W/H, read us
FRAME_META_V2 = struct.Struct("!IQQHHIII")  # append encode us, previous send us
RESULT_META = struct.Struct("!IQQ")  # epoch, image ID, original camera timestamp
MAX_IMAGE = 2 * 1024 * 1024
MAX_CONTROL = 65536
DISCOVERY_PORT = 9002
DISCOVERY_QUERY = b"PCV1_DISCOVER:"
DISCOVERY_RESPONSE = "PCV1_DISCOVERY"


def encode(kind, payload=b""):
    if kind not in range(1, 8) or len(payload) > MAX_IMAGE:
        raise ValueError("Invalid envelope")
    return HEADER.pack(MAGIC, kind, len(payload)) + payload


def read_exact(sock, size, deadline):
    data = bytearray()
    while len(data) < size:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            raise TimeoutError("Incomplete network message")
        sock.settimeout(remaining)
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("Peer disconnected")
        data.extend(chunk)
    return bytes(data)


def receive(sock, timeout=2.0, limit=MAX_IMAGE):
    deadline = time.perf_counter() + timeout
    magic, kind, size = HEADER.unpack(read_exact(sock, HEADER.size, deadline))
    if magic != MAGIC or kind not in range(1, 8) or size > limit:
        raise ValueError("Invalid network header/length")
    return kind, read_exact(sock, size, deadline)


def json_bytes(value):
    return json.dumps(value, ensure_ascii=True, allow_nan=False).encode("utf-8")


def unpack_frame(payload, version=1, expected_size=(480, 360)):
    layout = FRAME_META_V2 if version == 2 else FRAME_META
    if len(payload) <= layout.size:
        raise ValueError("Missing JPEG")
    meta = layout.unpack_from(payload)
    if meta[3:5] != tuple(expected_size):
        raise ValueError("Unexpected camera dimensions")
    return meta, payload[layout.size:]
