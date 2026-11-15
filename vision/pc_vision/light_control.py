"""CAM1 fill-light brightness client for the MaixCAM bridge light port.

The camera bridge owns the A18/PWM6 light; this module only sends a small
newline-delimited JSON command over TCP. Failures never raise so the desktop
panel can keep the vision/preview path untouched.
"""
import json
import socket

DEFAULT_LIGHT_PORT = 9004
DEFAULT_TIMEOUT = 1.5
MAX_REPLY = 1024


def set_fill_light(host, percent, port=DEFAULT_LIGHT_PORT, timeout=DEFAULT_TIMEOUT):
    """Set a manual brightness (0..100) or None for auto.

    Returns (ok, message). A camera bridge older than 1.3.4 has no light port;
    that is reported as a normal failure instead of an exception.
    """
    if not isinstance(host, str) or not host.strip():
        return False, '未设置相机IP'
    command = {'light': None if percent is None else int(round(float(percent)))}
    if command['light'] is not None and not 0 <= command['light'] <= 100:
        return False, '亮度应在0～100%'
    try:
        connection = socket.create_connection((host.strip(), int(port)), timeout=timeout)
    except (OSError, ValueError) as exc:
        return False, '补光连接失败：%s（需CAM1桥1.3.4）' % exc
    try:
        with connection:
            connection.settimeout(timeout)
            connection.sendall((json.dumps(command) + '\n').encode('utf-8'))
            data = bytearray()
            while b'\n' not in data and len(data) <= MAX_REPLY:
                chunk = connection.recv(256)
                if not chunk:
                    break
                data.extend(chunk)
    except OSError as exc:
        return False, '补光发送失败：%s' % exc
    try:
        reply = json.loads(bytes(data).partition(b'\n')[0].decode('utf-8'))
    except (ValueError, UnicodeDecodeError):
        return False, '补光回执无效'
    if not isinstance(reply, dict) or not reply.get('ok'):
        error = reply.get('error', '未知') if isinstance(reply, dict) else '未知'
        return False, '补光失败：%s' % error
    manual = reply.get('manual')
    if manual is None:
        return True, '补光已设为自动；相机回报%s%%，模式%s（25/60/30档需CAM1桥1.3.6）' % (reply.get('percent','未知'),reply.get('mode','未知'))
    try:
        return True, '补光已设为手动 %d%%' % int(manual)
    except (TypeError, ValueError):
        return False, '补光回执无效'
