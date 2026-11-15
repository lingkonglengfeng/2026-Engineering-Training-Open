"""PC fill-light client and settings; loopback TCP bridge stand-in, no cameras."""
import json
import socket
import sys
import tempfile
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'vision/pc_vision'))
import light_control
from settings import read_config


class FakeBridge:
    """Accept one command, record it, answer with a canned JSON line."""
    def __init__(self, reply=None, raw=None):
        self.reply, self.raw, self.commands = reply, raw, []
        self.server = socket.socket()
        self.server.bind(('127.0.0.1', 0))
        self.server.listen(1)
        self.port = self.server.getsockname()[1]
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            connection, _ = self.server.accept()
            with connection:
                data = bytearray()
                while b'\n' not in data and len(data) < 1024:
                    chunk = connection.recv(256)
                    if not chunk:
                        break
                    data.extend(chunk)
                line = bytes(data).partition(b'\n')[0]
                if line:
                    self.commands.append(json.loads(line))
                payload = self.raw if self.raw is not None else json.dumps(self.reply).encode('utf-8') + b'\n'
                connection.sendall(payload)
        except OSError:
            pass

    def close(self):
        self.server.close()
        self.thread.join(1)


class LightControlTests(unittest.TestCase):
    def test_manual_brightness_reaches_bridge_and_is_reported(self):
        bridge = FakeBridge({'ok': True, 'percent': 60, 'manual': 60, 'mode': 2})
        try:
            ok, message = light_control.set_fill_light('127.0.0.1', 60, bridge.port, timeout=1)
        finally:
            bridge.close()
        self.assertTrue(ok)
        self.assertIn('手动 60%', message)
        self.assertEqual(bridge.commands, [{'light': 60}])

    def test_auto_sends_null_and_reports_mode_default(self):
        bridge = FakeBridge({'ok': True, 'percent': 70, 'manual': None, 'mode': 8})
        try:
            ok, message = light_control.set_fill_light('127.0.0.1', None, bridge.port, timeout=1)
        finally:
            bridge.close()
        self.assertTrue(ok)
        self.assertIn('自动', message)
        self.assertIn('回报70%', message)  # An old camera must not be reported as60%.
        self.assertIn('模式8', message)
        self.assertEqual(bridge.commands, [{'light': None}])

    def test_percent_is_rounded_and_out_of_range_is_rejected_before_connecting(self):
        bridge = FakeBridge({'ok': True, 'percent': 40, 'manual': 40, 'mode': None})
        try:
            ok, _ = light_control.set_fill_light('127.0.0.1', 40.4, bridge.port, timeout=1)
        finally:
            bridge.close()
        self.assertTrue(ok)
        self.assertEqual(bridge.commands, [{'light': 40}])
        ok, message = light_control.set_fill_light('127.0.0.1', 101, 1, timeout=.1)
        self.assertFalse(ok)
        self.assertIn('0～100', message)
        ok, message = light_control.set_fill_light('', 50, 1, timeout=.1)
        self.assertFalse(ok)
        self.assertIn('未设置相机IP', message)

    def test_unreachable_old_bridge_and_bad_reply_are_normal_failures(self):
        ok, message = light_control.set_fill_light('127.0.0.1', 50, 1, timeout=.2)
        self.assertFalse(ok)
        self.assertIn('连接失败', message)
        self.assertIn('1.3.4', message)
        bridge = FakeBridge(raw=b'not json\n')
        try:
            ok, message = light_control.set_fill_light('127.0.0.1', 50, bridge.port, timeout=1)
        finally:
            bridge.close()
        self.assertFalse(ok)
        self.assertIn('回执无效', message)

    def test_bridge_error_reply_is_forwarded(self):
        bridge = FakeBridge({'ok': False, 'error': 'light unavailable'})
        try:
            ok, message = light_control.set_fill_light('127.0.0.1', 50, bridge.port, timeout=1)
        finally:
            bridge.close()
        self.assertFalse(ok)
        self.assertIn('light unavailable', message)


class LightSettingsTests(unittest.TestCase):
    def test_light_port_and_fill_light_round_trip_and_validation(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'config.json'
            path.write_text(json.dumps({'fill_light': 45, 'light_port': 9014}))
            config = read_config(path)
            self.assertEqual(config['fill_light'], 45)
            self.assertEqual(config['light_port'], 9014)
            path.write_text(json.dumps({'fill_light': None}))
            self.assertIsNone(read_config(path)['fill_light'])
            self.assertEqual(read_config(Path(temp) / 'missing.json')['light_port'],
                             light_control.DEFAULT_LIGHT_PORT)
            for bad in ({'fill_light': 101}, {'fill_light': True}, {'fill_light': 'manual'},
                        {'light_port': 0}, {'light_port': 70000}):
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    read_config(path)


if __name__ == '__main__':
    unittest.main()
