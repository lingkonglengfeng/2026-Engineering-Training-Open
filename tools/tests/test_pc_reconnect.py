"""Real loopback bridge reconnect and SET replay; no hardware connections."""
import sys
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_pc_vision import Loopback, transport, backend, service


class ReconnectTests(unittest.TestCase):
    def test_network_errors_retry_promptly(self):
        self.assertEqual(transport.reconnect_delay(2, TimeoutError()), .25)
        self.assertEqual(transport.reconnect_delay(2, ConnectionError()), .25)
        self.assertEqual(transport.reconnect_delay(.1, OSError()), .1)
        self.assertEqual(transport.reconnect_delay(2, ValueError()), 2)

    def test_gui_worker_automatically_retries_network_timeout(self):
        import gui
        with tempfile.TemporaryDirectory() as temp:
            worker = gui.VisionWorker({'camera_role':'material', 'reconnect_seconds':2},
                                      Path(temp)/'config.json', 'run')
            attempts = []
            def connected(folder):
                attempts.append(folder)
                if len(attempts) == 1:
                    raise TimeoutError('stalled video socket')
                worker.stop_event.set()
            with patch.object(gui, 'HERE', Path(temp)), \
                 patch.object(worker, 'connected', side_effect=connected), \
                 patch.object(worker.stop_event, 'wait', return_value=False) as wait:
                worker.run()
            self.assertEqual(len(attempts), 2)
            wait.assert_called_once_with(.25)

    def test_bridge_replays_current_set_after_disconnect_and_honors_stop(self):
        network = Loopback()
        control = video = obj = None
        try:
            packet = service.make_frame(service.SET, 5, 6, False, bytes((2, 1, 20, 0)))
            network.state.accept_request(packet, time.monotonic())
            with tempfile.TemporaryDirectory() as temp:
                def connect_and_confirm(expected):
                    nonlocal control, video, obj
                    control = transport.ControlClient('127.0.0.1', network.control_port)
                    video = transport.VideoClient('127.0.0.1', network.video_port,
                                                  mode='push-v2', control=control)
                    obj = backend.PcBackend(video, control, Path(temp)/'cal.json', show=False)
                    engine = service.VisionService(control, obj, clock=time.perf_counter)
                    packets = []
                    deadline = time.monotonic()+4
                    while time.monotonic() < deadline:
                        engine.tick()
                        while not network.state.results.empty():
                            connection, payload = network.state.results.get_nowait()
                            accepted = network.state.result_current(connection, payload, time.monotonic_ns())
                            if accepted:
                                packets.extend(accepted[i:i+8] for i in range(0, len(accepted), 8))
                        if any(p[1] == 0x81 and p[2] & 128 and p[5] == expected for p in packets):
                            break
                        time.sleep(.005)
                    self.assertTrue(any(p[1] == 0x81 and p[2] & 128 and p[5] == expected for p in packets))
                    self.assertEqual(engine.config[0], expected)
                    return control.epoch, obj.last_meta

                epoch, first = connect_and_confirm(2)
                self.assertGreater(obj.decode_ok, 0)  # ACK requires a new decoded picture.
                obj.close(); obj = None
                video.close(); video = None
                control.close(); control = None
                # No new UART SET: bridge must replay its cached current request.
                recovered_epoch, fresh = connect_and_confirm(2)
                self.assertEqual(recovered_epoch, epoch)
                self.assertGreater(fresh[1], first[1])
                self.assertEqual(obj.config, (2, 1, 6, 20, 0))
                obj.close(); obj = None
                video.close(); video = None
                control.close(); control = None
                network.state.accept_request(service.make_frame(service.SET, 6, 0, False, bytes(4)), time.monotonic())
                connect_and_confirm(0)
                self.assertEqual(obj.config[0], 0)
        finally:
            if obj: obj.close()
            if video: video.close()
            if control: control.close()
            network.close()


if __name__ == '__main__':
    unittest.main()
