"""同步阻塞返回后的控制优先、慢帧跳过预览，协议仍整组发送。"""
import unittest
import service
from test_service import Clock,Serial,Backend,frames


class TimingTests(unittest.TestCase):
    def setup(self):
        self.clock,self.serial,self.backend=Clock(),Serial(),Backend()
        self.server=service.VisionService(self.serial,self.backend,self.clock)
        self.serial.rx.extend(service.encode_set(3,2,2,20,6))
        self.server.tick();self.serial.output.clear()
    def test_slow_detect_sends_due_heartbeat_before_pixels_and_throttles_preview(self):
        self.setup();self.clock.value=.4
        previews=self.backend.previews
        self.backend.on_read=lambda:setattr(self.clock,'value',1.5)
        self.server.tick()
        sent=frames(self.serial.output)
        self.assertEqual([f[1]for f in sent],[service.HEARTBEAT,service.RING_PX,service.RING_COUNT])
        self.assertFalse(sent[1][2]&0x80)
        self.assertFalse(sent[2][2]&0x80)
        self.assertEqual(self.backend.previews,previews+1)
        self.assertGreaterEqual(self.server.next_preview,self.clock.value+1)
        self.assertEqual(self.server.writer.max_heartbeat_gap,1.5)
    def test_persistently_slow_detect_still_displays_first_and_later_frames(self):
        self.setup();before=self.backend.previews
        self.backend.on_read=lambda:setattr(self.clock,'value',self.clock.value+.25)
        self.server.next_preview=0
        for _ in range(12):self.server.tick()
        self.assertGreaterEqual(self.backend.previews-before,2)
        self.assertLessEqual(self.backend.previews-before,4)
    def test_slow_preview_return_services_received_stop_without_old_tail(self):
        self.setup();self.clock.value=.4;self.server.next_preview=0
        def preview(*args):
            self.clock.value=1.1
            self.serial.rx.extend(service.encode_set(0,0,0,0,7))
        self.backend.preview=preview
        self.server.tick()
        self.assertEqual(self.server.config[0],service.STOP)
        sent=frames(self.serial.output)
        ack=next(i for i,f in enumerate(sent)if f[1]==service.ACK)
        self.assertFalse(any(f[1]in (service.RING_PX,service.RING_COUNT)for f in sent[ack+1:]))


if __name__=='__main__':unittest.main()
