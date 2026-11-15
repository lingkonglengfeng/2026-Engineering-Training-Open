"""同一观测的0x03/0x83配对、时限、失败和切换隔离。"""
import unittest

import service
from test_service import Clock,Serial,Backend,frames


class CountTests(unittest.TestCase):
    def setUp(self):
        self.clock,self.serial,self.backend=Clock(),Serial(),Backend()
        self.server=service.VisionService(self.serial,self.backend,self.clock)
    def configure(self,sequence=7,station=2):
        self.serial.rx.extend(service.encode_set(3,station,2,20,sequence))
        self.server.tick()
    def observations(self):
        return [f for f in frames(self.serial.output) if f[1] in (service.RING_PX,service.RING_COUNT)]

    def test_pair_has_same_image_seq_id_and_binding_and_crc(self):
        self.configure()
        position,count=self.observations()
        self.assertEqual(position[2],count[2])
        self.assertTrue(count[2]&0x80)
        self.assertEqual(count[3:7],bytes((3,2,7,0)))
        self.assertEqual(position[1],3);self.assertEqual(count[1],0x83)

    def test_two_rings_force_invalid_position_even_if_tracker_has_point(self):
        self.backend.observe=lambda now:dict(coordinates=(10,20),id=2,scale=1,ring_count=2,ring_count_trusted=True)
        self.configure()
        position,count=self.observations()
        self.assertFalse(position[2]&0x80);self.assertEqual(position[3:7],bytes(4))
        self.assertTrue(count[2]&0x80);self.assertEqual(count[3],2)

    def test_untrusted_or_failed_count_never_turns_roi_or_cache_into_three(self):
        for metadata in ({},{'ring_count':3,'ring_count_trusted':False}):
            with self.subTest(metadata=metadata):
                self.setUp()
                self.backend.observe=lambda now:dict(coordinates=(10,20),id=2,scale=1,**metadata)
                self.configure()
                position,count=self.observations()
                self.assertFalse(position[2]&0x80);self.assertFalse(count[2]&0x80)

    def test_zero_count_can_be_trusted_but_has_no_position(self):
        self.backend.observe=lambda now:dict(coordinates=(10,20),id=2,scale=1,ring_count=0,ring_count_trusted=True)
        self.configure()
        position,count=self.observations()
        self.assertFalse(position[2]&0x80);self.assertTrue(count[2]&0x80);self.assertEqual(count[3],0)

    def test_short_group_write_completed_without_heartbeat_interleaving(self):
        self.configure();self.serial.output.clear()
        self.serial.writes=[8,0]
        self.clock.value=.1;self.server.tick()
        self.assertEqual(len(self.serial.output),8)
        self.assertEqual(self.server.writer.pending[1],service.RING_COUNT)
        self.server.writer.control(service.make_frame(service.HEARTBEAT,0,0,True,bytes((1,3,2,0))),'heartbeat')
        self.clock.value=.11;self.server.tick()
        self.clock.value=.12;self.server.tick()
        sent=frames(self.serial.output)
        self.assertEqual([f[1] for f in sent[:3]],[service.RING_PX,service.RING_COUNT,service.HEARTBEAT])
        self.assertEqual(sent[0][2],sent[1][2])

    def test_stop_discards_old_count_tail_before_ack(self):
        self.configure();self.serial.output.clear();self.serial.writes=[8]
        self.clock.value=.1;self.server.tick()
        self.assertEqual(len(self.serial.output),8)
        self.clock.value=.11
        self.serial.rx.extend(service.encode_set(0,0,0,0,8));self.server.tick()
        self.assertEqual(len(self.serial.output),8)
        self.clock.value=.17;self.server.tick()
        tail=frames(self.serial.output[8:])
        self.assertEqual([f[1] for f in tail],[service.ACK])

    def test_group_timeout_abandons_both_frames_and_waits_resync_gap(self):
        self.configure();self.serial.output.clear();self.serial.writes=[8,0,0,0]
        self.clock.value=.1;self.server.tick()
        self.clock.value=.149;self.server.tick()
        self.clock.value=.151;self.server.tick()
        self.assertEqual(len(self.serial.output),8)
        self.assertGreaterEqual(self.server.writer.resume_at,.206)
        self.assertFalse(self.server.writer.ready(.20))

    def test_count_transition_not_delayed_by_rate_limit(self):
        self.configure();self.serial.output.clear()
        self.backend.observe=lambda now:dict(coordinates=(10,20),id=2,scale=1,ring_count=2,ring_count_trusted=True)
        self.clock.value=.01;self.server.tick()
        self.assertEqual(len(self.observations()),2)
        self.assertEqual(self.observations()[-1][3],2)

    def test_same_set_retry_keeps_binding_new_set_resets_it(self):
        self.configure();self.clock.value=.1;self.configure()
        self.assertEqual(len(self.backend.configs),1)
        self.assertEqual(self.observations()[-1][5],7)
        self.clock.value=.2;self.configure(sequence=8,station=3)
        self.assertEqual(self.observations()[-1][3:7],bytes((3,3,8,0)))

    def test_throttled_frames_do_not_consume_transmitted_group_sequence(self):
        self.configure()
        for i in range(1,81):
            self.clock.value=i*.004;self.server.tick()
        positions=[f for f in self.observations() if f[1]==service.RING_PX]
        sequences=[(f[2]>>3)&15 for f in positions]
        self.assertEqual(sequences,list(range(len(sequences))))


if __name__=='__main__':unittest.main()
