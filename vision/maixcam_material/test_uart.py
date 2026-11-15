"""固定8字节帧、数值/方向、限频及半帧恢复回归。"""
import math
import struct
import unittest
import main


class FakeUART:
    def __init__(self, writes=None):
        self.writes = list(writes or [])
        self.stream = bytearray()
        self.calls = 0

    def write(self, data):
        self.calls += 1
        result = self.writes.pop(0) if self.writes else len(data)
        if isinstance(result, Exception):
            raise result
        if result > 0:
            self.stream.extend(data[:result])
        return result


def decode(packet):
    assert len(packet) == 8 and packet[0] == 0x6B
    assert main.crc8(packet[:7]) == packet[7]
    x, y = struct.unpack('<hh', packet[3:7])
    return dict(type=packet[1], id=packet[2]&7, seq=(packet[2]>>3)&15,
                valid=bool(packet[2]&0x80), x=x/10, y=y/10)


def packets(stream):
    assert len(stream)%8 == 0
    return [decode(stream[i:i+8]) for i in range(0,len(stream),8)]


class CoordinateUARTTests(unittest.TestCase):
    def test_exact_frame_crc_and_protocol_document_vector(self):
        self.assertEqual(main.crc8(b'123456789'),0xF4)
        self.assertEqual(main.encode_coordinate_frame(1,(150.3,-16.9),2025),
                         bytes.fromhex('6B 02 C9 DF 05 A9 00 64'))
        self.assertEqual(main.encode_coordinate_frame(3,(30,-20),12),
                         bytes.fromhex('6B 02 E3 2C 01 C8 00 AF'))
        self.assertEqual(main.encode_coordinate_frame(3,None,13),
                         bytes.fromhex('6B 02 6B 00 00 00 00 29'))

    def test_valid_origin_is_not_missing_target(self):
        self.assertTrue(decode(main.encode_coordinate_frame(1,(0,0),0))['valid'])
        self.assertFalse(decode(main.encode_coordinate_frame(1,None,0))['valid'])
        self.assertEqual(decode(main.encode_coordinate_frame(1,(-.01,-.01),0))['x'],0)

    def test_invalid_config_rejected_and_bad_measurement_invalidated(self):
        for arguments in [(0,(0,0),0),(1,(0,0),65536)]:
            with self.assertRaises(ValueError): main.encode_coordinate_frame(*arguments)
        for xy in [(math.nan,0),(0,math.inf),(3276.8,0),(-3276.9,0)]:
            record=decode(main.encode_coordinate_frame(1,xy,0))
            self.assertFalse(record['valid']);self.assertEqual((record['x'],record['y']),(0,0))

    def test_rounding_ranges_scale_and_color_mapping(self):
        self.assertEqual(main.fixed10(.05),1)
        self.assertEqual(main.fixed10(-.05),-1)
        record=decode(main.encode_coordinate_frame(3,(40,60),0,2))
        self.assertEqual((record['x'],record['y']),(20,-30))
        self.assertEqual(decode(main.encode_coordinate_frame(5,(0,0),0))['id'],6)
        self.assertEqual(decode(main.encode_coordinate_frame(6,(0,0),0))['id'],5)
        self.assertTrue(decode(main.encode_coordinate_frame(1,(-3276.8,-3276.7),0))['valid'])

    def test_rate_limit_latest_coordinates_and_image_sequence(self):
        serial=FakeUART();sender=main.CoordinateSender(serial,hz=20)
        self.assertTrue(sender.send(3,(1,2),0))
        self.assertFalse(sender.send(3,(4,5),.01))
        self.assertFalse(sender.send(3,(7,8),.04))
        self.assertTrue(sender.send(3,(10,11),.05))
        records=packets(serial.stream)
        self.assertEqual(len(records),2)
        self.assertEqual((records[1]['x'],records[1]['y'],records[1]['seq']),(10,-11,3))

    def test_loss_recovery_and_color_change_send_immediately(self):
        sender=main.CoordinateSender(FakeUART(),hz=20)
        self.assertTrue(sender.send(3,(10,20),0))
        self.assertTrue(sender.send(3,None,.01))
        self.assertTrue(sender.send(3,(5,6),.02))
        self.assertTrue(sender.send(6,(7,8),.03))
        self.assertEqual(sender.sent,4)
        self.assertEqual(sender.last_state,(6,True))

    def test_sequence_wrap(self):
        serial=FakeUART();sender=main.CoordinateSender(serial);sender.sequence=15
        sender.send(3,(1,2),0);sender.send(3,(1,2),.05)
        self.assertEqual([record['seq'] for record in packets(serial.stream)],[15,0])
        self.assertEqual(sender.sequence,1)

    def test_partial_write_resumes_without_mixing_frames(self):
        serial=FakeUART([3]);sender=main.CoordinateSender(serial)
        self.assertFalse(sender.send(3,(10,20),0))
        sender.send(3,(30,40),.01)
        self.assertEqual(bytes(serial.stream),main.encode_coordinate_frame(3,(10,20),0))
        self.assertEqual((sender.sent,sender.errors),(1,0))

    def test_partial_valid_frame_aborted_on_target_loss(self):
        serial=FakeUART([3]);sender=main.CoordinateSender(serial)
        sender.send(3,(10,20),0)
        self.assertFalse(sender.send(3,None,.01))
        self.assertEqual(len(serial.stream),3)
        self.assertTrue(sender.send(3,None,.07))
        self.assertEqual(bytes(serial.stream),main.encode_coordinate_frame(3,(10,20),0)[:3]
                         +main.encode_coordinate_frame(3,None,2))

    def test_write_error_uses_receiver_timeout_before_retry(self):
        for failure in [-1,OSError('test UART error')]:
            serial=FakeUART([failure]);sender=main.CoordinateSender(serial)
            self.assertFalse(sender.send(3,(1,2),0))
            self.assertFalse(sender.send(3,(2,3),.05))
            self.assertTrue(sender.send(3,(3,4),.06))
            self.assertEqual(sender.errors,1)
            self.assertEqual(bytes(serial.stream),main.encode_coordinate_frame(3,(3,4),2))

    def test_stalled_write_is_bounded_and_fresh_after_recovery(self):
        serial=FakeUART([0]);sender=main.CoordinateSender(serial)
        sender.send(3,(1,2),0)
        self.assertFalse(sender.send(3,(3,4),.06))
        self.assertEqual(sender.pending,b'')
        self.assertEqual(sender.errors,1)
        sender.send(3,(5,6),.12)
        self.assertEqual(decode(bytes(serial.stream))['x'],5)

    def test_invalid_numeric_measurement_bypasses_valid_rate_limit(self):
        serial=FakeUART();sender=main.CoordinateSender(serial)
        sender.send(3,(1,2),0);sender.send(3,(math.nan,2),.01)
        self.assertFalse(packets(serial.stream)[-1]['valid'])

    def test_force_exit_invalidates_before_next_deadline(self):
        serial=FakeUART();sender=main.CoordinateSender(serial)
        sender.send(3,None,0)
        self.assertTrue(sender.send(3,None,.01,force=True))
        self.assertEqual(sender.sent,2)

    def test_default_blue_presets_split_screenshot_estimates(self):
        blue = main.COLORS[2][2]
        light = main.COLORS[5][2]
        for threshold, own, other in [(blue, [31, 36, -64], [58, 0, -46]),
                                      (light, [58, 0, -46], [31, 36, -64])]:
            self.assertTrue(all(threshold[2*i] <= own[i] <= threshold[2*i+1]
                                for i in range(3)))
            self.assertFalse(all(threshold[2*i] <= other[i] <= threshold[2*i+1]
                                 for i in range(3)))


if __name__ == '__main__':
    unittest.main()
