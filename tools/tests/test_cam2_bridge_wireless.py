"""CAM2 wireless observation bridge checks: local sockets and fake UART only."""
import importlib.util
import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'vision/pc_vision'),str(ROOT/'vision')]
import wire
sys.modules['pc_wire']=wire


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


bridge=load('wireless_bridge','vision/maixcam_bridge/bridge.py')
qr=load('qr_service_wireless','vision/maixcam_qr/service.py')


def observation(gap=300,right=-100,age=0):
    payload=bytes((0x6b,0x86,age))+struct.pack('<Hh',gap,right)
    return payload+bytes((bridge.crc8(payload),))


def setup_state(mode=0):
    state=bridge.BridgeState('qr');state.connection=7
    state.accept_request(qr.encode_set(mode,4 if mode else 0,20 if mode else 0,3),1.)
    stamp=1_000_000_000;state.remember_frame(state.epoch,42,stamp)
    payload=wire.RESULT_META.pack(state.epoch,42,stamp)+observation()
    return state,stamp,payload


class FakeUart:
    def __init__(self,counts):self.counts=list(counts);self.attempts=[]
    def write(self,packet):
        self.attempts.append(bytes(packet))
        return self.counts.pop(0)


class WirelessBridgeTests(unittest.TestCase):
    def test_capability_is_explicit_and_cam1_does_not_accept_obstacle(self):
        qr_hello=bridge.BridgeState('qr').hello()
        material=bridge.BridgeState('material');material_hello=material.hello()
        self.assertEqual((qr_hello['build'],qr_hello['obstacle_uart_type'],qr_hello['obstacle_max_age_ms']),
                         ('pc-vision-bridge-1.4.1',0x86,200))
        self.assertEqual(material_hello['build'],'pc-vision-bridge-1.3.1')
        self.assertNotIn('obstacle_uart_type',material_hello)
        self.assertTrue(bridge.valid_group(observation(),'qr'))
        self.assertFalse(bridge.valid_group(observation(),'material'))
        self.assertFalse(bridge.valid_group(observation(age=1),'qr'))
        bad=observation();bad=bad[:-1]+bytes((bad[-1]^1,))
        self.assertFalse(bridge.valid_group(bad,'qr'))

    def test_stop_and_qr_accept_fresh_result_with_conservative_age(self):
        for mode in (0,6):
            state,stamp,payload=setup_state(mode)
            for age,encoded in ((0,0),(1,1),(10_000_000,1),(10_000_001,2),(200_000_000,20)):
                packet=state.result_current(7,payload,stamp+age)
                self.assertEqual(packet[2],encoded,(mode,age))
                self.assertEqual(struct.unpack('<Hh',packet[3:7]),(300,-100))
                self.assertEqual(bridge.crc8(packet[:7]),packet[7])
            self.assertIsNone(state.result_current(7,payload,stamp+200_000_001))
            self.assertIsNone(state.result_current(7,payload,stamp-1))

    def test_frame_epoch_connection_and_crc_are_not_bypassed(self):
        state,stamp,payload=setup_state()
        self.assertIsNone(state.result_current(8,payload,stamp+1))
        self.assertIsNone(state.result_current(7,wire.RESULT_META.pack(state.epoch,43,stamp)+observation(),stamp+1))
        self.assertIsNone(state.result_current(7,payload[:-1]+bytes((payload[-1]^1,)),stamp+1))
        state.accept_request(qr.encode_set(6,4,20,4),1.1)
        self.assertIsNone(state.result_current(7,payload,stamp+1))

    def test_zero_byte_retry_recomputes_age_before_first_uart_byte(self):
        state,stamp,payload=setup_state();uart=FakeUart([0,3])
        first,written=bridge.write_obstacle_first(state,uart,7,payload,stamp+11_000_000)
        self.assertEqual((first[2],written),(2,0))
        second,written=bridge.write_obstacle_first(state,uart,7,payload,stamp+61_000_000)
        self.assertEqual((second[2],written),(7,3))
        self.assertEqual(bridge.crc8(second[:7]),second[7])
        # The main UART loop continues only this suffix after the first byte.
        self.assertEqual(second[:written]+second[written:],second)

    def test_stale_or_disconnected_obstacle_never_starts_uart(self):
        state,stamp,payload=setup_state();uart=FakeUart([0])
        self.assertEqual(bridge.write_obstacle_first(state,uart,7,payload,stamp+199_000_000)[1],0)
        self.assertEqual(bridge.write_obstacle_first(state,uart,7,payload,stamp+201_000_000),(None,0))
        self.assertEqual(len(uart.attempts),1)
        state.connection=8
        self.assertEqual(bridge.write_obstacle_first(state,uart,7,payload,stamp+2),(None,0))
        self.assertEqual(len(uart.attempts),1)

    def test_qr_group_nonce_and_task_survive_obstacle_and_stop(self):
        state,stamp,payload=setup_state(6)
        task=qr.encode_task('156+123+516+231',3)
        task_payload=wire.RESULT_META.pack(state.epoch,42,stamp)+task
        self.assertEqual(state.result_current(7,task_payload,stamp+30_000_000),task)
        self.assertEqual(state.display_task,'156+123+516+231')
        self.assertIsNotNone(state.result_current(7,payload,stamp+20_000_000))
        self.assertEqual(state.display_task,'156+123+516+231')
        wrong=qr.encode_task('156+123+516+231',4)
        self.assertIsNone(state.result_current(7,wire.RESULT_META.pack(state.epoch,42,stamp)+wrong,stamp+1))
        state.accept_request(qr.encode_set(0,0,0,4),1.2)
        self.assertIsNone(state.result_current(7,task_payload,stamp+1))
        state.remember_frame(state.epoch,43,stamp)
        fresh=wire.RESULT_META.pack(state.epoch,43,stamp)+observation()
        self.assertIsNotNone(state.result_current(7,fresh,stamp+2))
        self.assertEqual(state.display_task,'156+123+516+231')

    def test_control_server_hello_and_wire_result_use_real_loopback(self):
        state=bridge.BridgeState('qr');listener=bridge.server_socket(0)
        worker=threading.Thread(target=bridge.control_server,args=(state,listener),daemon=True);worker.start()
        client=socket.create_connection(('127.0.0.1',listener.getsockname()[1]),timeout=2)
        try:
            kind,payload=wire.receive(client,2,wire.MAX_CONTROL)
            self.assertEqual(kind,wire.HELLO)
            self.assertEqual(json.loads(payload)['obstacle_uart_type'],0x86)
            state.accept_request(qr.encode_set(0,0,0,3),time.monotonic())
            kind,payload=wire.receive(client,2,wire.MAX_CONTROL)
            self.assertEqual(kind,wire.REQUEST)
            epoch=struct.unpack('!I',payload[:4])[0]
            stamp=time.monotonic_ns();state.remember_frame(epoch,42,stamp)
            payload=wire.RESULT_META.pack(epoch,42,stamp)+observation()
            client.sendall(wire.encode(wire.RESULT,payload))
            connection,received=state.results.get(timeout=2)
            uart=FakeUart([8]);packet,written=bridge.write_obstacle_first(state,uart,connection,received)
            self.assertEqual(written,8)
            self.assertLessEqual(packet[2],20)
            self.assertEqual(bridge.crc8(packet[:7]),packet[7])
        finally:
            client.close();state.stop.set();listener.close();worker.join(timeout=2)


if __name__=='__main__':unittest.main()
