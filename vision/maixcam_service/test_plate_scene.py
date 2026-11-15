"""0x09+0x85：区分正常可搜索、候选确认、歧义、有效目标与处理故障。"""
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import service
from test_service import Clock,Serial,Backend,frames,NativeImage


class PlateSceneWireTests(unittest.TestCase):
    def setUp(self):
        self.clock,self.serial,self.backend=Clock(),Serial(),Backend()
        self.server=service.VisionService(self.serial,self.backend,self.clock)
    def configure(self,seq=7,station=1):
        self.serial.rx.extend(service.encode_set(8,station,0,20,seq))
        self.server.tick()
    def pair(self):
        return [f for f in frames(self.serial.output) if f[1] in (service.PLATE_PX,service.PLATE_SCENE)][-2:]

    def test_found_pair_same_image_seq_id_and_station_nonce(self):
        self.configure()
        px,scene=self.pair()
        self.assertEqual(px[2],scene[2]);self.assertTrue(px[2]&0x80)
        self.assertEqual(px[2]&7,0);self.assertEqual(scene[1],0x85)
        self.assertEqual(scene[3:7],bytes((3,1,7,0)))

    def test_search_and_acquire_send_healthy_state_before_pixel_confirmation(self):
        for state in (1,2):
            with self.subTest(state=state):
                self.setUp()
                self.backend.observe=lambda now:dict(coordinates=None,id=0,scale=1,plate_scene=state,plate_scene_trusted=True)
                self.configure()
                px,scene=self.pair()
                self.assertFalse(px[2]&0x80);self.assertEqual(px[3:7],bytes(4))
                self.assertTrue(scene[2]&0x80);self.assertEqual(scene[3],state)
                self.assertEqual((px[2]>>3)&15,(scene[2]>>3)&15)

    def test_unknown_missing_untrusted_and_inconsistent_state_never_authorize_search(self):
        for state,trusted,coords in ((0,True,None),(99,True,None),(1,False,None),(1,True,(1,2)),(3,True,None)):
            with self.subTest(state=state,trusted=trusted,coords=coords):
                self.setUp()
                self.backend.observe=lambda now:dict(coordinates=coords,id=0,scale=1,plate_scene=state,plate_scene_trusted=trusted)
                self.configure()
                px,scene=self.pair()
                self.assertFalse(px[2]&0x80);self.assertFalse(scene[2]&0x80)
                self.assertEqual(scene[3],0)
        self.setUp()
        self.backend.observe=lambda now:dict(coordinates=None,id=0,scale=1)
        self.configure();self.assertFalse(self.pair()[1][2]&0x80)

    def test_nonfinite_and_unencodable_found_coordinates_invalidate_scene(self):
        for coords in ((float('nan'),0),(1e10,0)):
            self.setUp()
            self.backend.observe=lambda now:dict(coordinates=coords,id=0,scale=1,plate_scene=3,plate_scene_trusted=True)
            self.configure()
            px,scene=self.pair()
            self.assertFalse(px[2]&0x80);self.assertFalse(scene[2]&0x80);self.assertEqual(scene[3],0)

    def test_observation_failure_sends_unknown_invalid_pair_and_clears_tracker(self):
        resets=[];self.backend.invalidate=lambda:resets.append(True)
        self.backend.observe=lambda now:(_ for _ in ()).throw(RuntimeError('camera empty'))
        self.configure()
        px,scene=self.pair()
        self.assertFalse(px[2]&0x80);self.assertFalse(scene[2]&0x80)
        self.assertEqual(scene[3:7],bytes((0,1,7,0)));self.assertEqual(resets,[True])

    def test_state_transition_bypasses_rate_limit(self):
        self.backend.observe=lambda now:dict(coordinates=None,id=0,scale=1,plate_scene=1,plate_scene_trusted=True)
        self.configure();self.serial.output.clear()
        self.backend.observe=lambda now:dict(coordinates=None,id=0,scale=1,plate_scene=2,plate_scene_trusted=True)
        self.clock.value=.01;self.server.tick()
        self.assertEqual(self.pair()[1][3],2)

    def test_short_pair_no_control_interleaving_and_stop_discards_old_tail(self):
        self.configure();self.serial.output.clear();self.serial.writes=[8]
        self.clock.value=.1;self.server.tick()
        self.assertEqual(len(self.serial.output),8)
        self.assertEqual(self.server.writer.pending[1],0x85)
        self.clock.value=.11
        self.serial.rx.extend(service.encode_set(0,0,0,0,8));self.server.tick()
        self.clock.value=.17;self.server.tick()
        self.assertEqual([f[1] for f in frames(self.serial.output[8:])],[service.ACK])

    def test_old_result_discarded_when_new_station_set_arrives_during_detection(self):
        self.configure();self.serial.output.clear()
        self.backend.on_read=lambda:self.serial.rx.extend(service.encode_set(8,2,0,20,8))
        self.clock.value=.1;self.server.tick()
        self.assertEqual(frames(self.serial.output),[])
        self.backend.on_read=None;self.clock.value=.2;self.server.tick()
        self.assertEqual(self.pair()[1][3:7],bytes((3,2,8,0)))

    def test_plate_scene_does_not_change_ring_packet_or_set_nonce(self):
        self.configure()
        self.clock.value=.1;self.serial.rx.extend(service.encode_set(3,2,2,20,8));self.server.tick()
        sent=frames(self.serial.output)
        self.assertEqual([f[1] for f in sent[-3:]],[service.ACK,service.RING_PX,service.RING_COUNT])
        self.assertEqual(sent[-1][3:7],bytes((3,2,8,0)))


class PlateBackendSceneTests(unittest.TestCase):
    def backend(self,array):
        class Frame(NativeImage):
            def __init__(self,pixels):self.array=pixels
            def width(self):return self.array.shape[1]
            def height(self):return self.array.shape[0]
            def draw_ellipse(self,*a,**k):pass
            def draw_line(self,*a,**k):pass
        image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb'),image2cv=lambda f,**k:f.array,
            COLOR_WHITE=0,COLOR_YELLOW=1,COLOR_GREEN=2,COLOR_RED=3)
        camera=SimpleNamespace(Camera=lambda *a,**k:SimpleNamespace(read=lambda **kw:Frame(array)))
        result=service.MaixBackend(camera,image,SimpleNamespace(width=lambda:320,height=lambda:240))
        result.show_message=lambda *a:None
        result.configure((8,1,0,20,0))
        return result

    def test_real_plate_photo_confirming_then_found_without_threshold_changes(self):
        path=Path(__file__).resolve().parents[1]/'maixcam_material/test_data/plate_lit.png'
        bgr=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_COLOR)
        backend=self.backend(cv2.cvtColor(cv2.resize(bgr,(480,360)),cv2.COLOR_BGR2RGB))
        observations=[backend.observe(time.monotonic()) for _ in range(3)]
        self.assertEqual([o['plate_scene'] for o in observations],[2,2,3])
        self.assertTrue(all(o['plate_scene_trusted'] for o in observations))
        self.assertIsNone(observations[0]['coordinates']);self.assertIsNotNone(observations[-1]['coordinates'])

    def test_real_blank_finished_detection_is_search_not_fault(self):
        backend=self.backend(np.full((360,480,3),180,np.uint8))
        observation=backend.observe(time.monotonic())
        self.assertEqual(observation['plate_scene'],1);self.assertTrue(observation['plate_scene_trusted'])
        self.assertIsNone(observation['coordinates'])

    def test_ambiguity_and_unknown_reason_are_distinct_from_search(self):
        for reason,state,trusted in [('AMBIGUOUS_EDGE',2,True),('NO_EDGE_ARC',1,True),('PLATE_ERROR',0,False),('NEW_UNKNOWN_REASON',0,False)]:
            backend=self.backend(np.full((360,480,3),180,np.uint8))
            def detect(frame,image,diagnostics,edges):
                diagnostics.update(reason=reason);return None,frame
            with patch.object(backend.module,'detect_plate_color',side_effect=detect):
                observation=backend.observe(time.monotonic())
            self.assertEqual(observation['plate_scene'],state);self.assertEqual(observation['plate_scene_trusted'],trusted)


if __name__=='__main__':unittest.main()
