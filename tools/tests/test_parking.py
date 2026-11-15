"""Real parked reference, transformed frames and real PC protocol over fake UART."""
from pathlib import Path
import sys,unittest,cv2,numpy as np
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'vision/pc_vision'),str(ROOT/'vision/maixcam_service')]
from parking import ParkingDetector,features
from backend import PcBackend
import service

def photo():return cv2.imdecode(np.fromfile(str(ROOT/'vision/pc_vision/test_data/parking/reference_raw.png'),np.uint8),1)
class Serial:
 def __init__(self):self.rx=bytearray();self.output=bytearray()
 def read(self,n,timeout=0):v=self.rx[:n];del self.rx[:n];return bytes(v)
 def write(self,v):self.output.extend(v);return len(v)
class ParkingTests(unittest.TestCase):
 def test_reference_is_zero_and_raw_unchanged(self):
  image=photo();copy=image.copy();xy,axis,overlay,r=ParkingDetector().observe(image)
  self.assertEqual(r['reference']['start_zone'],1);self.assertAlmostEqual(xy[0],0);self.assertAlmostEqual(xy[1],0)
  self.assertAlmostEqual(axis['degrees'],0);self.assertTrue(np.array_equal(image,copy));self.assertFalse(np.array_equal(image,overlay))
 def test_shift_and_rotation_follow_visible_edge_not_occluded_centroid(self):
  im=photo();d=ParkingDetector()
  shifted=cv2.warpAffine(im,np.float32([[1,0,-15],[0,1,-8]]),(480,360),borderValue=(150,150,150))
  xy,axis,_,r=d.observe(shifted);self.assertLess(abs(xy[0]+15),1);self.assertLess(abs(xy[1]-8),1)
  turned=cv2.warpAffine(im,cv2.getRotationMatrix2D((300,206),2,1),(480,360),borderValue=(150,150,150))
  xy,axis,_,r=d.observe(turned);self.assertTrue(r['valid']);self.assertLess(abs(axis['degrees']-2),.5)
 def test_blank_and_clipped_are_invalid_and_bottom_occlusion_keeps_reference(self):
  d=ParkingDetector();im=photo();im[280:]=20
  self.assertTrue(d.observe(im)[3]['valid'])
  self.assertIsNone(d.observe(np.full_like(im,150))[0])
  shifted=cv2.warpAffine(photo(),np.float32([[1,0,30],[0,1,0]]),(480,360))
  self.assertIsNone(d.observe(shifted)[0])
 def backend(self):
  class Video:
   def __init__(self):self.n=0
   def get(self,epoch):
    self.n+=1;jpg=cv2.imencode('.jpg',photo(),[cv2.IMWRITE_JPEG_QUALITY,95])[1].tobytes()
    return (epoch,self.n,self.n*1000000,480,360,10,10,10),jpg,0.
  return PcBackend(Video(),NS(epoch=0,hello={'rotation':0},observation_status=lambda:{}),ROOT/'tmp/nonexistent_parking_calibration.json',False)
 def test_mode_validation_and_real_backend_protocol_group(self):
  good=service.encode_set(9,5,2,20,1,1);self.assertEqual(service.validate_set(good),0)
  self.assertNotEqual(service.validate_set(service.encode_set(9,2,2,20,1,1)),0)
  self.assertEqual(service.validate_set(service.encode_set(4,5,2,20,1,1)),3)
  obj=self.backend();uart=Serial();clock=[1.];engine=service.VisionService(uart,obj,lambda:clock[0])
  engine.request(good,clock[0]);clock[0]+=.1;engine.tick()
  frames=service.FrameParser().feed(uart.output,clock[0]);types=[f[1] for f in frames]
  self.assertIn(service.ACK,types);self.assertIn(service.RING_PX,types);self.assertIn(service.RING_AXIS,types);self.assertIn(service.RING_COUNT,types)
  group=[f for f in frames if f[1] in (service.RING_PX,service.RING_AXIS,service.RING_COUNT)]
  self.assertEqual(len(group),3);self.assertTrue(all(f[2]&128 for f in group));self.assertEqual(group[-1][4],5)
  self.assertTrue(obj.parking_report['valid']);self.assertIn('PARK BLUE',obj.status)

if __name__=='__main__':unittest.main()
