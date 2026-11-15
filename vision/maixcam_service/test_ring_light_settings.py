"""Ring brightness request is LED PWM; switching modes must not touch ISP or AE."""
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import service
from test_service import Clock,Serial,frames


class RingLightSettingsTests(unittest.TestCase):
    def module(self):
        return SimpleNamespace(load_samples=lambda:{},PlateTracker=lambda:object(),
            MiddleTracker=lambda:object(),RingDetector=lambda:SimpleNamespace(),
            AlignmentState=lambda:object(),cv2=SimpleNamespace(setNumThreads=lambda n:None))

    def test_ring15_material20_plate30_stop50_on_same_pwm_and_active_low_inversion(self):
        for off in (0,1):
            events=[]
            output=SimpleNamespace(duty=lambda duty:events.append(('duty',duty)) or duty,
                                   disable=lambda:events.append(('disable',)) or 0)
            light=service._PwmLight(output,None,SimpleNamespace(set_pin_function=lambda *args:events.append(('map',*args))),
                SimpleNamespace(check_raise=lambda *args:None))
            light.off_gpio=SimpleNamespace(value=lambda value:events.append(('off',value)))
            camera=SimpleNamespace(clear_buff=lambda:None,close=lambda:None)
            backend=service.MaixBackend(SimpleNamespace(Camera=lambda *args,**kwargs:camera),
                SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb')),None)
            backend.light=light
            with patch.object(service,'LIGHT_OFF_LEVEL',off),patch.object(service,'detector_module',return_value=self.module()):
                for flags in (0,1,2,3):backend.configure((3,2 if flags<2 else 3,2,20,flags))
                for config in ((2,1,1,20,0),(8,1,0,20,0),(0,0,0,0,0)):backend.configure(config)
                self.assertIs(light.output,output)
                duties=[event[1] for event in events if event[0]=='duty']
                self.assertEqual(duties,[15 if off==0 else 85]*4+
                                 [20 if off==0 else 80,30 if off==0 else 70,50])
                service.light_off_on_exit(light)
                self.assertEqual(events[-1],('off',off));self.assertIn(('disable',),events)
                backend.close()

    def test_failed_ring_duty_change_rolls_back_before_failure_ack(self):
        calls=[]
        def duty(value):
            calls.append(value)
            return -1 if value==15 else value
        light=service._PwmLight(SimpleNamespace(duty=duty),None,None,None)
        backend=service.MaixBackend(None,None,None);backend.light=light
        clock,serial=Clock(),Serial();server=service.VisionService(serial,backend,clock)
        with patch.object(service,'report_fault'):
            server.request(service.encode_set(3,2,2,20,7,3),0)
        server.writer.pump(0)
        packet=frames(serial.output)[0]
        self.assertEqual(calls,[15,50])
        self.assertEqual(packet[1],service.ACK);self.assertEqual(packet[4],7)
        self.assertFalse(packet[2]&0x80);self.assertEqual(server.config[0],service.STOP)

    def test_invalid_or_unapplied_duty_is_not_accepted(self):
        light=service._PwmLight(SimpleNamespace(duty=lambda value:0),None,None,None)
        with self.assertRaises(RuntimeError):light.set_brightness(25)
        for value in (-1,101,None,float('nan')):
            with self.assertRaises(ValueError):light.set_brightness(value)
    def test_all_profiles_and_mode_switches_never_touch_camera_brightness(self):
        class Camera:
            closed=False
            def close(self):self.closed=True
            def clear_buff(self):pass
            def luma(self,*args):raise AssertionError('User requested lamp PWM, not ISP luma')
            def exposure(self,*args):raise AssertionError('Exposure must stay unchanged')
            def gain(self,*args):raise AssertionError('Gain must stay unchanged')
            def exp_mode(self,*args):raise AssertionError('AE must stay unchanged')
        cam=Camera()
        backend=service.MaixBackend(SimpleNamespace(Camera=lambda *a,**k:cam),
            SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb')),None)
        module=SimpleNamespace(load_samples=lambda:{},PlateTracker=lambda:object(),
            MiddleTracker=lambda:object(),RingDetector=lambda:SimpleNamespace(),
            AlignmentState=lambda:object(),cv2=SimpleNamespace(setNumThreads=lambda n:None))
        with patch.object(service,'detector_module',return_value=module):
            for flags in (0,1,2,3):backend.configure((3,2 if flags<2 else 3,2,20,flags))
            for config in ((2,1,1,20,0),(8,1,0,20,0),(0,0,0,0,0)):backend.configure(config)
            backend.close()
        self.assertTrue(cam.closed)


if __name__=='__main__':unittest.main()
