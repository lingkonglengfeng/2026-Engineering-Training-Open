"""只模拟SDK流程：50%硬件PWM、异常清理、绝不操作真实灯板。"""
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import pwm50_preview as preview
import importlib.util
from pathlib import Path


class PwmPreviewTests(unittest.TestCase):
    def run_mock(self,camera_failure=False):
        events=[]
        class Output:
            def __init__(self,channel,**kwargs):events.append(('pwm',channel,kwargs))
            def enable(self):events.append(('enable',));return 0
            def duty(self,value):events.append(('duty',value));return value
            def disable(self):events.append(('disable',));return 0
        def camera(*a,**k):
            if camera_failure:raise RuntimeError('camera unavailable')
            return SimpleNamespace(close=lambda:events.append(('camera_close',)))
        peripheral=SimpleNamespace(pwm=SimpleNamespace(PWM=Output),
            pinmap=SimpleNamespace(set_pin_function=lambda pin,func:events.append(('map',pin,func))),
            gpio=SimpleNamespace(Mode=SimpleNamespace(OUT=2),Pull=SimpleNamespace(PULL_DOWN=2),
                GPIO=lambda *args:SimpleNamespace(value=lambda v:events.append(('gpio',v)))))
        maix=SimpleNamespace(app=SimpleNamespace(need_exit=lambda:True),camera=SimpleNamespace(Camera=camera),
            image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0)),
            display=SimpleNamespace(Display=lambda:SimpleNamespace(close=lambda:None),set_trans_image_quality=lambda value:None),
            err=SimpleNamespace(check_raise=lambda *args:None))
        with patch.dict(sys.modules,{'maix':maix,'maix.peripheral':peripheral}):
            if camera_failure:
                with self.assertRaisesRegex(RuntimeError,'camera unavailable'):preview.run_device()
            else:preview.run_device()
        return events

    def test_hardware_pwm7_half_duty_and_gpio_off_cleanup(self):
        events=self.run_mock()
        self.assertIn(('pwm',7,{'freq':20000,'duty':50.0,'enable':False}),events)
        self.assertIn(('map','A19','PWM7'),events)
        self.assertLess(events.index(('disable',)),len(events)-1)
        self.assertEqual([e for e in events if e[0]=='gpio'],[('gpio',0),('gpio',0)])

    def test_camera_failure_still_disables_pwm_and_restores_low(self):
        events=self.run_mock(True)
        self.assertIn(('disable',),events)
        self.assertEqual(events[-1],('gpio',0))

    def test_shared_pwm_helper_reuses_gpio_and_cleans_up_failed_start(self):
        path=Path(__file__).resolve().parents[1]/'light_gpio.py'
        spec=importlib.util.spec_from_file_location('tested_light_gpio',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        module.LIGHT_PWM_ENABLED=True
        for failed in (False,True):
            with self.subTest(failed=failed):
                events=[]
                class Output:
                    def __init__(self,channel,**kwargs):events.append(('pwm',channel,kwargs))
                    def enable(self):
                        if failed:raise RuntimeError('PWM unavailable')
                        return 0
                    def duty(self,value):events.append(('duty',value));return value
                    def disable(self):events.append(('disable',));return 0
                def gpio_object(*args):
                    self.assertFalse(any(e[0]=='create_gpio' for e in events),'GPIO requested twice')
                    events.append(('create_gpio',));return SimpleNamespace(value=lambda v:events.append(('gpio',v)))
                gpio=SimpleNamespace(GPIO=gpio_object,Mode=SimpleNamespace(OUT=2),Pull=SimpleNamespace(PULL_UP=1,PULL_DOWN=2))
                pinmap=SimpleNamespace(set_pin_function=lambda *args:events.append(('map',*args)))
                err=SimpleNamespace(check_raise=lambda *args:None)
                with patch.dict(sys.modules,{'maix.peripheral':SimpleNamespace(pwm=SimpleNamespace(PWM=Output))}):
                    if failed:
                        with self.assertRaisesRegex(RuntimeError,'PWM unavailable'):module.init_light_on(gpio,pinmap,err)
                    else:
                        handle=module.init_light_on(gpio,pinmap,err);module.light_off_on_exit(handle)
                self.assertEqual(events[-1],('gpio',0))
                self.assertIn(('disable',),events)
                self.assertIn(('pwm',7,{'freq':20000,'duty':50.0,'enable':False}),events)


if __name__=='__main__':unittest.main()
