"""CAM1 PWM modes/cleanup with fake Maix peripherals; no physical devices."""
import importlib.util,io,json,socket,sys,threading,unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/pc_vision'))
import wire
sys.modules['pc_wire']=wire
spec=importlib.util.spec_from_file_location('fill_bridge',ROOT/'vision/maixcam_bridge/bridge.py')
bridge=importlib.util.module_from_spec(spec);spec.loader.exec_module(bridge)

class FillLightTests(unittest.TestCase):
    def environment(self,broken_on=False,broken_map=False,broken_uart=False):
        events=[];lamp=NS()
        def value(level):
            events.append(('level',level))
        lamp.value=value
        def GPIO(name,mode,pull):events.append(('gpio',name,mode,pull));return lamp
        def mapping(pin,function):events.append(('map',pin,function));return 1 if broken_map and pin=='A18' else 0
        def check(code,message):
            if code:raise RuntimeError(message)
        def PWM(channel,**kwargs):
            events.append(('pwm',channel,kwargs))
            def enable():
                events.append(('enable',))
                if broken_on:raise RuntimeError('light test failure')
                return 0
            return NS(enable=enable,disable=lambda:events.append(('disable',)),
                      duty=lambda level:events.append(('duty',level)))
        def UART(*args,**kwargs):
            events.append(('uart',args[0]))
            if broken_uart:raise RuntimeError('uart test failure')
            return NS(close=lambda:events.append(('serial_close',)))
        uart=NS(UART=UART,BITS=NS(BITS_8=8),PARITY=NS(PARITY_NONE=0),STOP=NS(STOP_1=1))
        per=NS(gpio=NS(GPIO=GPIO,Mode=NS(OUT='OUT'),Pull=NS(PULL_DOWN='DOWN')),pinmap=NS(set_pin_function=mapping),uart=uart,pwm=NS(PWM=PWM))
        maix=NS(err=NS(check_raise=check),app=NS(need_exit=lambda:True),camera=NS(),image=NS(),
                network=NS(wifi=NS(Wifi=lambda:NS(get_ip=lambda:'FAKE'))),display=NS(Display=lambda:NS()))
        return events,{'maix':maix,'maix.peripheral':per}
    def run_fake(self,role='material',**kwargs):
        events,modules=self.environment(**kwargs)
        listener=NS(close=lambda:events.append(('listener_close',)))
        thread=lambda *a,**k:NS(start=lambda:None,join=lambda **k:None)
        with patch.dict(sys.modules,modules),patch.object(bridge,'BRIDGE_ROLE',role),patch.object(bridge,'server_socket',return_value=listener),patch.object(bridge,'discovery_socket',return_value=listener),patch.object(bridge.threading,'Thread',side_effect=thread),redirect_stdout(io.StringIO()) as output:
            bridge.run()
        return events,output.getvalue()
    def test_cam1_pwm_on_start_gpio_low_on_exit_uart_pins_unchanged(self):
        events,output=self.run_fake()
        self.assertIn(('map','A18','GPIOA18'),events)
        self.assertIn(('gpio','GPIOA18','OUT','DOWN'),events)
        self.assertIn(('map','A16','UART0_TX'),events);self.assertIn(('map','A17','UART0_RX'),events)
        self.assertIn(('map','A18','PWM6'),events)
        self.assertIn(('pwm',6,{'freq':20000,'duty':100,'enable':False}),events)
        self.assertEqual([e for e in events if e[0]=='level'],[('level',0)])
        self.assertIn(('duty',0),events);self.assertIn(('disable',),events)
        self.assertLess(events.index(('level',0)),events.index(('serial_close',)))
        self.assertIn('startup100%',output);self.assertIn('LOW/OFF',output)
        self.assertFalse(any('A19' in str(e) for e in events))
    def test_cam2_initializes_no_light_pin(self):
        events,_=self.run_fake('qr')
        self.assertFalse(any(e[0] in ('gpio','level','pwm','duty') for e in events))
        self.assertEqual([e[1] for e in events if e[0]=='map'],['A16','A17'])
    def test_qr_helper_does_not_even_import_maix(self):
        with patch.dict(sys.modules,{'maix':None,'maix.peripheral':None}):
            self.assertIsNone(bridge.init_fill_light('qr'))
    def test_light_mapping_failure_reports_it_and_keeps_uart_bridge_alive(self):
        events,output=self.run_fake(broken_map=True)
        self.assertIn('unavailable; video/UART continue',output);self.assertNotIn('HIGH/ON',output)
        self.assertIn(('serial_close',),events)
    def test_partial_gpio_start_failure_attempts_off_and_keeps_bridge_alive(self):
        events,output=self.run_fake(broken_on=True)
        self.assertEqual([e for e in events if e[0]=='level'],[('level',0)])
        self.assertIn(('serial_close',),events);self.assertIn('unavailable',output)
    def test_later_startup_error_still_turns_light_off(self):
        events,modules=self.environment(broken_uart=True)
        with patch.dict(sys.modules,modules),patch.object(bridge,'BRIDGE_ROLE','material'),redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'uart test failure'):bridge.run()
        self.assertEqual([e for e in events if e[0]=='level'],[('level',0)])

    def test_disc60_material25_ring30_and_parking100_without_cam2_or_uart_changes(self):
        events,modules=self.environment()
        with patch.dict(sys.modules,modules),redirect_stdout(io.StringIO()):
            light=bridge.init_fill_light('material');state=bridge.BridgeState();state.fill_light=light
            for mode in (8,2,2,3,9,0,8):
                state.accept_request(bytes((0x6B,0x80,0,mode,1,20,0,0)),1)
            bridge.close_fill_light(light)
        self.assertEqual([e for e in events if e[0]=='duty'],[('duty',60),('duty',25),('duty',30),('duty',100),('duty',60),('duty',0)])
        self.assertEqual([e for e in events if e[0]=='enable'],[('enable',)])
        self.assertFalse(any('A16' in str(e) or 'A17' in str(e) or 'A19' in str(e) for e in events))

    def new_light(self):
        events,modules=self.environment()
        with patch.dict(sys.modules,modules),redirect_stdout(io.StringIO()):
            light=bridge.init_fill_light('material')
        return events,light

    def test_manual_brightness_overrides_mode_and_auto_restores(self):
        events,light=self.new_light();state=bridge.BridgeState();state.fill_light=light
        reply=bridge.light_command(state,'{"light": 70}')
        self.assertEqual(reply,{'ok':True,'percent':70,'manual':70,'mode':None})
        state.accept_request(bytes((0x6B,0x80,0,2,1,20,0,0)),1)  # material default is25
        self.assertEqual(light.percent,70)
        self.assertEqual(bridge.light_command(state,'{"light":"auto"}')['percent'],25)
        state.accept_request(bytes((0x6B,0x80,0,3,1,20,0,0)),2)
        bridge.close_fill_light(light)
        self.assertEqual([e for e in events if e[0]=='duty'],
                         [('duty',70),('duty',25),('duty',30),('duty',0)])

    def test_light_command_validation_clamp_and_unavailable_light(self):
        _,light=self.new_light();state=bridge.BridgeState();state.fill_light=light
        self.assertFalse(bridge.light_command(state,'not json')['ok'])
        self.assertFalse(bridge.light_command(state,'{"light":"x"}')['ok'])
        self.assertFalse(bridge.light_command(state,'{}')['ok'])
        self.assertTrue(bridge.light_command(state,'{"light":150}')['ok'])
        self.assertEqual(light.percent,100)
        self.assertTrue(bridge.light_command(state,'{"light":-20}')['ok'])
        self.assertEqual(light.percent,0)
        status=bridge.light_command(state,'{"status":true}')
        self.assertTrue(status['ok']);self.assertEqual(status['manual'],0)
        self.assertFalse(bridge.light_command(bridge.BridgeState(),'{"light":40}')['ok'])

    def test_light_port_only_for_material_and_hello_reports_light(self):
        for role,expected in (('material',True),('qr',False)):
            events,modules=self.environment();ports=[]
            def sock(port): ports.append(port); return NS(close=lambda:None)
            thread=lambda *a,**k:NS(start=lambda:None,join=lambda **k:None)
            with patch.dict(sys.modules,modules),patch.object(bridge,'BRIDGE_ROLE',role),\
                 patch.object(bridge,'server_socket',side_effect=sock),\
                 patch.object(bridge,'discovery_socket',return_value=NS(close=lambda:None)),\
                 patch.object(bridge.threading,'Thread',side_effect=thread),redirect_stdout(io.StringIO()):
                bridge.run()
            self.assertEqual(bridge.LIGHT_PORT in ports,expected)
        events,modules=self.environment()
        with patch.dict(sys.modules,modules),redirect_stdout(io.StringIO()):
            state=bridge.BridgeState()
            self.assertEqual(state.hello()['light_port'],bridge.LIGHT_PORT)
            self.assertIsNone(state.hello()['light'])
            state.fill_light=bridge.init_fill_light('material')
        self.assertEqual(state.hello()['light']['percent'],100)

    def test_command_server_answers_one_json_line(self):
        _,light=self.new_light();state=bridge.BridgeState();state.fill_light=light
        left,right=socket.socketpair()
        class Listener:
            def __init__(self):self.n=0
            def accept(self):
                self.n+=1
                if self.n==1:return right,None
                raise socket.timeout()
        thread=threading.Thread(target=bridge.command_server,args=(state,Listener()),daemon=True)
        thread.start()
        try:
            left.sendall(b'{"light": 42}\n')
            line=left.makefile('rb').readline()
        finally:
            state.stop.set();thread.join(1);left.close()
        self.assertEqual(json.loads(line)['manual'],42)

if __name__=='__main__':unittest.main()
