import sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/pc_vision'))
from obstacle_map import ObstacleMapState,draw_obstacles
from serial_monitor import SerialLog


ACK='[VOBS ACK] session=21 obstacle=1 site=P06 result=DEFERRED raw=2148.3,1874.4 plan=2125,1650'
MAP='[VOBS MAP] session=21 obstacle=1 site=P06 x=2125 y=1650 rect=1850,1600,2400,1700 source=PREPLAN'
STATE='[VOBS STATE] v=2 session=21 armed=1 x=1200 y=1400 yaw=90 phase=0'


class MapTests(unittest.TestCase):
    def test_confirm_map_and_repeated_update_preserves_barrier(self):
        s=ObstacleMapState();s.receive(STATE);s.receive(ACK)
        self.assertEqual(s.snapshot()['rows'][0]['plan'],(2125,1650))
        self.assertFalse(s.snapshot()['rows'][0]['mapped'])
        s.receive(MAP);s.receive(ACK.replace('DEFERRED','MERGED'))
        self.assertTrue(s.snapshot()['rows'][0]['mapped'])
        self.assertEqual(s.snapshot()['rows'][0]['rect'],(1850,1600,2400,1700))
        rev=s.revision;s.receive(ACK.replace('DEFERRED','MERGED'));self.assertEqual(s.revision,rev)

    def test_eviction_and_session_reset_no_ghost_markers(self):
        s=ObstacleMapState();s.receive(ACK);s.receive(ACK.replace('obstacle=1','obstacle=2'))
        self.assertEqual(len(s.rows),2)
        s.receive('[VOBS EVICT] session=21 obstacle=1 old_site=P06 new_site=P02')
        self.assertEqual(list(s.rows),[2])
        s.receive(ACK.replace('P06','P02').replace('2125,1650','750,2125'))
        self.assertEqual(s.rows[1]['plan'],(750,2125));self.assertFalse(s.rows[1]['mapped'])
        s.receive(STATE.replace('session=21','session=22'));self.assertFalse(s.rows)
        s.receive(ACK);self.assertFalse(s.rows)
        s.receive(ACK.replace('session=21','session=22'));s.receive(STATE.replace('session=21','session=22').replace('armed=1','armed=0'))
        self.assertFalse(s.rows)

    def test_bad_and_rejected_reports_do_not_change_map(self):
        s=ObstacleMapState();s.receive(ACK);before=s.snapshot()
        for bad in (ACK.replace('DEFERRED','INELIGIBLE'),ACK.replace('obstacle=1','obstacle=3'),
                    ACK.replace('2148.3','nan'),MAP.replace('2400','2500'),MAP+' radius=invalid',STATE.replace('yaw=90','yaw=nan')):
            s.receive(bad);self.assertEqual(before,s.snapshot())

    def test_serial_collects_map_even_before_panel_open(self):
        with tempfile.TemporaryDirectory() as folder:
            log=SerialLog(folder)
            log._record('RX',STATE);log._record('RX',ACK);log._record('RX',MAP)
            for _ in range(1100):log._record('RX','diagnostic')
            snapshot=log.obstacle_map_snapshot();self.assertEqual(len(snapshot['rows']),1)
            self.assertTrue(snapshot['rows'][0]['mapped']);self.assertEqual(snapshot['car'],(1200,1400,90))
            snapshot['rows'].clear();self.assertEqual(len(log.obstacle_map_snapshot()['rows']),1)
            self.assertTrue(log.outbound.empty())

    def test_draw_renders_actual_controller_rect_and_car(self):
        s=ObstacleMapState();s.receive(STATE);s.receive(ACK);s.receive(MAP)
        class Canvas:
            def __init__(self):self.calls=[]
            def __getattr__(self,name):return lambda *a,**k:self.calls.append((name,a,k))
        c=Canvas();draw_obstacles(c,lambda x,y:(x/4,(2400-y)/4),.25,s.snapshot())
        rect=next(p for p in c.calls if p[0]=='create_rectangle')
        self.assertEqual(rect[1],(462.5,175.,600.,200.))
        self.assertTrue(any(p[0]=='create_polygon' and p[2]['tags']=='live_car' for p in c.calls))

    def test_real_tk_panel_draws_saved_obstacles_and_removes_evicted_slot(self):
        import tkinter as tk
        from tkinter import ttk
        from field_map_panel import FieldMapPanel
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as folder:
                log=SerialLog(folder)
                for line in (STATE,ACK,MAP):log._record('RX',line)
                frame=ttk.Frame(root);frame.pack();panel=FieldMapPanel(frame,log)
                frame.after_cancel(panel._after);panel.poll()
                self.assertTrue(panel.canvas.find_withtag('live_obstacle'))
                self.assertTrue(panel.canvas.find_withtag('live_car'))
                self.assertIn('1/2',panel.obstacle_status.get())
                from test_field_map_panel import rows
                data=rows();data[0]+=' yellow_margin_mm=10 qr_rebase=1'
                for line in data:log._record('RX',line)
                frame.after_cancel(panel._after);panel.poll()
                self.assertEqual(panel.yellow_margin,10)
                self.assertEqual(len(panel.canvas.find_withtag('yellow_margin')),4)
                self.assertIn('10mm',panel.status.get())
                self.assertTrue(panel.qr_rebase)
                self.assertIn('扫码重置点',panel.details.get())
                self.assertIn('QR扫码成功重置',panel.status.get())
                for line in rows():log._record('RX',line)
                frame.after_cancel(panel._after);panel.poll()
                self.assertFalse(panel.qr_rebase)
                self.assertIn('扫码参考点',panel.details.get())
                log._record('RX','[VOBS EVICT] session=21 obstacle=1 old_site=P06 new_site=P02')
                frame.after_cancel(panel._after);panel.poll()
                self.assertFalse(panel.canvas.find_withtag('live_obstacle'))
        finally:root.destroy()


    def test_manual_map_clicks_replace_oldest_and_restore_vision(self):
        import tkinter as tk
        from tkinter import ttk
        from field_map_panel import FieldMapPanel
        from test_field_map_panel import rows
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as folder:
                log=SerialLog(folder);log.connected=True;log.session='fake'
                frame=ttk.Frame(root);panel=FieldMapPanel(frame,log)
                def snapshot(points,manual=1,firmware=303):
                    data=rows();data[0]=__import__('re').sub(r'firmware=\d+','firmware=%d'%firmware,data[0])
                    data[0]+=' manual_mode=%d manual_count=%d'%(manual,len(points))
                    data[-1:-1]=['[FIELD MAP MANUAL] id=%d x=%d y=%d radius=50'%(i+1,x,y) for i,(x,y) in enumerate(points)]
                    for line in data:log._record('RX',line)
                    frame.after_cancel(panel._after);panel.poll()
                def ack(points,manual=1):
                    log._record('RX','[MANUAL OBS] OK manual=%d count=%d avoid=%d'%(manual,len(points),not manual))
                    frame.after_cancel(panel._after);panel.poll()
                    self.assertEqual(log.outbound.get_nowait()[0],'fieldmap status');snapshot(points,manual)
                snapshot([],0);panel.placing.set(True);panel.begin_manual();self.assertTrue(log.outbound.empty())
                panel.place_manual(2125,1550);self.assertEqual(log.outbound.get_nowait()[0],'manualobs set 1 2125 1550');ack([(2125,1550)])
                self.assertEqual(len(panel.canvas.find_withtag('manual_obstacle')),2)
                panel.place_manual(1200,762);self.assertEqual(log.outbound.get_nowait()[0],'manualobs set 2 2125 1550 1200 762');ack([(2125,1550),(1200,762)])
                panel.place_manual(275,1700);self.assertEqual(log.outbound.get_nowait()[0],'manualobs set 2 1200 762 275 1700');ack([(1200,762),(275,1700)])
                panel.remove_manual();self.assertEqual(log.outbound.get_nowait()[0],'manualobs set 1 275 1700');ack([(275,1700)])
                self.assertIn('视觉障碍检测已关闭',panel.manual_status.get())
                self.assertTrue(log.manual_obstacle_mode())
                panel.restore_visual();self.assertEqual(log.outbound.get_nowait()[0],'manualobs vision');ack([],0)
                self.assertFalse(panel.manual_mode);self.assertFalse(panel.placing.get());self.assertFalse(panel.canvas.find_withtag('manual_obstacle'))
                self.assertFalse(log.manual_obstacle_mode())
                snapshot([],0,302);panel.placing.set(True)
                with self.assertRaises(ValueError):panel.begin_manual()
                self.assertFalse(panel.placing.get());self.assertTrue(log.outbound.empty())
        finally:root.destroy()

if __name__=='__main__':unittest.main()
