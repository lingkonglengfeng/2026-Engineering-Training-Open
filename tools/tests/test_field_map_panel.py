import sys, tempfile, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'vision/pc_vision'))
from field_map_panel import FieldMapPanel, MapState, STATIONS, defaults, apply_command
from serial_monitor import SerialLog, command_packet


def rows(deltas=(0,)*8, source='RAM', limit=300):
    result = ['[FIELD MAP BEGIN] v=1 firmware=258 variant=PRELIM source=%s sequence=2 limit=%d field=2400 cell=50' % (source, limit)]
    for i, (key, _, xy, anchor) in enumerate(STATIONS):
        dx, dy = deltas[2*i:2*i+2]
        result.append('[FIELD MAP POINT] key=%s base_x=%d base_y=%d dx=%d dy=%d x=%d y=%d anchor_x=%d anchor_y=%d' %
                      (key, *xy, dx, dy, xy[0]+dx, xy[1]+dy, anchor[0]+dx, anchor[1]+dy))
    return result + ['[FIELD MAP END]']


class MapTests(unittest.TestCase):
    def test_manual_obstacle_snapshot_is_complete_and_bounded(self):
        data=rows();data[0]+=' manual_mode=1 manual_count=2'
        data[-1:-1]=['[FIELD MAP MANUAL] id=1 x=2125 y=1550 radius=50','[FIELD MAP MANUAL] id=2 x=1200 y=762 radius=50']
        state=MapState()
        for line in data:state.receive(line)
        self.assertEqual(state.data['manual'][2]['y'],762)
        good=state.data
        for bad in (data[:-2]+data[-1:], [s.replace('x=1200','x=2400') for s in data],
                    [s.replace('manual_count=2','manual_count=3') for s in data]):
            for line in bad:state.receive(line)
            self.assertIs(state.data,good)
        for line in rows():state.receive(line)
        self.assertEqual(state.data['manual'],{})
    def test_yellow_margin_metadata_and_legacy(self):
        data=rows();data[0]+=' yellow_margin_mm=10';state=MapState()
        for line in data:state.receive(line)
        self.assertEqual(state.data['meta']['yellow_margin_mm'],'10')
        old=state.data;bad=data.copy();bad[0]=bad[0].replace('yellow_margin_mm=10','yellow_margin_mm=-1')
        for line in bad:state.receive(line)
        self.assertIs(state.data,old)
        for line in rows():state.receive(line)
        self.assertNotIn('yellow_margin_mm',state.data['meta'])
    def test_shared_target_protocol_and_defaults(self):
        for p in defaults().values():self.assertEqual((p['x'],p['y']),(p['anchor_x'],p['anchor_y']))
        data=rows((1,2,3,4,5,6,7,8));data[0]+=' shared_target=1';state=MapState()
        for line in data:state.receive(line)
        self.assertIsNotNone(state.data)
        for p in state.data['points'].values():self.assertEqual((p['x'],p['y']),(p['anchor_x'],p['anchor_y']))
        old=state.data;bad=data.copy();bad[2]=bad[2].replace('anchor_x=1253','anchor_x=1254')
        for line in bad:state.receive(line)
        self.assertIs(state.data,old)
        bad[0]=bad[0].replace(' shared_target=1','')
        for line in bad:state.receive(line)
        self.assertEqual(state.data['points']['raw']['anchor_x'],1254)  # Preserve actual legacy reply.
    def test_complete_snapshot_and_corrupt_reply(self):
        state = MapState()
        for line in rows((10,-10,0,0,-5,4,1,2)): state.receive(line)
        self.assertEqual(state.data['points']['qr']['x'], 2135)
        self.assertEqual(state.data['points']['rough']['anchor_y'], 384)
        self.assertEqual(state.limit, 300)
        previous = state.data
        for line in rows((301,0,0,0,0,0,0,0)): state.receive(line)
        self.assertIs(state.data, previous)
        for line in rows()[:-2] + ['[FIELD MAP END]']: state.receive(line)
        self.assertIs(state.data, previous)

    def test_firmware_limit_is_honoured_and_capped(self):
        state = MapState()
        for line in rows(limit=10): state.receive(line)
        self.assertEqual(state.limit, 10)
        previous = state.data
        for line in rows((11,)*8, limit=10): state.receive(line)
        self.assertIs(state.data, previous)
        for line in rows(limit=301): state.receive(line)
        self.assertIs(state.data, previous)
        for line in rows(limit=300): state.receive(line)
        self.assertEqual(state.limit, 300)

    def test_coordinates_bounds_and_packet(self):
        points = defaults(); values = {k: {a: p[a] for a in ('x','y')} for k,p in points.items()}
        values['qr']['x'] += 10; values['temp']['y'] -= 10
        cmd = apply_command(values, points)
        self.assertEqual(cmd, 'fieldmap apply 10 0 0 0 0 0 0 -10')
        self.assertLessEqual(len(cmd), 63); self.assertTrue(command_packet(cmd)[1].endswith(b'\r\n'))
        values = {k: {a: p[a] for a in ('x','y')} for k,p in points.items()}
        values['rough']['x'] = points['rough']['base_x'] + 300
        values['rough']['y'] = points['rough']['base_y'] - 300
        self.assertEqual(apply_command(values, points), 'fieldmap apply 0 0 0 0 300 -300 0 0')
        self.assertLessEqual(len(apply_command(values, points)), 63)
        for key, axis, invalid in (('rough','x',points['rough']['base_x']+301),
                                   ('qr','x',points['qr']['base_x']+300),
                                   ('qr','x','nan'),('qr','x','2125.5')):
            values = {k: {a: p[a] for a in ('x','y')} for k,p in points.items()}
            values[key][axis] = invalid
            with self.assertRaises(ValueError): apply_command(values, points)

    def test_panel_ack_save_and_offline_preview(self):
        import tkinter as tk
        from tkinter import ttk
        root = tk.Tk(); root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                log = SerialLog(temp); log.connected = True; log.session = 'fake'
                log.arm.data = {'version': {'firmware':'258', 'variant':'PRELIM'}}
                parent = ttk.Frame(root); parent.pack(); panel = FieldMapPanel(parent, log)
                def poll(): panel.parent.after_cancel(panel._after); panel.poll()
                self.assertTrue(log.outbound.empty()); panel.references(); self.assertTrue(log.outbound.empty())
                panel.local_click(SimpleNamespace(x=200, y=120))
                self.assertEqual(panel.entries['qr']['x'].get(), str(round(2125+20/panel.local_zoom())))
                self.assertTrue(log.outbound.empty()); panel.read()
                self.assertEqual(log.outbound.get_nowait()[0], 'arm ui')
                log._record('RX','[ARM UI END]'); poll()
                self.assertEqual(log.outbound.get_nowait()[0], 'fieldmap status')
                for line in rows(): log._record('RX', line)
                poll(); self.assertIsNone(panel.actions.active)
                panel.entries['qr']['x'].set('2135'); panel.entries['rough']['y'].set('374')
                panel.apply(); self.assertEqual(log.outbound.get_nowait()[0], 'fieldmap apply 10 0 0 0 0 -6 0 0')
                self.assertTrue(log.outbound.empty())
                log._record('RX','[FIELD MAP APPLY] OK; RAM only, no motion'); poll()
                self.assertEqual(log.outbound.get_nowait()[0], 'fieldmap status')
                for line in rows((10,0,0,0,0,-6,0,0)): log._record('RX', line)
                poll(); self.assertEqual(panel.points['rough']['anchor_y'], 374)
                panel.save(); self.assertEqual(log.outbound.get_nowait()[0], 'fieldmap save')
                log._record('RX','[FIELD MAP SAVE] FAILED'); poll()
                self.assertIsNone(panel.actions.active); self.assertTrue(log.outbound.empty())
                self.assertIsNone(log.thread)
                log.connected = False
                with self.assertRaises(ValueError): panel.apply()
        finally: root.destroy()

    def test_old_firmware_and_new_session_do_not_send_map_writes(self):
        import tkinter as tk
        from tkinter import ttk
        root = tk.Tk(); root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as temp:
                log = SerialLog(temp); log.connected = True; log.session = 'fake'
                parent = ttk.Frame(root); panel = FieldMapPanel(parent, log)
                panel.read(); self.assertEqual(log.outbound.get_nowait()[0], 'arm ui')
                log.arm.data = {'version': {'firmware':'257', 'variant':'PRELIM'}}
                log._record('RX','[ARM UI END]'); panel.parent.after_cancel(panel._after); panel.poll()
                self.assertIsNone(panel.actions.active); self.assertTrue(log.outbound.empty())
                self.assertIn('V258', panel.actions.message)
                log.session = 'reconnected'
                with self.assertRaises(ValueError): panel.save()
                self.assertTrue(log.outbound.empty())
        finally: root.destroy()


if __name__ == '__main__': unittest.main()
