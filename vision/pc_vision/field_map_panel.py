"""Surveyed car-centre station adjustments; the existing SerialLog owns UART."""
import math
import tkinter as tk
from tkinter import ttk, messagebox
from arm_control import ArmActions, fields
from obstacle_map import draw_obstacles
from match_timing import STAGES, seconds, draw_stage_times

STATIONS = (('qr', '二维码', (2125, 1130), (2125, 1130)),
            ('raw', '圆盘/原料', (1250, 2100), (1250, 2100)),
            ('rough', 'ROUGH', (1200, 380), (1200, 380)),
            ('temp', 'TEMP', (350, 1200), (350, 1200)))
LIMIT = 300  # Maximum per-axis adjustment range supported by this panel, millimetres.
FIELD = 2400
BLOCKS = ((550, 550, 1000, 1000), (1400, 550, 1850, 1000),
          (550, 1400, 1000, 1850), (1400, 1400, 1850, 1850))
PADS = ((1000, 2315, 1400, 2400), (1000, 0, 1400, 150),
        (0, 1050, 150, 1350), (2350, 1100, 2400, 1300))


def defaults():
    return {key: dict(base_x=xy[0], base_y=xy[1], x=xy[0], y=xy[1], dx=0, dy=0,
                      anchor_x=anchor[0], anchor_y=anchor[1])
            for key, _, xy, anchor in STATIONS}


def integer(value):
    try:
        number = float(value)
        if not math.isfinite(number) or not number.is_integer(): raise ValueError()
        return int(number)
    except (ValueError, TypeError, OverflowError):
        raise ValueError('坐标使用整数毫米') from None


def offsets(values, points, limit=LIMIT):
    result = []
    for key, label, _, _ in STATIONS:
        for axis in ('x', 'y'):
            value = integer(values[key][axis])
            delta = value - points[key]['base_' + axis]
            if not -limit <= delta <= limit:
                raise ValueError('%s的%s坐标必须在基准±%dmm内' % (label, axis.upper(), limit))
            if not 0 <= value <= FIELD:
                raise ValueError('%s的%s坐标超出场地0～%dmm' % (label, axis.upper(), FIELD))
            result.append(delta)
    return result


def apply_command(values, points, limit=LIMIT):
    return 'fieldmap apply ' + ' '.join(map(str, offsets(values, points, limit)))


class MapState:
    """Firmware owns the limit; the panel never accepts more than LIMIT."""
    def __init__(self): self.data = self.pending = None; self.limit = LIMIT; self.pending_limit = LIMIT

    def receive(self, line):
        if line.startswith('[FIELD MAP BEGIN]'):
            meta = fields(line)
            try:
                limit = int(meta.get('limit', ''))
                margin = integer(meta.get('yellow_margin_mm', 0))
                if not 0 <= margin <= FIELD: limit = 0
            except (TypeError, ValueError): limit = 0
            try:
                mode=integer(meta.get('manual_mode',0));count=integer(meta.get('manual_count',0))
                if mode not in (0,1) or not 0<=count<=2 or (not mode and count):limit=0
            except ValueError:limit=0
            self.pending = dict(meta=meta, points={}, manual={}) if 1 <= limit <= LIMIT else None
            self.pending_limit = limit
        elif line.startswith('[FIELD MAP POINT]') and self.pending is not None:
            try:
                row = fields(line)
                key = row.pop('key')
                row = {k: integer(row[k]) for k in ('base_x', 'base_y', 'x', 'y', 'dx', 'dy', 'anchor_x', 'anchor_y')}
                if key not in defaults(): raise ValueError()
                for axis in ('x', 'y'):
                    if row[axis] - row['base_' + axis] != row['d' + axis]: raise ValueError()
                    if abs(row['d' + axis]) > self.pending_limit or not 0 <= row[axis] <= FIELD: raise ValueError()
                    if not 0 <= row['anchor_' + axis] <= FIELD: raise ValueError()
                    if self.pending['meta'].get('shared_target')=='1' and row['anchor_'+axis]!=row[axis]:raise ValueError()
                self.pending['points'][key] = row
            except (ValueError, KeyError): self.pending = None
        elif line.startswith('[FIELD MAP MANUAL]') and self.pending is not None:
            try:
                row={k:integer(v) for k,v in fields(line).items()}
                if row['id'] not in (1,2) or not (0<=row['x']<FIELD and 0<=row['y']<FIELD) or row['radius']!=50:raise ValueError()
                if row['id'] in self.pending['manual']:raise ValueError()
                self.pending['manual'][row['id']]=row
            except (KeyError,ValueError):self.pending=None
        elif line == '[FIELD MAP END]':
            if self.pending is not None:
                meta = self.pending['meta']
                if (set(self.pending['points']) == set(defaults()) and meta.get('v') == '1'
                        and meta.get('variant') == 'PRELIM' and meta.get('field') == '2400'
                        and set(self.pending['manual'])==set(range(1,integer(meta.get('manual_count',0))+1))):
                    self.data = self.pending; self.limit = self.pending_limit
            self.pending = None


class FieldMapPanel:
    def __init__(self, parent, logger):
        self.parent, self.logger = parent, logger
        self.state = MapState(); self.actions = ArmActions(logger.send)
        self.session = logger.session; self.seen = logger.arm_snapshot()['sequence']
        self.closed = False; self._after = None; self.last_data = None; self.fill = False
        self.points = defaults(); self.selected = tk.StringVar(value='qr'); self.zoom = tk.DoubleVar(value=1)
        self.limit = LIMIT; self.title = tk.StringVar(value=self.title_text())
        self.obstacle_data=dict(rows=[],car=None,phase=0,revision=-1)
        self.yellow_margin=0  # Read the controller's actual planning margin.
        self.qr_rebase=False
        self.manual_mode=False;self.manual_rows=[];self.placing=tk.BooleanVar(value=False)
        self.manual_status=tk.StringVar(value='手动障碍：先读取主控地图；RAM保存，准备保留，断电清除。')
        self.timing_data={}; self.timing_signature=None
        self.timing_status=tk.StringVar(value='环节计时：等待发车；本段包含行驶、纠偏和取放，累计从发车开始')
        self.obstacle_status=tk.StringVar(value='视觉障碍：等待主控回报；橙色已确认，红色已落图，灰十字原始估计，青色小车')
        self.status = tk.StringVar(value='初赛V258起支持；当前显示代码基准图，尚未读取主控。')
        self.message = tk.StringVar(value=self.range_text())
        self.entries = {k: {a: tk.StringVar(value=str(v[a])) for a in ('x', 'y')} for k, v in self.points.items()}
        ttk.Label(parent, textvariable=self.title, font=('Microsoft YaHei UI', 14, 'bold'), padding=8).pack(anchor='w')
        ttk.Label(parent, textvariable=self.status, padding=(8, 2)).pack(anchor='w')
        timing = ttk.LabelFrame(parent, text='本轮环节用时（本段 / 完成累计）', padding=5); timing.pack(fill='x', padx=8)
        ttk.Label(timing, textvariable=self.timing_status).grid(row=0,column=0,columnspan=3,sticky='w')
        self.timing_labels={}
        for i,(key,label,_) in enumerate(STAGES):
            var=tk.StringVar(value=label+'：— / —'); self.timing_labels[key]=var
            ttk.Label(timing,textvariable=var).grid(row=1+i//3,column=i%3,sticky='w',padx=8,pady=1)
        for i in range(3):timing.columnconfigure(i,weight=1)
        row = ttk.Frame(parent, padding=8); row.pack(fill='x')
        for label, call in (('读取主控地图', self.read), ('应用工位坐标到RAM', self.apply),
                            ('保存工位坐标到W25', self.save), ('从W25读回工位', self.load),
                            ('基准值填入预览', self.references)):
            ttk.Button(row, text=label, command=lambda c=call: self.perform(c)).pack(side='left', padx=3)
        body = ttk.Frame(parent, padding=8); body.pack(fill='both', expand=True)
        manual_bar=ttk.Frame(parent,padding=(8,0));manual_bar.pack(fill='x',before=body)
        self.manual_widgets=[]
        place=ttk.Checkbutton(manual_bar,text='放置障碍物（点地图）',variable=self.placing,command=lambda:self.perform(self.begin_manual))
        place.pack(side='left',padx=3);self.manual_widgets.append(place)
        for title,call in (('删除最早一个',self.remove_manual),('清除手动障碍',self.clear_manual),('恢复视觉避障',self.restore_visual)):
            button=ttk.Button(manual_bar,text=title,command=lambda c=call:self.perform(c));button.pack(side='left',padx=3);self.manual_widgets.append(button)
        ttk.Label(parent,textvariable=self.manual_status,padding=(8,2),wraplength=1100).pack(fill='x',before=body)
        for button in self.manual_widgets:button.configure(state='disabled')
        left = ttk.Frame(body); left.pack(side='left', fill='both', expand=True)
        right = ttk.Frame(body, width=440); right.pack(side='right', fill='y', padx=(10, 0))
        self.canvas = tk.Canvas(left, bg='#f8fafc', highlightthickness=0, width=560, height=560)
        self.canvas.pack(fill='both', expand=True); self.canvas.bind('<Configure>', lambda _: self.redraw())
        self.canvas.bind('<Button-1>', self.map_click)
        ttk.Label(left, text='空心：基准点　绿色：主控RAM　蓝色：输入预览　虚线：280mm车体').pack(anchor='w')
        ttk.Label(left,textvariable=self.obstacle_status,wraplength=560).pack(anchor='w')
        zoomrow = ttk.Frame(left); zoomrow.pack(fill='x')
        ttk.Label(zoomrow, text='围绕所选点放大').pack(side='left')
        ttk.Scale(zoomrow, from_=1, to=8, variable=self.zoom, command=lambda _: self.redraw()).pack(side='left', fill='x', expand=True)
        headers = ('工位', '基准X/Y', '预览X', '预览Y')
        for column, label in enumerate(headers): ttk.Label(right, text=label).grid(row=0, column=column, padx=5, pady=6)
        self.base_labels = {}
        for row_index, (key, name, _, _) in enumerate(STATIONS, 1):
            ttk.Radiobutton(right, text=name, variable=self.selected, value=key, command=self.redraw).grid(row=row_index, column=0, sticky='w')
            self.base_labels[key] = ttk.Label(right, text='%d / %d' % (self.points[key]['base_x'], self.points[key]['base_y']))
            self.base_labels[key].grid(row=row_index, column=1, padx=5)
            for column, axis in enumerate(('x', 'y'), 2):
                base = self.points[key]['base_' + axis]
                spin = ttk.Spinbox(right, from_=base-self.limit, to=base+self.limit, increment=1,
                                   textvariable=self.entries[key][axis], width=8)
                spin.grid(row=row_index, column=column, padx=3, pady=6)
                setattr(self, key + '_' + axis + '_spin', spin)
                self.entries[key][axis].trace_add('write', lambda *_: self.redraw())
        self.details = tk.StringVar()
        ttk.Label(right, textvariable=self.details, wraplength=410, padding=(2, 12)).grid(row=6, column=0, columnspan=4, sticky='w')
        ttk.Label(right, text='局部放大：点击设定所选点的微调位置（1mm步进）').grid(row=7, column=0, columnspan=4, sticky='w')
        self.local = tk.Canvas(right, width=360, height=240, bg='#f1f5f9', highlightthickness=0)
        self.local.grid(row=8, column=0, columnspan=4, pady=8); self.local.bind('<Button-1>', self.local_click)
        self.local.bind('<B1-Motion>', self.local_click)
        ttk.Label(right, text='圆盘/ROUGH/TEMP目标与视觉重置位置相同。\nV300起：到QR点并扫码成功后，车位重置到QR目标。\n先应用预览再保存W25；下次启动加载，编辑本身不发车。',
                  wraplength=410, padding=5).grid(row=9, column=0, columnspan=4, sticky='w')
        ttk.Label(parent, textvariable=self.message, padding=8, wraplength=1120).pack(anchor='w')
        parent.bind('<Destroy>', self.destroy); self._after = parent.after(80, self.poll); self.redraw()

    def title_text(self): return '工位地图与±%dmm微调' % self.limit

    def range_text(self): return '地图坐标为车中心，左下角原点，X向右、Y向上；每轴仅允许基准±%dmm。' % self.limit

    def local_zoom(self): return min(8.0, 110.0/self.limit)

    def values(self): return {k: {a: v.get() for a, v in row.items()} for k, row in self.entries.items()}

    def draft(self):
        values = self.values(); offsets(values, self.points, self.limit)
        return {k: (integer(v['x']), integer(v['y'])) for k, v in values.items()}

    def redraw(self):
        if not hasattr(self, 'local'): return
        try: draft = self.draft()
        except ValueError as exc: self.details.set(str(exc)); return
        key = self.selected.get(); point = self.points[key]; x, y = draft[key]
        dx, dy = x-point['base_x'], y-point['base_y']
        ax, ay = point['anchor_x'] + x-point['x'], point['anchor_y'] + y-point['y']
        label = ('扫码重置点' if self.qr_rebase else '扫码参考点') if key == 'qr' else '视觉重置点'
        self.details.set('所选点偏移：X%+d / Y%+d mm\n主控RAM目标点：(%d, %d)\n预览目标点：(%d, %d)；%s：(%d, %d)' % (dx, dy, point['x'], point['y'], x, y, label, ax, ay))
        c = self.canvas; c.delete('all'); w, h = max(c.winfo_width(), 400), max(c.winfo_height(), 400)
        scale = min(w-64, h-64)/FIELD*self.zoom.get()
        cx, cy = (FIELD/2, FIELD/2) if self.zoom.get() <= 1.01 else (point['base_x'], point['base_y'])
        self.transform = (scale, w/2-cx*scale, h/2+cy*scale)
        def xy(px, py): return self.transform[1]+px*scale, self.transform[2]-py*scale
        def rect(bounds, **style):
            x0, y0, x1, y1 = bounds; c.create_rectangle(*xy(x0, y1), *xy(x1, y0), **style)
        rect((0, 0, FIELD, FIELD), fill='#e2e8f0', outline='#475569', width=2)
        for grid in range(0, FIELD+1, 100):
            c.create_line(*xy(grid, 0), *xy(grid, FIELD), fill='#cbd5e1')
            c.create_line(*xy(0, grid), *xy(FIELD, grid), fill='#cbd5e1')
        for bounds in BLOCKS:
            rect(bounds, fill='#fde68a', outline='#d97706')
            if self.yellow_margin:
                x0,y0,x1,y1=bounds;m=self.yellow_margin
                rect((x0-m,y0-m,x1+m,y1+m),outline='#92400e',dash=(4,3),tags='yellow_margin')
        for bounds in PADS: rect(bounds, fill='#67e8f9', outline='#0891b2')
        for bounds, title in (((2100, 0, 2400, 300), '1号启停区'), ((2100, 2100, 2400, 2400), '2号启停区')):
            rect(bounds, outline='#8b5cf6', width=2)
            c.create_text(*xy((bounds[0]+bounds[2])/2, (bounds[1]+bounds[3])/2), text=title, fill='#6d28d9')
        rect((x-140, y-140, x+140, y+140), outline='#64748b', dash=(4, 3))
        for name, title, _, _ in STATIONS:
            p = self.points[name]
            for px, py, color, radius, fill in ((p['base_x'], p['base_y'], '#64748b', 7, ''),
                                               (p['x'], p['y'], '#16a34a', 4, '#16a34a'),
                                               (*draft[name], '#2563eb', 3, '#2563eb')):
                sx, sy = xy(px, py); c.create_oval(sx-radius, sy-radius, sx+radius, sy+radius, outline=color, fill=fill, width=2)
            c.create_text(*xy(*draft[name]), text=title+'  ', anchor='se', fill='#0f172a')
        draw_obstacles(c,xy,scale,self.obstacle_data)
        for item in self.manual_rows:
            x,y=xy(item['x'],item['y']);radius=item['radius']*scale
            c.create_oval(x-radius,y-radius,x+radius,y+radius,outline='#7c3aed',fill='#ede9fe',width=2,tags='manual_obstacle')
            c.create_text(x+8,y-10,text='手动%d (%d,%d)'%(item['id'],item['x'],item['y']),anchor='sw',fill='#6d28d9',tags='manual_obstacle')
        draw_stage_times(c,xy,self.timing_data,self.points)
        c.create_text(*xy(0, 0), text='(0,0) mm', anchor='sw', fill='#475569')
        local = self.local; local.delete('all'); ox, oy, zoom = 180, 120, self.local_zoom(); half = self.limit*zoom
        local.create_rectangle(ox-half, oy-half, ox+half, oy+half, outline='#94a3b8', dash=(3, 3))
        local.create_line(20, oy, 340, oy, fill='#cbd5e1'); local.create_line(ox, 10, ox, 230, fill='#cbd5e1')
        local.create_oval(ox-5, oy-5, ox+5, oy+5, outline='#64748b', width=2)
        local.create_line(ox, oy, ox+dx*zoom, oy-dy*zoom, fill='#2563eb', width=2)
        local.create_oval(ox+dx*zoom-4, oy-dy*zoom-4, ox+dx*zoom+4, oy-dy*zoom+4, fill='#2563eb', outline='')
        local.create_text(10, 10, anchor='nw', text='虚框：允许±%dmm；蓝点：预览' % self.limit)

    def set_preview(self, x, y):
        key = self.selected.get(); point = self.points[key]
        for axis, value in (('x', x), ('y', y)):
            base = point['base_' + axis]
            self.entries[key][axis].set(str(max(base-self.limit, min(base+self.limit, round(value)))))

    def map_click(self, event):
        scale, ox, oy = self.transform
        world = ((event.x-ox)/scale, (oy-event.y)/scale)
        if self.placing.get():self.perform(lambda:self.place_manual(*world));return
        key = min(self.points, key=lambda k: math.dist(world, (self.points[k]['base_x'], self.points[k]['base_y'])))
        self.selected.set(key); self.redraw()

    def local_click(self, event):
        p = self.points[self.selected.get()]; zoom = self.local_zoom()
        self.set_preview(p['base_x']+(event.x-180)/zoom, p['base_y']+(120-event.y)/zoom)

    def perform(self, callback):
        try: callback()
        except (ValueError, RuntimeError, KeyError) as exc: messagebox.showerror('工位地图', str(exc), parent=self.parent)

    def sync_session(self, snapshot):
        if self.session == self.logger.session: return
        self.session = self.logger.session; self.actions.cancel('串口会话已改变，请重新读取地图')
        self.state = MapState(); self.last_data = None; self.seen = snapshot['sequence']
        self.yellow_margin=0
        self.qr_rebase=False
        self.manual_mode=False;self.manual_rows=[];self.placing.set(False)
        self.manual_status.set('串口会话改变，请重新读取手动障碍。')
        self.status.set('串口会话已改变；当前预览尚未和主控同步。')

    def ready(self, loaded=True):
        self.sync_session(self.logger.arm_snapshot())
        if not self.logger.connected or self.logger.stop_event.is_set(): raise ValueError('请先接入主控串口')
        if self.actions.active: raise ValueError('请等待上一条指令回执')
        if loaded and self.state.data is None: raise ValueError('先读取当前主控地图，再修改或保存')

    def status_command(self):
        version = self.logger.arm_snapshot()['data'].get('version', {})
        if int(version.get('firmware', 0)) < 258 or version.get('variant') != 'PRELIM':
            raise ValueError('地图微调需要初赛V258及以后固件')
        return 'fieldmap status'

    def chain(self, steps):
        self.fill = True; self.state.data = None
        self.actions.start(steps, self.logger.arm_snapshot()['sequence'])

    def read(self):
        self.ready(False); self.chain([('arm ui', '[ARM UI END]'), (self.status_command, '[FIELD MAP END]')])

    def apply(self):
        self.ready(); command = apply_command(self.values(), self.points, self.limit)
        self.chain([(command, '[FIELD MAP APPLY] OK'), ('fieldmap status', '[FIELD MAP END]')])

    def save(self):
        self.ready(); self.chain([('fieldmap save', '[FIELD MAP SAVE] OK'), ('fieldmap status', '[FIELD MAP END]')])

    def load(self):
        self.ready(); self.chain([('fieldmap load', '[FIELD MAP LOAD] OK'), ('fieldmap status', '[FIELD MAP END]')])

    def references(self):
        for key in self.entries:
            for axis in ('x', 'y'): self.entries[key][axis].set(str(self.points[key]['base_' + axis]))
        self.actions.message = '基准值只填入预览，尚未应用到RAM或写入W25。'; self.message.set(self.actions.message)

    def manual_ready(self):
        self.ready()
        if int(self.state.data['meta'].get('firmware',0))<303:raise ValueError('手动障碍需要初赛V303及以上固件')

    def begin_manual(self):
        try:self.manual_ready()
        except (ValueError,RuntimeError):self.placing.set(False);raise
        self.message.set('在主地图点击放置，最多2个；第三个替换最早一个。应用后自动关闭视觉障碍检测，随后正常准备/发车。' if self.placing.get() else '已退出障碍点选；点击地图继续选择工位。')

    def set_manual(self,points):
        self.manual_ready()
        command='manualobs set '+str(len(points))+''.join(' %d %d'%(x,y) for x,y in points)
        self.chain([(command,'[MANUAL OBS] OK'),('fieldmap status','[FIELD MAP END]')])

    def place_manual(self,x,y):
        self.manual_ready();x,y=round(x),round(y)
        if not (0<=x<FIELD and 0<=y<FIELD):raise ValueError('请点击场地0～2399mm范围内')
        points=[(p['x'],p['y']) for p in self.manual_rows][-1:]+[(x,y)]
        self.set_manual(points)

    def remove_manual(self):
        self.manual_ready()
        if not self.manual_rows:raise ValueError('当前没有手动障碍')
        self.set_manual([(p['x'],p['y']) for p in self.manual_rows[1:]])

    def clear_manual(self):
        self.manual_ready()
        if self.manual_mode:self.set_manual([])

    def restore_visual(self):
        self.manual_ready();self.placing.set(False)
        self.chain([('manualobs vision','[MANUAL OBS] OK'),('fieldmap status','[FIELD MAP END]')])

    def poll(self):
        self._after = None
        if self.closed: return
        timing=self.logger.match_timing_snapshot()
        self.timing_status.set('%s；总用时%s；当前%s（中途接入缺失记录显示—）' %
            (timing['state'],seconds(timing['elapsed_ms']),seconds(timing['current_ms']) if timing['current'] else '—'))
        for key,label,_ in STAGES:
            row=timing['rows'].get(key)
            text=label+'：'+('%s / %s%s' % (seconds(row['duration_ms']),seconds(row['cumulative_ms']),
                '（PC估算）' if row['source']=='PC估算' else '') if row else
                seconds(timing['current_ms'])+('进行中 / —' if timing['state']=='运行中' else ' 未完成 / —')
                if timing['current']==key else '— / —')
            self.timing_labels[key].set(text)
        signature=(timing['revision'],timing['current'],tuple(timing['rows']))
        self.timing_data=timing
        if signature!=self.timing_signature:self.timing_signature=signature;self.redraw()
        obstacle_data=self.logger.obstacle_map_snapshot()
        if obstacle_data!=self.obstacle_data:
            self.obstacle_data=obstacle_data
            rows=obstacle_data['rows'];mapped=sum(bool(p.get('mapped')) for p in rows)
            self.obstacle_status.set('视觉障碍：%d/2，已落图%d；橙色已确认，红色已落图，灰十字原始估计，青色小车%s' %
                (len(rows),mapped,'；无可行路线，暂停中' if obstacle_data['phase']==2 else ''))
            self.redraw()
        snapshot = self.logger.arm_snapshot(); self.sync_session(snapshot); ended = False
        for seq, _, line in snapshot['events']:
            if seq > self.seen:
                self.state.receive(line); ended = ended or line == '[FIELD MAP END]'
        self.seen = snapshot['sequence']; self.actions.update(snapshot)
        if ended and self.state.data is None: self.actions.message = '主控地图回报不完整，请重新读取'
        if self.state.data is not None and self.state.data is not self.last_data:
            self.last_data = self.state.data; self.points = self.state.data['points']; self.limit = self.state.limit
            self.title.set(self.title_text()); self.message.set(self.range_text())
            for key, point in self.points.items():
                self.base_labels[key].configure(text='%d / %d' % (point['base_x'], point['base_y']))
                for axis in ('x', 'y'):
                    base = point['base_' + axis]
                    getattr(self, key+'_'+axis+'_spin').configure(from_=base-self.limit, to=base+self.limit)
                    self.entries[key][axis].set(str(point[axis]))
            meta = self.state.data['meta']
            self.yellow_margin=integer(meta.get('yellow_margin_mm',0))
            self.qr_rebase=meta.get('qr_rebase')=='1'
            self.manual_mode=meta.get('manual_mode')=='1'
            self.manual_rows=[self.state.data['manual'][i] for i in sorted(self.state.data['manual'])]
            self.manual_status.set(('手动模式：%d/2个障碍，视觉障碍检测已关闭；准备/复位保留，断电清除。'%len(self.manual_rows)) if self.manual_mode else '未使用手动障碍；点选应用后关闭视觉障碍，最多2个，第三个替换最早一个。')
            self.status.set('主控V%s / %s；当前地图%s；保存序号%s；每轴基准±%dmm' % (meta.get('firmware'), meta.get('variant'), meta.get('source'), meta.get('sequence'), self.limit))
            self.status.set(self.status.get()+('；目标/视觉重置点统一' if meta.get('shared_target')=='1' else '；旧固件未声明目标/重置统一'))
            self.status.set(self.status.get()+'；黄区规划外扩%dmm（棕色虚线）'%self.yellow_margin)
            self.status.set(self.status.get()+('；QR扫码成功重置到停车点' if self.qr_rebase else '；旧固件QR不重置坐标'))
            self.redraw()
        supported=self.state.data and int(self.state.data['meta'].get('firmware',0))>=303 and self.logger.connected and not self.actions.active
        for button in self.manual_widgets:button.configure(state='normal' if supported else 'disabled')
        self.message.set(self.actions.message or self.message.get())
        self._after = self.parent.after(80, self.poll)

    def destroy(self, event):
        if event.widget is not self.parent: return
        self.closed = True
        if self._after is not None: self.parent.after_cancel(self._after)
