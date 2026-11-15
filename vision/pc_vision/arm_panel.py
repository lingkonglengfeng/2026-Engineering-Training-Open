"""Tk arm bench panel: previews never move hardware; release sends opt-in."""
import math
import time
import tkinter as tk
from tkinter import ttk, messagebox

from arm_control import ArmActions, axis_steps, degree_span, displacement, move_command, number, point_pose, pose_check


class ArmPanel:
    def __init__(self, parent, logger, connect_vision=None):
        self.parent, self.logger = parent, logger
        self.connect_vision = connect_vision
        self.actions = ArmActions(logger.send)
        self.data, self.points = {}, {}
        self.disc_points = {}
        self.disc_kind = tk.StringVar(value='等待物料')
        self.disc_offsets = {key: tk.DoubleVar(value=0) for key in ('5', '6', 'dm')}
        self.disc_text = tk.StringVar(value='读取当前主控姿态后微调；灰色原目标、绿色已应用、蓝色预览。')
        self.disc_color = tk.StringVar(value='1 红色')
        self.disc_slot = tk.StringVar(value='1')
        self.profile_choice = tk.StringVar(value='初赛参数（供初赛代码）')
        self.disc_loaded = None
        self.session = None
        self.last_poll = 0
        self.closed = False
        self._after = None
        self.status = tk.StringVar(value='先在“主控串口日志”接入串口，再读取主控状态（需 V196）。')
        self.feedback = tk.StringVar(value='实际反馈：未读取')
        self.guard_text = tk.StringVar(value='旋转限制：等待主控参数')
        self.delta_text = tk.StringVar(value='选择工位和圆环，读取主控目标后预览微调。')
        self.message = tk.StringVar(value='滑块默认只预览；勾选实时控制后，松开才发送单轴目标。')
        self.area = tk.StringVar(value='ROUGH')
        self.ring = tk.StringVar(value='1')
        self.live = tk.BooleanVar(value=False)
        self.occupied = tk.BooleanVar(value=False)
        self.angles = {key: tk.DoubleVar(value=0) for key in ('5', '6', 'dm')}
        self.offsets = {key: tk.DoubleVar(value=0) for key in ('5', '6', 'dm')}
        self.servos = {'grip': tk.DoubleVar(value=225), 'table': tk.DoubleVar(value=109)}
        self.presets = {'open': tk.DoubleVar(value=225), 'close': tk.DoubleVar(value=308)}
        self.slots = [tk.DoubleVar(value=value) for value in (19, 109, 199)]
        self.last_point = None
        self.presets_received = None
        self.axis_scales = {}
        self.axis_labels = {}

        bar = ttk.Frame(parent, padding=(12, 8)); bar.pack(fill='x')
        ttk.Label(bar, text='机械臂调试与姿态标定', font=('Microsoft YaHei UI', 14, 'bold')).pack(side='left')
        ttk.Button(bar, text='读取状态 / 圆环与圆盘目标', command=self.request_all).pack(side='right', padx=4)
        ttk.Button(bar, text='停止当前动作', command=self.stop).pack(side='right', padx=4)
        ttk.Label(parent, textvariable=self.status, padding=(12, 2)).pack(fill='x')
        ttk.Label(parent, textvariable=self.feedback, padding=(12, 2)).pack(fill='x')
        ttk.Label(parent, textvariable=self.guard_text, padding=(12, 2)).pack(fill='x')

        # The notebook keeps every control reachable on smaller laptop screens.
        tabs = ttk.Notebook(parent); tabs.pack(fill='both', expand=True, padx=12, pady=6)
        manual = ttk.Frame(tabs, padding=10); tabs.add(manual, text='三轴与舵机')
        targets = ttk.Frame(tabs, padding=10); tabs.add(targets, text='ROUGH / TEMP 三点微调')
        disc = ttk.Frame(tabs, padding=10); tabs.add(disc, text='圆盘 / 圆环取放')
        jobs = ttk.Frame(tabs, padding=12); tabs.add(jobs, text='一键动作 / 参数保存')
        self.tabs = tabs
        self.manual_canvas = tk.Canvas(manual, bg='#f7fafc', height=220, highlightthickness=0)
        self.manual_canvas.pack(fill='both', expand=True)
        self.manual_canvas.bind('<Configure>', lambda e: self.draw())
        self._build_manual(manual)
        self._build_targets(targets)
        self._build_disc(disc)
        self._build_jobs(jobs)
        ttk.Label(parent, textvariable=self.message, padding=(12, 6), wraplength=1100).pack(fill='x')
        for var in list(self.angles.values()) + list(self.offsets.values()) + list(self.disc_offsets.values()):
            var.trace_add('write', lambda *a: self.draw())
        parent.bind('<Destroy>', self._destroy, add='+')
        self._after = parent.after(120, self.poll)

    def button(self, parent, text, command, **pack):
        widget = ttk.Button(parent, text=text, command=lambda: self.perform(command))
        widget.pack(side='left', padx=3, pady=3, **pack)
        return widget

    def perform(self, callback):
        try:
            callback()
        except (ValueError, RuntimeError, KeyError, tk.TclError) as exc:
            self.message.set(str(exc))
            messagebox.showerror('机械臂控制', str(exc), parent=self.parent)

    def ready(self, idle=True):
        if not self.logger.connected or self.logger.stop_event.is_set():
            raise ValueError('先在“主控串口日志”接入主控串口')
        if self.session != self.logger.session:
            raise ValueError('串口会话已改变，请等待重新读取主控状态')
        if not self.data or time.perf_counter()-self.data.get('received', 0) > 3:
            raise ValueError('主控状态尚未读取或已过期，请读取主控状态（需 V196）')
        if idle and (self.actions.active or self.data['state'].get('busy') == '1'):
            raise ValueError('机械臂仍在执行动作，请等待主控完成')

    def send(self, command):
        self.ready()
        self.logger.send(command)
        self.message.set('已入发送队列：'+command+'；实际结果见主控回执')

    def request_all(self):
        try:
            if self.actions.active:
                self.message.set('当前指令正在握手/执行，请完成后再手动刷新')
                return
            self.logger.send('dm query')
            self.logger.send('arm ui')
            self.logger.send('arm points')
            if self.data and int(self.data['version'].get('firmware', 0)) >= 238:
                self.logger.send('arm disc points')
            self.last_poll = time.perf_counter()
        except (ValueError, RuntimeError) as exc:
            self.message.set(str(exc))

    def _build_manual(self, frame):
        controls = ttk.Frame(frame); controls.pack(fill='x', pady=5)
        for key, title, low, high in (('5', '升降电机角度', -1206, 0), ('6', '伸缩电机角度', -729, 0), ('dm', '旋转关节角度', -180, 185)):
            row = ttk.Frame(controls); row.pack(fill='x', pady=3)
            ttk.Label(row, text=title, width=17).pack(side='left')
            scale = ttk.Scale(row, from_=low, to=high, variable=self.angles[key])
            scale.pack(side='left', fill='x', expand=True, padx=8)
            scale.bind('<ButtonRelease-1>', lambda e, axis=key: self._release(axis))
            scale.bind('<KeyRelease>', lambda e, axis=key: self._release(axis))
            self.axis_scales[key] = scale
            ttk.Spinbox(row, from_=low, to=high, increment=.1, textvariable=self.angles[key], width=10).pack(side='left')
            ttk.Label(row, text='°').pack(side='left')
            self.button(row, '发送此轴', lambda axis=key: self.send_axis(axis))
            label = ttk.Label(row, text=''); label.pack(side='left', padx=5); self.axis_labels[key] = label
        row = ttk.Frame(frame); row.pack(fill='x')
        ttk.Checkbutton(row, text='松开滑块发送单轴目标', variable=self.live).pack(side='left', padx=4)
        self.button(row, '用实际角度填充滑块', self.use_current)
        self.button(row, '按三轴目标移动（自动先抬升）', self.move)
        for axis, label in (('5', '升降'), ('6', '伸缩'), ('dm', '旋转')):
            self.button(row, label+'使能', lambda a=axis: self.send('arm enable '+a))
        servo_frame = ttk.LabelFrame(frame, text='舵机独立控制（PWM 角度，无位置传感器反馈）', padding=6)
        servo_frame.pack(fill='x', pady=8)
        for key, label in (('grip', '夹爪'), ('table', '载物盘')):
            row = ttk.Frame(servo_frame); row.pack(fill='x', pady=2)
            ttk.Label(row, text=label, width=10).pack(side='left')
            slider = ttk.Scale(row, from_=0, to=360, variable=self.servos[key]); slider.pack(side='left', fill='x', expand=True, padx=8)
            slider.bind('<ButtonRelease-1>', lambda e, s=key: self._release_servo(s))
            ttk.Spinbox(row, from_=0, to=360, increment=.5, textvariable=self.servos[key], width=10).pack(side='left')
            self.button(row, '发送角度', lambda s=key: self.send_servo(s))
        row = ttk.Frame(servo_frame); row.pack(fill='x')
        self.button(row, '夹爪张开（已设预置）', lambda: self.send('arm grip open'))
        self.button(row, '夹爪闭合（已设预置）', lambda: self.send('arm grip close'))
        self.button(row, '使能夹爪', lambda: self.send('arm grip enable'))
        self.button(row, '使能载物盘', lambda: self.send('arm table enable'))

    def _build_targets(self, frame):
        row = ttk.Frame(frame); row.pack(fill='x')
        ttk.Label(row, text='工位').pack(side='left')
        for variable, values, width in ((self.area, ('ROUGH', 'TEMP'), 12), (self.ring, ('1', '2', '3'), 6)):
            menu = ttk.Combobox(row, textvariable=variable, values=values, width=width, state='readonly')
            menu.pack(side='left', padx=6); menu.bind('<<ComboboxSelected>>', lambda e: self.select_point())
        ttk.Label(row, text='地面圆环').pack(side='left')
        self.button(row, '读取所选点已应用微调', self.select_point)
        self.target_canvas = tk.Canvas(frame, bg='#f7fafc', height=280, highlightthickness=0)
        self.target_canvas.pack(fill='both', expand=True, pady=8)
        self.target_canvas.bind('<Configure>', lambda e: self.draw())
        for axis, label, low, high in (('5', '升降角度增量', -80, 80), ('6', '伸缩角度增量', -80, 80), ('dm', '旋转角度增量', -12, 12)):
            row = ttk.Frame(frame); row.pack(fill='x', pady=3)
            ttk.Label(row, text=label, width=17).pack(side='left')
            ttk.Scale(row, from_=low, to=high, variable=self.offsets[axis]).pack(side='left', fill='x', expand=True, padx=8)
            ttk.Spinbox(row, from_=low, to=high, increment=.1, textvariable=self.offsets[axis], width=11).pack(side='left')
            ttk.Label(row, text='°').pack(side='left')
        ttk.Label(frame, textvariable=self.delta_text, wraplength=1080).pack(fill='x', pady=6)
        row = ttk.Frame(frame); row.pack(fill='x')
        self.button(row, '微调清零（仅预览）', lambda: [v.set(0) for v in self.offsets.values()])
        self.button(row, '应用所选点微调到主控 RAM', self.apply_trim)
        self.button(row, '移动到预览点（保持夹爪）', self.move_point)
        self.button(row, '应用微调并放置此圆环', lambda: self.drop(False))
        self.button(row, '夹取后放置此圆环', lambda: self.drop(True))

    def _build_disc(self, frame):
        row = ttk.Frame(frame); row.pack(fill='x')
        self.button(row, '圆盘纠偏 → RAM等待姿态', self.align_plate)
        self.button(row, '读取姿态参数', self.read_disc)
        ttk.Label(row, text='微调姿态').pack(side='left', padx=(12, 2))
        menu = ttk.Combobox(row, textvariable=self.disc_kind, values=('等待物料', '圆盘夹取物料', '放回载物盘', '载物盘取料（圆环放置前）'), state='readonly', width=24)
        menu.pack(side='left'); menu.bind('<<ComboboxSelected>>', lambda event: self.perform(self.select_disc))
        self.disc_canvas = tk.Canvas(frame, bg='#f7fafc', height=215, highlightthickness=0)
        self.disc_canvas.pack(fill='both', expand=True, pady=6)
        self.disc_canvas.bind('<Configure>', lambda event: self.draw_disc())
        for axis, label, low, high in (('5', '升降角度增量', -300, 300), ('6', '伸缩角度增量', -300, 300), ('dm', '旋转角度增量', -180, 180)):
            row = ttk.Frame(frame); row.pack(fill='x', pady=2)
            ttk.Label(row, text=label, width=16).pack(side='left')
            ttk.Scale(row, from_=low, to=high, variable=self.disc_offsets[axis]).pack(side='left', fill='x', expand=True, padx=8)
            ttk.Spinbox(row, from_=low, to=high, increment=.1, textvariable=self.disc_offsets[axis], width=12).pack(side='left')
            ttk.Label(row, text='°').pack(side='left')
        ttk.Label(frame, textvariable=self.disc_text, wraplength=1030).pack(fill='x', pady=5)
        row = ttk.Frame(frame); row.pack(fill='x')
        self.button(row, '读取所选姿态已应用微调', self.select_disc)
        self.button(row, '用当前三轴角度作为目标', self.disc_from_current)
        self.button(row, '应用所选微调到RAM', self.apply_disc)
        self.button(row, '移动到预览姿态（保持夹爪）', self.move_disc)
        row = ttk.Frame(frame); row.pack(fill='x')
        for key, label in (('wait', '到等待物料姿态'), ('grab', '到圆盘夹取姿态'), ('tray', '到放回载物盘姿态'), ('traypick', '到载物盘取料姿态')):
            self.button(row, label, lambda k=key: self.move_disc_stored(k))
        self.button(row, '从载物盘夹取（闭爪）', lambda: self.chain([('arm pick', '[ARM RESULT]')]))
        ttk.Label(frame, text='“到…”只移动三轴、保持夹爪；“从载物盘夹取”会按取料参数移动并闭爪。预览值须先应用到RAM。', wraplength=1030).pack(anchor='w')
        row = ttk.Frame(frame); row.pack(fill='x')
        ttk.Label(row, text='单次抓取颜色').pack(side='left')
        ttk.Combobox(row, textvariable=self.disc_color, values=('1 红色', '2 黄色', '3 蓝色', '4 绿色', '5 浅蓝', '6 黑色'), width=10, state='readonly').pack(side='left', padx=4)
        ttk.Label(row, text='载物盘槽').pack(side='left')
        ttk.Combobox(row, textvariable=self.disc_slot, values=('1', '2', '3'), width=4, state='readonly').pack(side='left', padx=4)
        self.button(row, '等物料进框 → 夹取 → 放回载物盘', self.disc_cycle)
        self.button(row, '放回载物盘当前槽（含松爪）', lambda: self.chain([('arm place', '[ARM RESULT]')]))
        row = ttk.Frame(frame); row.pack(fill='x', pady=3)
        self.button(row, '保存姿态到W25Q64', lambda: self.chain([('arm disc params save', '[ARM DISC PARAMS] SAVED')]))
        self.button(row, '读取W25Q64姿态', self.load_disc)
        ttk.Label(frame, text='初赛/决赛自动流程均使用本版本已应用姿态；载物盘取料供圆环放置前夹取使用。保存后上电自动装载。', wraplength=1030).pack(anchor='w')

    def disc_key(self):
        return {'等待物料': 'wait', '圆盘夹取物料': 'grab', '放回载物盘': 'tray', '载物盘取料（圆环放置前）': 'traypick'}[self.disc_kind.get()]

    def read_disc(self):
        self.ready(False)
        if int(self.data['version'].get('firmware', 0)) < 238:
            raise ValueError('姿态微调需要V245初赛/决赛，或原V238/V239及以后决赛固件')
        self.disc_loaded = None
        self.chain([('arm disc points', '[ARM DISC POINTS] END')])

    def load_disc(self):
        self.disc_loaded = None
        self.chain([('arm disc params load', '[ARM DISC PARAMS] LOADED'), ('arm disc points', '[ARM DISC POINTS] END')])

    def select_disc(self):
        point = self.disc_points.get(self.disc_key())
        if point is None: raise ValueError('请先读取姿态参数；载物盘取料微调需要V245初赛或V239及以后决赛固件')
        self.disc_loaded = self.disc_key()
        for axis, field in (('5', 'dl'), ('6', 'dr'), ('dm', 'dd')): self.disc_offsets[axis].set(number(point, field))
        self.draw_disc()

    def disc_target(self):
        point = self.disc_points.get(self.disc_key())
        if point is None: raise ValueError('请先读取姿态参数；载物盘取料微调需要V245初赛或V239及以后决赛固件')
        return point_pose(point, [self.disc_offsets[a].get() for a in ('5', '6', 'dm')])

    def disc_from_current(self):
        self.ready(False)
        if any(self.data['axes'][a].get('valid') != '1' for a in ('5', '6')) or self.data['rotation'].get('valid') != '1':
            raise ValueError('三轴实际反馈尚未全部有效')
        base, _ = self.disc_target()
        actual = [number(self.data['axes'][a], 'angle') for a in ('5', '6')] + [number(self.data['rotation'], 'angle')]
        for i, axis in enumerate(('5', '6', 'dm')): self.disc_offsets[axis].set(actual[i]-base[i])

    def apply_disc(self):
        self.ready()
        _, target = self.disc_target(); pose_check(self.data, target, self.disc_key() in ('tray', 'traypick'))
        command = 'arm disc trim %s %s' % (self.disc_key(), ' '.join('%.3f' % self.disc_offsets[a].get() for a in ('5', '6', 'dm')))
        self.chain([(command, '[ARM DISC TRIM] HAL=0'), ('arm disc points', '[ARM DISC POINTS] END')])

    def move_disc(self):
        self.ready(); _, target = self.disc_target()
        self.chain([(move_command(self.data, target, self.disc_key() in ('tray', 'traypick')), '[ARM MOVE] READY')])

    def move_disc_stored(self, key):
        if key not in self.disc_points:
            raise ValueError('请先读取姿态参数；载物盘取料姿态需要V245初赛或V239及以后决赛固件')
        self.chain([('arm disc pose '+key, '[ARM MOVE] READY')])

    def align_plate(self):
        self.ready()
        if self.connect_vision: self.connect_vision()
        self.chain([('valign run plate', '[MANUAL ALIGN] DONE'), ('arm disc pose wait', '[ARM MOVE] READY')],
                   completed_message='动作完成：圆盘已纠偏，已到主控RAM等待物料姿态；未启动夹取')

    def disc_cycle(self):
        self.ready()
        if self.connect_vision: self.connect_vision()
        self.chain([('arm material once %s %s' % (self.disc_color.get().split()[0], self.disc_slot.get()), '[MAT] SLOT_DONE')])

    def draw_disc(self):
        c = self.disc_canvas; c.delete('all')
        w, h = max(500, c.winfo_width()), max(170, c.winfo_height())
        try:
            if not self.data: raise ValueError('先连接串口并读取主控状态')
            base, target = self.disc_target()
            _, applied = point_pose(self.disc_points[self.disc_key()], [number(self.disc_points[self.disc_key()], k) for k in ('dl', 'dr', 'dd')])
            center = (w*.28, h*.5); radius = min(w*.21, h*.36)
            guard = self.data['guard']
            if guard.get('enabled') == '1':
                box = (center[0]-radius, center[1]-radius, center[0]+radius, center[1]+radius)
                c.create_oval(*box, fill='#fee2e2', outline='#fca5a5')
                c.create_arc(*box, start=number(guard, 'min'), extent=number(guard, 'max')-number(guard, 'min'), fill='#dcfce7', outline='#86efac')
            for i, (pose, color, label) in enumerate(((base, '#94a3b8', '原目标'), (applied, '#15803d', '已应用'), (target, '#2563eb', '预览'))):
                self._polar(c, center, radius*(.25+.75*pose[1]/degree_span(self.data, '6')), pose[2], color)
                c.create_text(18, 18+i*20, text=label, fill=color, anchor='w')
            delta = [target[i]-base[i] for i in range(3)]
            down, extension = displacement(self.data, '5', delta[0]), displacement(self.data, '6', delta[1])
            x = w*.72
            y = 25+displacement(self.data, '5', base[0])/number(self.data['axes']['5'], 'travel')*(h-50)
            target_y = 25+displacement(self.data, '5', target[0])/number(self.data['axes']['5'], 'travel')*(h-50)
            c.create_line(x, 25, x, h-25, fill='#94a3b8', width=4)
            c.create_line(x-22, y, x+22, y, fill='#94a3b8', width=5)
            c.create_line(x, y, x, target_y, fill='#2563eb', width=3, arrow='last')
            c.create_line(x-20, target_y, x+20, target_y, fill='#2563eb', width=4)
            if guard.get('enabled') == '1' and guard.get('top') == '1':
                limit_y = 25-number(guard, 'threshold')/number(self.data['axes']['5'], 'travel')*(h-50)
                c.create_line(x-40, limit_y, x+25, limit_y, fill='#dc2626', dash=(5, 3))
            c.create_text(x+35, y, text='高度增量\n%+.2fmm' % down, anchor='w', fill='#2563eb')
            c.create_text(w/2, h-10, text='相对位置示意，非空间模型；蓝色预览不会自动发送', fill='#64748b')
            self.disc_text.set('相对原目标：下降%+.2fmm / 伸出%+.2fmm / DM%+.3f°；最终角度：5号%.3f° / 6号%.3f° / DM%.3f°' % (down, extension, delta[2], *target))
        except (ValueError, KeyError, tk.TclError) as exc:
            c.create_text(w/2, h/2, text=str(exc), fill='#64748b')

    def _build_jobs(self, frame):
        profiles = ttk.LabelFrame(frame, text='初赛 / 决赛W25参数：与对应代码配套', padding=8)
        profiles.pack(fill='x', pady=(0, 8))
        row = ttk.Frame(profiles); row.pack(fill='x')
        ttk.Combobox(row, textvariable=self.profile_choice, values=('初赛参数（供初赛代码）', '决赛参数（供决赛代码）'), state='readonly', width=26).pack(side='left', padx=4)
        self.button(row, '一键切换 / 准备W25参数', self.switch_profile)
        ttk.Label(profiles, text='选择当前版本：读取本版本W25参数；选择另一版本：保存本版本并准备/核对目标区，再刷对应固件。此按钮不切换运行代码、不移动机械臂。', wraplength=1030).pack(anchor='w')
        ttk.Label(profiles, text='V245初赛/决赛各自保存圆环、舵机及四姿态；首次切初赛仍先在决赛固件中准备初赛区。PID和导航参数共用。', wraplength=1030).pack(anchor='w')
        actions = ttk.LabelFrame(frame, text='单次动作：请先把小车摆到对应工位', padding=12); actions.pack(fill='x')
        row = ttk.Frame(actions); row.pack(fill='x')
        self.button(row, '从载物盘当前槽夹取', lambda: self.chain([('arm pick', '[ARM RESULT]')]))
        self.button(row, '放回载物盘（当前槽位）', lambda: self.chain([('arm place', '[ARM RESULT]')]))
        row = ttk.Frame(actions); row.pack(fill='x')
        self.button(row, 'CAMERA 观察姿态', lambda: self.send('arm camera'))
        self.button(row, '恢复发车姿态', lambda: self.send('arm start'))
        for area in ('rough', 'temp'):
            self.button(row, area.upper()+' 位置 + 角度纠偏', lambda a=area: self.align(a))
        ttk.Checkbutton(actions, text='纠偏识别带物料圆环（空圆环时取消）', variable=self.occupied).pack(anchor='w', pady=5)
        ttk.Label(actions, text='纠偏会自动展开 CAMERA、请求 CAM1 模式并执行底盘 XY 与角度修正；完成后结束本次纠偏。', wraplength=1030).pack(anchor='w')
        presets = ttk.LabelFrame(frame, text='可保存参数：改数值后先应用，最后写入 W25Q64', padding=12); presets.pack(fill='x', pady=12)
        row = ttk.Frame(presets); row.pack(fill='x')
        for key, label in (('open', '夹爪张开'), ('close', '夹爪闭合')):
            ttk.Label(row, text=label).pack(side='left', padx=4)
            ttk.Spinbox(row, from_=0, to=360, increment=.5, textvariable=self.presets[key], width=10).pack(side='left')
        self.button(row, '应用夹爪开闭预置（不运动）', self.apply_grip)
        row = ttk.Frame(presets); row.pack(fill='x')
        for i, variable in enumerate(self.slots, 1):
            ttk.Label(row, text='载物盘槽%d' % i).pack(side='left', padx=4)
            ttk.Spinbox(row, from_=0, to=360, increment=.5, textvariable=variable, width=9).pack(side='left')
        self.button(row, '应用三槽角度（不运动）', self.apply_slots)
        row = ttk.Frame(presets); row.pack(fill='x', pady=8)
        self.button(row, '保存主控参数到 W25Q64', lambda: self.chain([('arm params save', '[ARM PARAMS] SAVED')]))
        self.button(row, '从 W25Q64 读取', lambda: self.chain([('arm params load', '[ARM PARAMS] LOADED'), ('arm points', '[ARM POINTS] END'), ('arm ui', '[ARM UI END]')]))
        self.button(row, '读取主控参数预置', lambda: self.send('arm params status'))
        ttk.Label(presets, text='保存内容：六个圆环点的三轴微调、夹爪开闭预置、载物盘三槽角度。滑块预览值须先应用到主控；读取参数不会下发运动。', wraplength=1030).pack(anchor='w')

    def switch_profile(self):
        self.ready()
        if int(self.data['version'].get('firmware', 0)) < 243:
            raise ValueError('参数分区需V245初赛/决赛或V243及以后决赛固件')
        target = 'PRELIM' if self.profile_choice.get().startswith('初赛') else 'FINAL'
        active = self.data['version'].get('variant', 'FINAL')
        steps = [('arm profile '+target.lower(), '[ARM PROFILE] READY target='+target)]
        if active == target:
            self.last_point = self.disc_loaded = self.presets_received = None
            steps += [('arm points', '[ARM POINTS] END'), ('arm disc points', '[ARM DISC POINTS] END'), ('arm ui', '[ARM UI END]')]
            message = '动作完成：已读取当前版本独立区参数；未移动机械臂'
        else:
            message = '动作完成：已保存当前参数并准备目标区；当前仍运行%s代码，请刷对应%s固件' % (
                '初赛' if active == 'PRELIM' else '决赛', '初赛' if target == 'PRELIM' else '决赛')
        self.chain(steps, completed_message=message)

    def _release(self, axis):
        if self.live.get(): self.perform(lambda: self.send_axis(axis))

    def _release_servo(self, servo):
        if self.live.get(): self.perform(lambda: self.send_servo(servo))

    def send_axis(self, axis):
        self.ready()
        session = self.logger.session

        def current_state():
            if not self.logger.connected or self.logger.stop_event.is_set() or self.logger.session != session:
                raise ValueError('串口会话已改变，未发送目标')
            return self.logger.arm_snapshot()['data']

        steps = axis_steps(self.data, axis, self.angles[axis].get(), current_state)
        self.chain(steps, completed_message='指令已由主控接受；实际到位请观察角度反馈')

    def send_servo(self, servo):
        angle = self.servos[servo].get()
        if not math.isfinite(angle) or not 0 <= angle <= 360: raise ValueError('舵机角度应为 0～360°')
        self.send('arm %s test %.3f' % (servo, angle))

    def use_current(self):
        self.ready(False)
        if any(self.data['axes'][axis].get('valid') != '1' for axis in ('5', '6')) or self.data['rotation'].get('valid') != '1':
            raise ValueError('三轴实际反馈尚未全部有效')
        for axis in ('5', '6'): self.angles[axis].set(number(self.data['axes'][axis], 'angle'))
        self.angles['dm'].set(number(self.data['rotation'], 'angle'))

    def move(self):
        self.ready()
        self.chain([(move_command(self.data, [self.angles[a].get() for a in ('5', '6', 'dm')]), '[ARM MOVE] READY')])

    def point(self):
        key = (self.area.get().lower(), self.ring.get())
        if key not in self.points: raise ValueError('请先读取主控六点目标')
        return self.points[key]

    def select_point(self):
        try:
            point = self.point()
            for axis, field in (('5', 'dl'), ('6', 'dr'), ('dm', 'dd')): self.offsets[axis].set(number(point, field))
            self.last_point = (self.area.get(), self.ring.get())
            self.draw()
        except ValueError as exc: self.message.set(str(exc))

    def target(self):
        return point_pose(self.point(), [self.offsets[a].get() for a in ('5', '6', 'dm')])

    def trim_step(self):
        _, pose = self.target(); pose_check(self.data, pose)
        return ('arm trim %s %s %s' % (self.area.get().lower(), self.ring.get(), ' '.join('%.3f' % self.offsets[a].get() for a in ('5', '6', 'dm'))), '[ARM TRIM] HAL=0')

    def apply_trim(self):
        self.ready(); self.chain([self.trim_step(), ('arm points', '[ARM POINTS] END')])

    def move_point(self):
        self.ready(); _, pose = self.target()
        self.chain([(move_command(self.data, pose), '[ARM MOVE] READY')])

    def drop(self, pick):
        self.ready()
        steps = [self.trim_step()]
        if pick: steps.append(('arm pick', '[ARM RESULT]'))
        steps.append(('arm ring %s %s' % (self.area.get().lower(), self.ring.get()), '[ARM RESULT]'))
        self.chain(steps)

    def chain(self, steps, completed_message='动作完成（主控已回执）'):
        self.ready()
        self.actions.start(steps, self.logger.arm_snapshot()['sequence'], completed_message)

    def align(self, area):
        self.ready()
        if self.connect_vision: self.connect_vision()
        profile = 'occupied' if self.occupied.get() else 'fast'
        self.chain([('valign rings '+profile, '[ALIGN] ring profile='), ('valign run '+area, '[MANUAL ALIGN] DONE')])

    def apply_grip(self):
        values = [v.get() for v in self.presets.values()]
        if any(not math.isfinite(v) or not 0 <= v <= 360 for v in values): raise ValueError('夹爪预置角度应为 0～360°')
        self.chain([('arm grip preset %.3f %.3f' % tuple(values), '[ARM PRESET] HAL=0')])

    def apply_slots(self):
        values = [v.get() for v in self.slots]
        if any(not math.isfinite(v) or not 0 <= v <= 360 for v in values): raise ValueError('载物盘角度应为 0～360°')
        self.chain([('arm table presets '+' '.join('%.3f' % v for v in values), '[ARM PRESET] HAL=0')])

    def stop(self):
        self.actions.cancel('已取消后续步骤')
        # STOP is explicit, including when feedback is stale or a chain is busy.
        try: self.logger.send('&STOP'); self.message.set('已发送 STOP，后续步骤已取消')
        except (ValueError, RuntimeError) as exc: self.message.set(str(exc))

    def draw(self):
        self.draw_disc()
        if self.closed: return
        try:
            self._draw_manual()
            self._draw_targets()
        except (ValueError, KeyError, tk.TclError): pass  # Incomplete text entry remains editable.

    def _polar(self, canvas, center, radius, angle, color, tag='', outline=False):
        x, y = center[0]+radius*math.cos(math.radians(angle)), center[1]-radius*math.sin(math.radians(angle))
        canvas.create_line(*center, x, y, fill=color, width=2, dash=(4, 3) if outline else ())
        canvas.create_oval(x-6, y-6, x+6, y+6, outline=color, fill='' if outline else color, width=2)
        if tag: canvas.create_text(x, y-14, text=tag, fill=color)
        return x, y

    def _draw_manual(self):
        c = self.manual_canvas; c.delete('all'); w, h = max(c.winfo_width(), 600), max(c.winfo_height(), 180)
        center = (w*.27, h*.53); radius = min(h*.39, 130)
        c.create_oval(center[0]-radius, center[1]-radius, center[0]+radius, center[1]+radius, outline='#cbd5e1')
        c.create_text(center[0], 14, text='俯视：旋转角 / 伸缩比例示意', fill='#334155')
        if self.data:
            guard = self.data['guard']
            if guard.get('enabled') == '1':
                # Full forbidden area, then overlay the permitted sector.
                box = (center[0]-radius, center[1]-radius, center[0]+radius, center[1]+radius)
                c.create_oval(*box, fill='#fee2e2', outline='#fca5a5')
                c.create_arc(*box, start=number(guard, 'min'), extent=number(guard, 'max')-number(guard, 'min'), fill='#dcfce7', outline='#86efac')
            reach = self.angles['6'].get()/degree_span(self.data, '6')
            self._polar(c, center, radius*(.25+.75*reach), self.angles['dm'].get(), '#2563eb', '目标')
            if time.perf_counter()-self.data.get('received', 0) <= 3 and self.data['rotation'].get('valid') == '1' and self.data['axes']['6'].get('valid') == '1':
                ratio = number(self.data['axes']['6'], 'angle')/degree_span(self.data, '6')
                self._polar(c, center, radius*(.25+.75*ratio), number(self.data['rotation'], 'angle'), '#16a34a', '反馈')
            x, top, bottom = w*.67, 34, h-25
            c.create_line(x, top, x, bottom, fill='#94a3b8', width=8)
            ratio = self.angles['5'].get()/degree_span(self.data, '5'); y = top+ratio*(bottom-top)
            c.create_rectangle(x-22, y-5, x+22, y+5, fill='#2563eb', outline='')
            c.create_text(x+65, y, text='下降 %.1f mm' % displacement(self.data, '5', self.angles['5'].get()), fill='#2563eb')
            if guard.get('enabled') == '1' and guard.get('top') == '1':
                limit = -number(guard, 'threshold')/number(self.data['axes']['5'], 'travel')
                ylimit = top+limit*(bottom-top)
                c.create_line(x-40, ylimit, x+130, ylimit, dash=(5, 3), fill='#dc2626')
                c.create_text(x+70, ylimit+12, text='以下仅限绿色扇区', fill='#b91c1c')
            c.create_text(x, 14, text='侧视：升降行程', fill='#334155')
        c.create_text(w/2, h-8, text='红色：低位禁转区  ·  蓝色：预览目标  ·  绿色：有效反馈  ·  非实测空间模型', fill='#64748b')

    def _draw_targets(self):
        c = self.target_canvas; c.delete('all'); w, h = max(c.winfo_width(), 600), max(c.winfo_height(), 210)
        center = (w*.32, h*.68); scale = min(h*.47, 150)
        scope = 'ROUGH / TEMP 共用三点放置参数' if self.data.get('state', {}).get('shared_ring') == '1' else self.area.get()+' 三个圆环目标'
        c.create_text(w/2, 15, text=scope+' · 空心原目标 / 灰色已应用 / 蓝色当前预览', fill='#334155')
        if not self.data: c.create_text(w/2, h/2, text='读取主控六点目标后显示'); return
        selected_base = selected_pose = None
        for ring in ('1', '2', '3'):
            point = self.points.get((self.area.get().lower(), ring))
            if point is None: continue
            base, applied = point_pose(point)
            radius = scale*(.4+.6*base[1]/degree_span(self.data, '6'))
            self._polar(c, center, radius, base[2], '#94a3b8', '圆环'+ring, True)
            radius_applied = scale*(.4+.6*applied[1]/degree_span(self.data, '6'))
            self._polar(c, center, radius_applied, applied[2], '#64748b')
            if ring == self.ring.get():
                selected_base, selected_pose = self.target()
                radius_new = scale*(.4+.6*selected_pose[1]/degree_span(self.data, '6'))
                old = (center[0]+radius*math.cos(math.radians(base[2])), center[1]-radius*math.sin(math.radians(base[2])))
                new = self._polar(c, center, radius_new, selected_pose[2], '#2563eb', '微调预览')
                c.create_line(*old, *new, fill='#2563eb', arrow='last', width=3)
        if selected_base:
            delta = tuple(b-a for a, b in zip(selected_base, selected_pose))
            down = displacement(self.data, '5', delta[0]); out = displacement(self.data, '6', delta[1])
            text = '相对原目标：下降 %+.2f mm / 伸出 %+.2f mm / 旋转 %+.3f°\n目标电机角度：5号 %.3f° / 6号 %.3f° / DM %.3f°' % (down, out, delta[2], *selected_pose)
            self.delta_text.set(text)
            c.create_text(w*.76, h*.38, text='高度相对原目标\n%+.2f mm（向下为正）' % down, fill='#2563eb')
            try: pose_check(self.data, selected_pose); result, color = '目标符合行程与高度限制', '#15803d'
            except ValueError as exc: result, color = str(exc), '#b91c1c'
            c.create_text(w*.76, h*.65, text=result, fill=color, width=w*.33)
        c.create_text(w/2, h-12, text='图中半径按伸缩比例绘制；相对位移由主控行程标定换算。', fill='#64748b')

    def poll(self):
        self._after = None
        if self.closed: return
        snapshot = self.logger.arm_snapshot()
        connected = self.logger.connected and not self.logger.stop_event.is_set()
        if not connected or self.session != self.logger.session:
            self.actions.cancel('串口会话已改变；后续步骤取消')
            self.data, self.points = {}, {}
            self.disc_points = {}; self.disc_loaded = None
            self.session = self.logger.session
            self.last_point = None
            self.presets_received = None
        if connected:
            self.actions.update(snapshot)
            self.data, self.points = snapshot['data'], snapshot['points']
            self.disc_points = snapshot.get('disc_points', {})
            if self.data:
                fresh = time.perf_counter()-self.data['received'] <= 3
                presets = self.data.get('presets')
                if presets and presets != self.presets_received:
                    for key in ('open', 'close'): self.presets[key].set(number(presets, key))
                    for i, var in enumerate(self.slots, 1): var.set(number(presets, 'table'+str(i)))
                    self.presets_received = dict(presets)
                boot_source = self.data['state'].get('boot_params')
                boot_text = ' · 启动参数：'+{'W25Q64':'W25Q64 已保存参数','DEFAULT':'默认参数','PENDING':'读取中'}.get(boot_source, boot_source or '') if boot_source else ''
                if self.data['state'].get('param_bank') == 'FINAL': boot_text += ' · 参数区：决赛独立区'
                elif self.data['state'].get('param_bank') == 'PRELIM': boot_text += ' · 参数区：初赛独立区'
                self.status.set('主控 V%s · 状态年龄 %.1fs · %s%s' % (self.data['version'].get('firmware', '?'), time.perf_counter()-self.data['received'], '动作中' if self.data['state'].get('busy') == '1' else '空闲', boot_text))
                actual = []
                for axis in ('5', '6'):
                    a = self.data['axes'][axis]; span = degree_span(self.data, axis)
                    self.axis_scales[axis].configure(from_=min(0, span), to=max(0, span))
                    actual.append(axis+'号 '+('%.3f°' % number(a, 'angle') if fresh and a.get('valid') == '1' else '未知'))
                    self.axis_labels[axis].configure(text='%.1f mm' % displacement(self.data, axis, self.angles[axis].get()))
                rot, guard = self.data['rotation'], self.data['guard']
                self.axis_scales['dm'].configure(from_=number(rot, 'min'), to=number(rot, 'max'))
                dm_text = '%.3f°' % number(rot, 'angle') if fresh and rot.get('valid') == '1' else ('未使能 / 无有效角度' if rot.get('enabled') != '1' else '角度未知')
                actual.append('DM '+dm_text)
                for key, label in (('grip', '夹爪PWM'), ('table', '载物盘PWM')):
                    servo = self.data['servos'].get(key)
                    if servo: actual.append(label+' '+servo.get('pwm_angle', '?')+'°')
                self.feedback.set('实际反馈：'+' · '.join(actual))
                self.guard_text.set(('当前主控高度旋转联锁关闭' if guard.get('enabled') != '1' else '主控限制：下降超过 %.1f mm 时，仅允许 [%.1f°, %.1f°]；面板不修改联锁' % (-number(guard, 'threshold'), number(guard, 'min'), number(guard, 'max'))))
                if self.points and self.last_point is None: self.select_point()
                if self.disc_points and self.disc_loaded is None: self.select_disc()
            else: self.status.set('串口已连接，等待 V196 主控 UI 状态；V195 可继续使用串口文本指令。')
            # Query only when this top-level notebook tab is selected. Never
            # drain the log queue: the serial log tab still receives every line.
            notebook = self.parent.master
            visible = not isinstance(notebook, ttk.Notebook) or notebook.select() == str(self.parent)
            if visible and not self.actions.active and time.perf_counter()-self.last_poll >= 1.2:
                self.last_poll = time.perf_counter()
                try:
                    self.logger.send('arm ui')
                    if not self.points: self.logger.send('arm points')
                    if not self.disc_points and self.data and int(self.data['version'].get('firmware', 0)) >= 238:
                        self.logger.send('arm disc points')
                except RuntimeError: pass
            if self.actions.active: self.message.set(self.actions.message)
            elif self.actions.message.startswith(('动作完成', '指令已由主控接受', '发送失败', '主控未完成', '等待回执超时')): self.message.set(self.actions.message)
        else:
            self.status.set('串口未连接；预览不下发动作。请在“主控串口日志”接入串口。')
            self.feedback.set('实际反馈：未知（未连接）')
            self.guard_text.set('旋转限制：未连接，等待主控当前参数')
        self.draw()
        self._after = self.parent.after(120, self.poll)

    def _destroy(self, event):
        if event.widget is not self.parent: return
        self.closed = True
        self.actions.cancel()
        if self._after is not None: self.parent.after_cancel(self._after); self._after = None
