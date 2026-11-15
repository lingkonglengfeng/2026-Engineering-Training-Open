"""Initial-race chassis tuning, using the existing serial owner only."""
import math
import time
import tkinter as tk
from tkinter import ttk, messagebox
from arm_control import ArmActions

# key, label, unit, fixed reference, minimum, maximum
FIELDS=(('spd','直线速度比例','×250RPM',.65,.01,1),
        ('arc','圆弧目标速度','mm/s',306.8352,10,1000),
        ('accel','轮速加速度','RPM/s',600,1,5000),
        ('decel','普通减速度','RPM/s',292,1,5000),
        ('turn_decel','转向轮速减速度','RPM/s',900,1,10000),
        ('straight','前进距离系数','倍',1.03,.5,1.5),
        ('wheel','场地轮速输出上限','RPM',500,100,1000),
        ('yaw','圆弧角速度上限','°/s',300,1,720),
        ('t1000','旧时间标定t1000','ms',1930,50,20000),
        ('hold_kp','直行航向Kp','omega/°',.03,0,.2),
        ('hold_cap','直行修正限幅','omega',.05,0,1),
        ('hold_dead','直行角度死区','°',.2,0,5),
        ('turn_band','转向衔接容差','±°',1,.1,15),
        ('turn_stable','容差内保持时间','ms',100,0,2000))


def number(key,text):
    field=next(f for f in FIELDS if f[0]==key)
    try:value=float(text)
    except (TypeError,ValueError):raise ValueError(field[1]+'必须是数值')
    if not math.isfinite(value) or not field[4]<=value<=field[5]:
        raise ValueError('%s范围为%s～%s %s'%(field[1],field[4],field[5],field[2]))
    return value


def fmt(value):return ('%.6f'%value).rstrip('0').rstrip('.') or '0'


def apply_command(values):
    return 'motion apply '+' '.join(fmt(number(key,values[key])) for key,*_ in FIELDS[:9])


class MotionState:
    def __init__(self):self.data=None;self.pending=None;self.switches=None
    def receive(self,line):
        if line.startswith('[NAV SWITCH] v=1 '):
            row=dict(t.split('=',1) for t in line.split() if '=' in t)
            try:
                if int(row['firmware'])>=271 and row['variant']=='PRELIM' and row['arc'] in ('0','1') and row['avoid'] in ('0','1'):
                    self.switches=row
            except (KeyError,ValueError):pass
        if line.startswith('[MOTION BEGIN]'):
            self.pending={'meta':dict(t.split('=',1) for t in line.split() if '=' in t),'values':{}}
        elif line.startswith('[MOTION VALUE]') and self.pending is not None:
            row=dict(t.split('=',1) for t in line.split() if '=' in t)
            try:self.pending['values'][row['key']]=number(row['key'],row['value'])
            except (KeyError,ValueError,StopIteration):self.pending=None
        elif line=='[MOTION END]':
            if self.pending is not None and {f[0] for f in (FIELDS if int(self.pending['meta'].get('firmware',0))>=272 else FIELDS[:9])}<=set(self.pending['values']):
                self.data=self.pending
            self.pending=None


class MotionPanel:
    def __init__(self,parent,logger):
        self.parent,self.logger=parent,logger;self.closed=False;self._after=None
        self.actions=ArmActions(logger.send);self.state=MotionState()
        self.seen=logger.arm_snapshot()['sequence'];self.session=logger.session;self.fill=False;self.last_data=None
        self.last_switches=None
        self.arc_enabled=tk.BooleanVar(value=True);self.avoid_enabled=tk.BooleanVar(value=True)
        self.switch_status=tk.StringVar(value='主控开关未读取；勾选仅为待应用值')
        self.entries={f[0]:tk.StringVar(value=fmt(f[3])) for f in FIELDS}
        self.current={f[0]:tk.StringVar(value='未读取') for f in FIELDS}
        self.status=tk.StringVar(value='初赛V250起支持；先接入串口并读取当前值。输入框初始为参考值，不会自动下发。')
        self.message=tk.StringVar(value='修改后先应用到RAM，再保存到W25；参考列始终保留。')
        self.actions.message=self.message.get()
        ttk.Label(parent,text='底盘参数与W25保存',font=('Microsoft YaHei UI',14,'bold'),padding=10).pack(anchor='w')
        ttk.Label(parent,textvariable=self.status,padding=8).pack(anchor='w')
        switches=ttk.LabelFrame(parent,text='运行功能开关（初赛V271起）',padding=8);switches.pack(fill='x',padx=12,pady=4)
        ttk.Checkbutton(switches,text='启用圆弧转弯',variable=self.arc_enabled).grid(row=0,column=0,padx=8)
        ttk.Checkbutton(switches,text='启用视觉避障',variable=self.avoid_enabled).grid(row=0,column=1,padx=8)
        ttk.Button(switches,text='读取开关',command=lambda:self.perform(self.read_switches)).grid(row=0,column=2,padx=8)
        ttk.Button(switches,text='应用两个开关',command=lambda:self.perform(self.apply_switches)).grid(row=0,column=3,padx=8)
        ttk.Label(switches,textvariable=self.switch_status).grid(row=1,column=0,columnspan=4,sticky='w',pady=4)
        ttk.Label(switches,text='空闲时应用；本次上电保持，准备/复位不重开。断电后默认均开启，开关不写W25。关闭避障会清临时障碍。').grid(row=2,column=0,columnspan=4,sticky='w')
        tabs=ttk.Notebook(parent);tabs.pack(fill='x',padx=10)
        pages=[ttk.Frame(tabs),ttk.Frame(tabs)]
        tabs.add(pages[0],text='速度 / 加减速');tabs.add(pages[1],text='直线纠偏 / 转向衔接（V272起）')
        for index,page in enumerate(pages):
            self.build_fields(page,FIELDS[:9] if index==0 else FIELDS[9:])
        row=ttk.Frame(parent,padding=10);row.pack(fill='x')
        for title,callback in [('读取当前值',self.read),('全部应用到RAM',self.apply_all),('保存当前RAM到W25',self.save),('从W25读回',self.load),('参考值填入输入框',self.references)]:
            ttk.Button(row,text=title,command=lambda c=callback:self.perform(c)).pack(side='left',padx=4)
        ttk.Label(parent,text='直行参考Kp0.03 / 限幅±0.05 / 死区±0.20°；转向参考±1°连续100ms。新五项需V272；应用后保存到W25，上电优先加载。',padding=8,wraplength=1000).pack(anchor='w')
        ttk.Label(parent,textvariable=self.message,padding=8,wraplength=1100).pack(anchor='w')
        parent.bind('<Destroy>',self.destroy);self._after=parent.after(80,self.poll)
    def build_fields(self,parent,fields):
        grid=ttk.Frame(parent,padding=8);grid.pack(fill='x')
        for col,title in enumerate(('参数','主控当前值','输入值','单位','参考值（固定）','操作')):
            ttk.Label(grid,text=title).grid(row=0,column=col,padx=8,pady=3)
        for row,(key,label,unit,ref,lo,hi) in enumerate(fields,1):
            ttk.Label(grid,text=label).grid(row=row,column=0,sticky='w',pady=3)
            ttk.Label(grid,textvariable=self.current[key],width=14).grid(row=row,column=1,padx=8)
            ttk.Entry(grid,textvariable=self.entries[key],width=14).grid(row=row,column=2,padx=8)
            ttk.Label(grid,text=unit,width=12).grid(row=row,column=3,padx=8)
            ttk.Label(grid,text=fmt(ref),width=14).grid(row=row,column=4,padx=8)
            ttk.Button(grid,text='应用此项',command=lambda k=key:self.perform(lambda:self.apply_one(k))).grid(row=row,column=5,padx=8)
    def perform(self,callback):
        try:callback()
        except (ValueError,RuntimeError,KeyError) as exc:messagebox.showerror('底盘参数',str(exc),parent=self.parent)
    def sync_session(self,snapshot):
        if self.session==self.logger.session:return
        self.session=self.logger.session;self.actions.cancel('串口会话已改变，请重新读取')
        self.state=MotionState();self.last_data=None;self.last_switches=None;self.seen=snapshot['sequence']
        self.switch_status.set('串口会话已改变，请重新读取开关')
        for variable in self.current.values():variable.set('未读取')
    def ready(self,loaded=True):
        self.sync_session(self.logger.arm_snapshot())
        if not self.logger.connected or self.logger.stop_event.is_set():raise ValueError('请先接入主控串口')
        if self.actions.active:raise ValueError('请等待上一条指令回执')
        if loaded and (not self.state.data or int(self.state.data['meta'].get('firmware',0))<250 or self.state.data['meta'].get('variant')!='PRELIM'):
            raise ValueError('请先读取当前底盘参数（初赛V250及以后）')
    def chain(self,steps):
        self.fill=True;self.state.data=None;self.status.set("等待主控完整回报；输入尚未代表已保存值")
        self.actions.start(steps,self.logger.arm_snapshot()["sequence"])
    def status_command(self):
        version=self.logger.arm_snapshot()['data'].get('version',{})
        if int(version.get('firmware',0))<250 or version.get('variant')!='PRELIM':
            raise ValueError('需要初赛V250及以后固件，请核对烧录版本')
        return 'motion status'
    def read(self):
        self.ready(False);self.chain([('arm ui','[ARM UI END]'),(self.status_command,'[MOTION END]')])
    def switch_status_command(self):
        version=self.logger.arm_snapshot()['data'].get('version',{})
        if int(version.get('firmware',0))<271 or version.get('variant')!='PRELIM':
            raise ValueError('运行功能开关需要初赛V271及以后固件')
        return 'navswitch status'
    def read_switches(self):
        self.ready(False)
        self.actions.start([('arm ui','[ARM UI END]'),(self.switch_status_command,'[NAV SWITCH]')],
                           self.logger.arm_snapshot()['sequence'],completed_message='主控运行开关已读回')
    def apply_switches(self):
        self.ready(False);self.switch_status_command()
        if self.state.switches is None:raise ValueError('请先读取主控当前开关')
        command='navswitch set %d %d'%(self.arc_enabled.get(),self.avoid_enabled.get())
        self.actions.start([(command,'[NAV SWITCH]')],self.logger.arm_snapshot()['sequence'],
                           completed_message='两个开关已应用到主控RAM；准备/复位保留，断电恢复默认')
    def apply_one(self,key):
        self.ready();value=number(key,self.entries[key].get())
        if key in {f[0] for f in FIELDS[9:]} and int(self.state.data['meta'].get('firmware',0))<272:
            raise ValueError('直线纠偏/转向衔接参数需要初赛V272及以后固件')
        self.chain([('motion set '+key+' '+fmt(value),'[MOTION SET] OK'),('motion status','[MOTION END]')])
    def apply_all(self):
        self.ready();command=apply_command({k:v.get() for k,v in self.entries.items()})
        steps=[(command,'[MOTION APPLY] OK')]
        if int(self.state.data['meta'].get('firmware',0))>=272:
            heading='motion heading '+' '.join(fmt(number(key,self.entries[key].get())) for key,*_ in FIELDS[9:])
            steps.append((heading,'[MOTION HEADING] OK'))
        self.chain(steps+[('motion status','[MOTION END]')])
    def save(self):
        self.ready();self.chain([('motion save','[MOTION SAVE] OK'),('motion status','[MOTION END]')])
    def load(self):
        self.ready();self.chain([('motion load','[MOTION LOAD] OK'),('motion status','[MOTION END]')])
    def references(self):
        for key,_,_,ref,*_ in FIELDS:self.entries[key].set(fmt(ref))
        self.message.set('参考值已填入输入框；尚未应用到主控，也未写W25。');self.actions.message=self.message.get()
    def poll(self):
        self._after=None
        if self.closed:return
        snapshot=self.logger.arm_snapshot()
        self.sync_session(snapshot)
        ended=False
        for seq,_,line in snapshot['events']:
            if seq>self.seen:
                self.state.receive(line);ended=ended or line=='[MOTION END]'
        self.seen=snapshot['sequence'];self.actions.update(snapshot)
        switches=self.state.switches
        if switches is not None and switches is not self.last_switches:
            self.last_switches=switches
            self.arc_enabled.set(switches['arc']=='1');self.avoid_enabled.set(switches['avoid']=='1')
            self.switch_status.set('主控实际：圆弧%s / 视觉避障%s；停车触发约%smm，接近最低%sRPM（RAM）'%(
                '开启' if switches['arc']=='1' else '关闭','开启' if switches['avoid']=='1' else '关闭',
                switches.get('trigger_mm','420'),switches.get('min_rpm','15')))
            if switches.get('stop_mode')=='RAMP':
                self.switch_status.set(self.switch_status.get()+'；按普通减速度斜坡停车，实测停稳后重规划')

        if ended and self.state.data is None:self.actions.message='主控回报不完整，请重新读取'
        data=self.state.data
        if data is not None and data is not self.last_data:
            self.last_data=data
            for key,value in data['values'].items():
                self.current[key].set(fmt(value))
                if self.fill:self.entries[key].set(fmt(value))
            self.fill=False
            meta=data['meta'];self.status.set('主控V%s / %s；来源%s；保存序号%s'%(meta.get('firmware'),meta.get('variant'),meta.get('source'),meta.get('sequence')))
        self.message.set(self.actions.message or self.message.get())
        self._after=self.parent.after(80,self.poll)
    def destroy(self,event):
        if event.widget is not self.parent:return
        self.closed=True
        if self._after is not None:self.parent.after_cancel(self._after)
