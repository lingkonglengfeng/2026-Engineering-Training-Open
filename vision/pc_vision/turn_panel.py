"""转向 PID 调参与波形面板。

本面板不建串口、不自动下发运动指令：所有读取与下发都由按钮/复选框点击触发，
串口仍由 SerialLog 独占。绘图只用 tkinter Canvas，不引入新依赖。
"""
import math
import time
import tkinter as tk
from tkinter import ttk, messagebox


DEFAULT_GAINS=dict(kp='0.045',ki='0.0005',kd='0.0035')
GAIN_KEYS=('kp','ki','kd')
TURN_TIMEOUT=8.0       # 固件单次转向的上限；超时恢复按钮，避免界面永久锁死
WAVE_SECONDS=10.0      # 波形滚动时间窗
DRAW_MIN_INTERVAL=.05  # 重绘节流：Canvas 最多每 50 ms 重画一次
SOURCE_TEXT={'DEFAULT':'默认值（W25 无有效记录）','W25Q64':'W25 已保存（上电自动装载）',
             'MANUAL':'RAM 手改（未保存，重启失效）'}


def source_text(value):
    """把主控回报的 PID 来源翻译成现场能看懂的一句话。"""
    return SOURCE_TEXT.get(value,value or '未知')


def gain_number(text,label):
    try:value=float(str(text).strip())
    except (TypeError,ValueError):raise ValueError('%s 必须是数值'%label)
    if not math.isfinite(value) or abs(value)>10000:
        raise ValueError('%s 必须是绝对值不超过 10000 的有限数值'%label)
    return value


def gain_text(value):
    # &kp 与数值之间不能有空格；用定点小数，避免固件解析到科学计数法。
    text=('%.6f'%value).rstrip('0').rstrip('.')
    return text or '0'


def turn_angle(text):
    try:value=int(str(text).strip())
    except (TypeError,ValueError):raise ValueError('自定义角度必须是 1～360 的整数')
    if not 1<=value<=360:raise ValueError('自定义角度必须是 1～360 的整数')
    return value


def signed(value,digits=2,suffix=''):
    return '—' if value is None else ('%+.'+str(digits)+'f'+suffix)%value


class TurnPanel:
    def __init__(self,parent,logger):
        self.parent,self.logger=parent,logger
        self.closed=False;self._after=None
        self.snapshot=None
        self.busy=False;self.busy_since=0.
        self.session=logger.session
        self.last_draw=0.
        # 收到过的事件时间戳：只对更新的一次做提示，避免每 60 ms 重复弹同一行。
        self.start_seen=self.done_seen=self.reject_seen=None
        self.trace_seen=self.pid_seen=self.pid_note_seen=None
        # 主控 RAM 里已知的增益；None=未知。用来拦住“改了输入框但没点应用就转向”。
        self.applied=None
        self.source=None
        self.turn_buttons=[]
        self.status=tk.StringVar(value='串口未连接：先在“主控串口日志”页接入串口。')
        self.message=tk.StringVar(value='所有读取与下发都需手动点击按钮；未连接时只报错、不发送。')
        self.trace_status=tk.StringVar(value='波形开关：未确认')
        self.gains={key:tk.StringVar(value=value) for key,value in DEFAULT_GAINS.items()}
        self.angle=tk.StringVar(value='90')
        # 默认勾选只是一个“意愿”状态：连上串口不会自动发 &ttrace 1；只有用户再次勾选时才发送。
        self.record=tk.BooleanVar(value=True)

        bar=ttk.Frame(parent,padding=(12,8));bar.pack(fill='x')
        ttk.Label(bar,text='转向 PID 调参与波形',font=('Microsoft YaHei UI',14,'bold')).pack(side='left')
        ttk.Label(parent,textvariable=self.status,padding=(12,2)).pack(fill='x')
        ttk.Label(parent,text='提示：先在“主控串口日志”页接入串口；固件需 V234 才支持 [TURN]/[PID] 遥测。',
                  padding=(12,2),wraplength=1100).pack(fill='x')

        params=ttk.LabelFrame(parent,text='转向 PID 参数（&kp/&ki/&kd 写入 RAM 立即生效；数值与 & 之间不能有空格）',padding=8)
        params.pack(fill='x',padx=12,pady=(4,0))
        row=ttk.Frame(params);row.pack(fill='x')
        for key,title in (('kp','Kp 比例'),('ki','Ki 积分'),('kd','Kd 微分')):
            ttk.Label(row,text=title,width=9).pack(side='left',padx=(10,2))
            ttk.Entry(row,textvariable=self.gains[key],width=12).pack(side='left')
        row=ttk.Frame(params);row.pack(fill='x',pady=(6,0))
        ttk.Button(row,text='读取当前(&pid)',command=self.request_pid).pack(side='left',padx=3)
        ttk.Button(row,text='应用到RAM(&kp/&ki/&kd)',command=self.apply_gains).pack(side='left',padx=3)
        ttk.Button(row,text='保存到W25(&save)',command=self.save_gains).pack(side='left',padx=3)
        ttk.Button(row,text='从W25读回(&read)',command=self.read_gains).pack(side='left',padx=3)
        ttk.Button(row,text='读取转向状态(&tstat)',command=self.request_status).pack(side='left',padx=3)

        turning=ttk.LabelFrame(parent,text='原地精转（固件需 &EN 且 IMU 零漂就绪；执行中按钮禁用，收到 [TURN] done 或 8 s 超时后恢复）',padding=8)
        turning.pack(fill='x',padx=12,pady=(8,0))
        row=ttk.Frame(turning);row.pack(fill='x')
        for text,command in (('左转90','&L90'),('右转90','&R90'),('左转180','&L180'),('右转180','&R180')):
            self.turn_button(row,text,command)
        row=ttk.Frame(turning);row.pack(fill='x',pady=(2,0))
        ttk.Label(row,text='自定义角度').pack(side='left',padx=(10,2))
        ttk.Entry(row,textvariable=self.angle,width=6).pack(side='left')
        ttk.Label(row,text='°（1～360 整数）').pack(side='left',padx=(2,10))
        self.turn_button(row,'左转',lambda:'&L%d'%turn_angle(self.angle.get()))
        self.turn_button(row,'右转',lambda:'&R%d'%turn_angle(self.angle.get()))
        row=ttk.Frame(turning);row.pack(fill='x',pady=(2,0))
        for text,command in (('使能底盘(&en)','&en'),('停止(&stop)','&stop'),('航向归零(&zero)','&zero')):
            ttk.Button(row,text=text,command=lambda c=command:self.perform(lambda:self.send_and_note(c,'已入队：'+c))).pack(side='left',padx=3)
        ttk.Label(turning,text='&zero 需静止约 3 s；&stop 只停当前动作。转向被拒时请看串口日志里的 [TURN] rejected 原因。',
                  wraplength=1050).pack(fill='x',pady=(4,0))

        wave=ttk.LabelFrame(parent,text='目标角度 vs 实际角度（最近 10 秒，按 PC 接收时间对齐）',padding=8)
        wave.pack(fill='both',expand=True,padx=12,pady=(8,6))
        head=ttk.Frame(wave);head.pack(fill='x')
        ttk.Checkbutton(head,text='波形记录（&ttrace）',variable=self.record,command=self.toggle_trace).pack(side='left',padx=3)
        ttk.Button(head,text='清空波形',command=self.clear_wave).pack(side='left',padx=3)
        ttk.Label(head,textvariable=self.trace_status).pack(side='left',padx=10)
        ttk.Label(head,text='绿虚线=目标角度（阶跃） · 青实线=实际角度').pack(side='right',padx=6)
        self.canvas=tk.Canvas(wave,bg='#f7fafc',height=340,highlightthickness=0)
        self.canvas.pack(fill='both',expand=True,pady=(4,0))
        self.canvas.bind('<Configure>',lambda event:self.draw(force=True))
        ttk.Label(parent,textvariable=self.message,padding=(12,4),wraplength=1100).pack(fill='x')
        parent.bind('<Destroy>',self._destroy,add='+')
        self._after=parent.after(60,self.poll)

    def turn_button(self,parent,text,command):
        button=ttk.Button(parent,text=text,command=lambda:self.perform(lambda:self.execute_turn(command)))
        button.pack(side='left',padx=3,pady=2)
        self.turn_buttons.append(button)
        return button

    def perform(self,callback):
        try:callback()
        except (ValueError,RuntimeError,KeyError,tk.TclError) as exc:
            self.message.set(str(exc))
            messagebox.showerror('转向 PID',str(exc),parent=self.parent)

    def require_connected(self):
        if not self.logger.connected or self.logger.stop_event.is_set():
            raise ValueError('请先在“主控串口日志”页接入主控串口')

    def send_and_note(self,command,note):
        self.require_connected()
        self.logger.send(command)
        self.message.set(note)

    def send_turn_command(self,command):
        self.require_connected()
        if self.busy:raise ValueError('上一个转向动作仍在执行，请等 [TURN] done 或 8 s 超时')
        self._require_applied_gains()
        if callable(command):command=command()   # 自定义角度的数值校验放在连接检查之后
        self.logger.send(command)
        self.busy=True;self.busy_since=time.perf_counter()
        self.message.set('已入队 %s；等待主控 [TURN] start/done（最长等 8 s）'%command)
        self.update_buttons()

    def _require_applied_gains(self):
        """拦住“只改了输入框、没点应用到RAM 就转向”的情况。

        这是现场“我调了参数，怎么运行还是原来的”的最常见原因之一：
        输入框只是本地预览，主控 RAM 里的 kp/ki/kd 只有点“应用到RAM”才会变。
        """
        if self.applied is None:return   # 还没读到过主控值，不拦（此时状态行会提示先读取）
        try:current={key:gain_number(self.gains[key].get(),key.upper()) for key in GAIN_KEYS}
        except ValueError:return          # 参数本身非法，交给后续输入校验报错
        changed=[key for key in GAIN_KEYS if abs(current[key]-self.applied[key])>1e-9]
        if changed:
            raise ValueError('参数还没应用到主控（%s 与当前生效值不同）：先点“应用到RAM(&kp/&ki/&kd)”再转向，'
                             '否则主控仍按旧 PID 运行'%'/'.join(key.upper() for key in changed))

    def execute_turn(self,command):
        self.send_turn_command(command)

    def request_pid(self):
        self.perform(lambda:self.send_and_note('&pid','已请求 &pid，等待 [PID] 回执；读数将自动填入参数框'))

    def request_status(self):
        self.perform(lambda:self.send_and_note('&tstat','已请求 &tstat，等待 [TURN] status 回执'))

    def apply_gains(self):
        def action():
            values={key:gain_number(self.gains[key].get(),key.upper()) for key in GAIN_KEYS}
            self.require_connected()
            for key in GAIN_KEYS:self.logger.send('&%s%s'%(key,gain_text(values[key])))
            self.applied=dict(values)
            self.message.set('已入队 &kp/&ki/&kd 三条参数指令（立即生效，不写 W25；'
                             '要断电保留请再点“保存到W25”）')
        self.perform(action)

    def save_gains(self):
        self.perform(lambda:self.send_and_note('&save','已入队 &save，等待 [PID] saved 回执（同时保存导航参数）'))

    def read_gains(self):
        self.perform(lambda:self.send_and_note('&read','已入队 &read，等待 [PID] loaded 回执'))

    def toggle_trace(self):
        try:
            self.require_connected()
            enabled=self.record.get()
            self.logger.send('&ttrace %d'%(1 if enabled else 0))
            self.message.set('已发送 &ttrace %d，等待 [TURN] trace 回执'%(1 if enabled else 0))
        except (ValueError,RuntimeError) as exc:
            # 发送失败要把勾选状态改回去，否则界面显示的意愿和固件不一致。
            self.record.set(not self.record.get())
            self.message.set(str(exc))
            messagebox.showerror('转向 PID 波形',str(exc),parent=self.parent)

    def clear_wave(self):
        try:self.logger.turn_clear()
        except (ValueError,RuntimeError) as exc:
            self.message.set(str(exc));return
        self.snapshot=self.logger.turn_snapshot()
        self.message.set('已清空本机波形缓存（不影响主控）')
        self.draw(force=True)

    def update_buttons(self):
        state='disabled' if self.busy else 'normal'
        for button in self.turn_buttons:button.configure(state=state)

    def reset_after_session(self):
        self.busy=False
        self.applied=None;self.source=None
        self.start_seen=self.done_seen=self.reject_seen=None
        self.trace_seen=self.pid_seen=self.pid_note_seen=None
        self.record.set(True)
        self.trace_status.set('波形开关：未确认（勾选已复位为默认，重连后需再次勾选才发送）')
        self.update_buttons()

    def poll(self):
        self._after=None
        if self.closed:return
        try:self.refresh()
        except (ValueError,RuntimeError,KeyError,tk.TclError) as exc:self.message.set(str(exc))
        finally:
            if not self.closed:self._after=self.parent.after(60,self.poll)

    def refresh(self):
        snapshot=self.logger.turn_snapshot()
        self.snapshot=snapshot
        connected=self.logger.connected and not self.logger.stop_event.is_set()
        if self.session!=self.logger.session:
            self.session=self.logger.session
            self.reset_after_session()
        if not connected and self.busy:
            self.busy=False;self.update_buttons()
        now=time.perf_counter()
        # 参数/波形回执先处理，转向事件后处理：同一轮里以转向事件的消息为准。
        trace=snapshot.get('trace')
        if trace and (self.trace_seen is None or trace['received']>self.trace_seen):
            self.trace_seen=trace['received']
            period='' if trace.get('period_ms') is None else '（周期 %g ms）'%trace['period_ms']
            self.trace_status.set('波形开关：固件回执'+('开' if trace['on'] else '关')+period)
        pid=snapshot.get('pid')
        if pid and (self.pid_seen is None or pid['received']>self.pid_seen):
            self.pid_seen=pid['received']
            self.applied={key:pid[key] for key in GAIN_KEYS}
            self.source=pid.get('source') or self.source
            for key in GAIN_KEYS:self.gains[key].set('%.4f'%pid[key])
            note='' if not self.source else '（来源：%s）'%source_text(self.source)
            self.message.set('已读取当前 PID：kp=%.4f ki=%.4f kd=%.4f%s'%(pid['kp'],pid['ki'],pid['kd'],note))
        note=snapshot.get('pid_note')
        if note and (self.pid_note_seen is None or note['received']>self.pid_note_seen):
            self.pid_note_seen=note['received']
            if pid is None or pid['received']!=note['received']:
                self.message.set('主控回执：'+note['text'])
        start=snapshot.get('start')
        if start and (self.start_seen is None or start['received']>self.start_seen):
            self.start_seen=start['received']
            if snapshot.get('active'):
                # 任何来源的 [TURN] start（包括导航自动对准）都算动作执行中。
                self.busy=True;self.busy_since=start['received']
                self.message.set('主控已开始转向：目标 %s°，档位 %s'%(signed(start.get('tgt')),start.get('profile') or '?'))
        done=snapshot.get('done')
        if done and (self.done_seen is None or done['received']>self.done_seen):
            self.done_seen=done['received']
            if self.busy and done['received']>=self.busy_since:self.busy=False;self.update_buttons()
            self.message.set('转向完成：%.0f ms · 残留 %s · 过冲 %s · 档位 %s'%(
                done['ms'],signed(done.get('err'),3),signed(done.get('overshoot'),3),
                ' '.join(done.get('flags') or []) or '—'))
        reject=snapshot.get('reject')
        if reject and (self.reject_seen is None or reject['received']>self.reject_seen):
            self.reject_seen=reject['received']
            if self.busy and reject['received']>=self.busy_since:
                self.busy=False;self.update_buttons()
            self.message.set('主控拒绝转向：'+reject['reason'])
        if self.busy and now-self.busy_since>TURN_TIMEOUT:
            self.busy=False;self.update_buttons()
            self.message.set('等待 [TURN] done 超时 8 s，转向按钮已恢复；请核对主控日志与底盘状态')
        self.update_status(snapshot,connected)
        self.draw()

    def update_status(self,snapshot,connected):
        if not connected:
            self.status.set('串口未连接：先在“主控串口日志”页接入串口。')
            return
        if not snapshot.get('supported'):
            self.status.set('串口已连接，但尚未收到 [TURN]/[PID] 回执；请点“读取当前(&pid)”或“读取转向状态(&tstat)”（需 V234 及以上固件）。')
            return
        bits=['串口已连接']
        pid=snapshot.get('pid')
        if pid:bits.append('当前PID kp=%.4f ki=%.4f kd=%.4f'%(pid['kp'],pid['ki'],pid['kd']))
        status=snapshot.get('status')
        if status:
            if status.get('enabled') is not None:bits.append('底盘'+('已使能' if status['enabled']==1 else '未使能'))
            if status.get('active') is not None:bits.append('转向'+('执行中' if status['active']==1 else '空闲'))
            if status.get('trace') is not None:bits.append('波形'+('开' if status['trace']==1 else '关'))
            # V235：显示当前档位的轮速上限，用来确认“限幅翻倍”是否真的在跑。
            if status.get('rpm') is not None:bits.append('轮速上限%g RPM'%status['rpm'])
        # V236：显示 PID 来源，直接回答“为什么运行还是原来的参数”。
        src=(status or {}).get('src') or (pid or {}).get('source')
        if src:bits.append('参数来源'+source_text(src))
        reject=snapshot.get('reject');done=snapshot.get('done')
        if reject and (not done or reject['received']>done['received']):bits.append('最近拒绝：'+reject['reason'])
        self.status.set(' · '.join(bits))

    def draw(self,force=False):
        if self.closed:return
        now=time.perf_counter()
        if not force and now-self.last_draw<DRAW_MIN_INTERVAL:return
        self.last_draw=now
        try:self.draw_wave(now)
        except (ValueError,KeyError,tk.TclError):pass  # 缩窗口/切换页时的瞬时尺寸不打断面板

    def draw_wave(self,now):
        canvas=self.canvas;canvas.delete('all')
        width=max(canvas.winfo_width(),640);height=max(canvas.winfo_height(),300)
        left,right,top,bottom=58,width-16,24,height-34
        snapshot=self.snapshot or {}
        samples=[sample for sample in (snapshot.get('samples') or []) if sample[0]>=now-WAVE_SECONDS]
        canvas.create_rectangle(left,top,right,bottom,outline='#cbd5e1')
        if not samples:
            canvas.create_text(width/2,height/2,text='暂无波形：勾选“波形记录”发 &ttrace 1，再执行一次转向（或点“读取转向状态”）',
                               fill='#64748b')
        else:
            self.draw_axes(canvas,left,right,top,bottom,now)
            self.draw_curves(canvas,left,right,top,bottom,now,samples)
        self.draw_legend(canvas,left,top)
        self.draw_readout(canvas,right,top,snapshot,samples)
        canvas.create_text(width/2,height-10,
                           text='窗口=最近 10 s · 目标=主控下发角度 · 实际=主控回传 act · 数据来自 [TURNW] 约 40 Hz',
                           fill='#64748b')

    def window_values(self,samples):
        values=[sample[2] for sample in samples]+[sample[3] for sample in samples]
        low=min(min(values),-10.);high=max(max(values),10.)
        pad=max((high-low)*.08,2.)
        return low-pad,high+pad

    def draw_axes(self,canvas,left,right,top,bottom,now):
        low,high=self.window_values([sample for sample in (self.snapshot or {}).get('samples') or []
                                     if sample[0]>=now-WAVE_SECONDS])
        span=high-low
        step=next((value for value in (5.,10.,20.,30.,45.,90.,180.,360.) if span/value<=8),360.)
        value=math.ceil(low/step)*step
        while value<=high+1e-9:
            y=bottom-(value-low)/(high-low)*(bottom-top)
            zero=abs(value)<1e-9
            canvas.create_line(left,y,right,y,fill='#94a3b8' if zero else '#e2e8f0',dash=(6,4) if zero else ())
            canvas.create_text(left-8,y,text='0°' if zero else '%+.0f°'%value,anchor='e',fill='#64748b')
            value+=step
        for offset in range(11):
            x=right-offset/10*(right-left)
            canvas.create_line(x,top,x,bottom,fill='#eef2f7')
            if offset%2==0:
                label='now' if offset==0 else '-%ds'%offset
                canvas.create_text(x,bottom+12,text=label,fill='#94a3b8')

    def draw_curves(self,canvas,left,right,top,bottom,now,samples):
        low,high=self.window_values(samples)
        def map_x(perf):return left+(perf-(now-WAVE_SECONDS))/WAVE_SECONDS*(right-left)
        def map_y(value):return bottom-(value-low)/(high-low)*(bottom-top)
        # 2000 点全画会拖慢 Tk；按约 500 点抽样，末点始终保留。
        stride=max(1,len(samples)//500)
        reduced=samples[::stride]
        if reduced[-1] is not samples[-1]:reduced.append(samples[-1])
        actual=[]
        for sample in reduced:actual.extend((map_x(sample[0]),map_y(sample[3])))
        target=[];last_value=None
        for sample in reduced:
            x,value=map_x(sample[0]),sample[2]
            if last_value is None:target.extend((x,map_y(value)))
            elif value!=last_value:target.extend((x,map_y(last_value),x,map_y(value)))
            else:target.extend((x,map_y(value)))
            last_value=value
        if len(target)>=4:canvas.create_line(*target,fill='#16a34a',width=2,dash=(7,4))
        elif target:canvas.create_oval(target[0]-2,target[1]-2,target[0]+2,target[1]+2,fill='#16a34a',outline='')
        if len(actual)>=4:canvas.create_line(*actual,fill='#0891b2',width=2)
        elif actual:canvas.create_oval(actual[0]-2,actual[1]-2,actual[0]+2,actual[1]+2,fill='#0891b2',outline='')

    def draw_legend(self,canvas,left,top):
        canvas.create_line(left+12,top+10,left+42,top+10,fill='#16a34a',width=2,dash=(7,4))
        canvas.create_text(left+48,top+10,text='目标角度（阶跃）',anchor='w',fill='#334155')
        canvas.create_line(left+12,top+28,left+42,top+28,fill='#0891b2',width=2)
        canvas.create_text(left+48,top+28,text='实际角度',anchor='w',fill='#334155')

    def readout(self,snapshot,samples):
        latest=samples[-1] if samples else None
        start=snapshot.get('start') or {};done=snapshot.get('done') or {};status=snapshot.get('status') or {}
        if latest:tgt,act,err,out=latest[2],latest[3],latest[4],latest[5]
        else:tgt,act,err,out=start.get('tgt'),start.get('act'),status.get('err'),status.get('out')
        if snapshot.get('active'):duration='进行中'
        elif done.get('ms') is not None:duration='%.0f ms'%done['ms']
        else:duration='—'
        profile=start.get('profile') or ' '.join(done.get('flags') or []) or '—'
        return [('目标',signed(tgt,2,'°')),('实际',signed(act,2,'°')),('误差',signed(err,3,'°')),
                ('输出',signed(out,3)),('耗时',duration),('档位',profile)]

    def draw_readout(self,canvas,right,top,snapshot,samples):
        y=top+10
        for label,text in self.readout(snapshot,samples):
            canvas.create_text(right-8,y,text='%s %s'%(label,text),anchor='ne',fill='#0f172a')
            y+=17

    def _destroy(self,event):
        if event.widget is not self.parent:return
        self.closed=True
        if self._after is not None:
            self.parent.after_cancel(self._after);self._after=None
