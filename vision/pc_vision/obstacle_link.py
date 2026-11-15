"""Follow the MCU competition session and publish fresh CAM2 observations.

All calls are made under the SerialLog lock. The MCU state is authoritative:
this link never sends an arm, stop, retry, start, or speed command.
"""
import math
import re
import time

STATE_MAX_AGE = .75
FRAME_MAX_AGE = .20


def parse_state(text, now):
    if not text.startswith('[VOBS STATE] '):
        return None
    values = dict(re.findall(r'([a-z]+)=([^\s]+)', text))
    try:
        version, session = int(values['v']), int(values['session'])
        armed, eligible = int(values['armed']), int(values['eligible'])
        x, y, yaw = (float(values[key]) for key in ('x', 'y', 'yaw'))
    except (KeyError, ValueError):
        return None
    if version not in (1,2) or not 0 <= session <= 0xffffffff or armed not in (0, 1) or eligible not in (0, 1):
        return None
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return None
    state=dict(v=version, session=session, armed=bool(armed), eligible=bool(eligible),
               x=x, y=y, yaw=yaw, received_perf_s=now)
    try:
        count=int(values['obstacles'])
        if 0<=count<=2:state['obstacles']=count
    except (KeyError,ValueError):pass
    for key in ('enabled','arc','manual'):
        if values.get(key) in ('0','1'):state[key]=values[key]=='1'
    return state


class ObstacleLink:
    def __init__(self):
        self.phase = 'WAIT_STATE'
        self.state = None
        self.expected_session = None
        self.pending = None
        # Keep this counter across a serial reconnect; wall-clock seeding also
        # lets a newly started PC process follow an already-armed MCU session.
        self.sequence = int(time.time() * 1000) & 0xffffffff
        self.last_sent_key = None
        self.last_sent_time = -1e9
        self.eligible_since = None
        self.blocked_session = None
        self.await_new_session = False
        self.sent = 0
        self.last_command = None
        self.reason = '等待主控串口与比赛状态'

    def connected(self):
        self.phase = 'WAIT_STATE'
        self.state = self.pending = self.expected_session = None
        self.eligible_since = None
        self.last_sent_key = None
        self.reason = '等待主控比赛状态'

    def disconnected(self):
        self.connected()
        self.reason = '串口未连接，障碍观测不发送'

    def cancel(self, reason):
        # A manual STOP/reset/prepare may have old STATE lines already in the
        # UART buffer. Never re-adopt the same session after such a command.
        if self.state is not None:
            self.blocked_session = self.state['session']
        elif self.expected_session is not None:
            self.blocked_session = self.expected_session
        self.await_new_session = True
        self.phase = 'WAIT_SESSION'
        self.state = self.pending = self.expected_session = None
        self.eligible_since = None
        self.last_sent_key = None
        self.reason = reason

    def receive(self, text, now):
        state = parse_state(text, now)
        if state is None:
            return
        session = state['session']
        if self.await_new_session:
            if self.blocked_session is None:
                # Before we have seen any session, only an unarmed state can
                # establish the old epoch; a later epoch must be different.
                if not state['armed']:
                    self.blocked_session = session
                return
            if session == self.blocked_session:
                return
            self.await_new_session = False
            self.blocked_session = None
        old = self.state
        changed = old is None or old['session'] != session
        became_eligible = state['armed'] and state['eligible'] and (
            changed or not old['armed'] or not old['eligible']
            or now - old['received_perf_s'] > STATE_MAX_AGE)
        if changed or not state['armed'] or not state['eligible']:
            self.pending = None
        if changed:
            self.last_sent_key = None
            self.last_sent_time = -1e9
        self.state = state
        if not state['armed']:
            self.expected_session = None
            self.eligible_since = None
            self.phase = 'WAIT_ARMED'
            self.reason = '主控视觉避障已关闭，图像识别继续' if state.get('enabled') is False else '等待主控准备比赛'
        elif not state['eligible']:
            self.expected_session = session
            self.eligible_since = None
            self.phase = 'WAIT_ELIGIBLE'
            self.reason = '已接入比赛，等待可观测的前向路段'
        else:
            self.expected_session = session
            if became_eligible:
                self.eligible_since = now
                self.pending = None
                self.reason = '已接入比赛，等待CAM2运行画面'
            self.phase = 'ACTIVE'

    def submit(self, snapshot):
        self.pending = None
        if self.phase == 'ACTIVE' and (not snapshot or snapshot.get('operation') != 'run'):
            self.reason = '等待CAM2接入运行画面'
        if (self.phase != 'ACTIVE' or not snapshot or snapshot.get('operation') != 'run'
                or snapshot.get('camera_role') != 'qr'):
            return
        report = snapshot.get('obstacle_report') or {}
        selected = report.get('selected')
        frame_time = report.get('frame_pc_perf_s')
        if (not report.get('reportable') or selected is None or frame_time is None
                or self.eligible_since is None or frame_time < self.eligible_since):
            return
        self.pending = {'frame_key': tuple(report['frame_key']),
                        'frame_pc_perf_s': frame_time,
                        'gap_mm': selected['front_gap_mm'], 'right_mm': selected['body_right_mm'],
                        'width_px': selected.get('width_px',50.), 'bbox_width': selected.get('bbox',[0,0,50,0])[2]}

    def observation_command(self, now):
        if self.phase != 'ACTIVE':
            return None
        state = self.state
        if (not state or not state['armed'] or not state['eligible']
                or state['session'] != self.expected_session
                or not 0 <= now - state['received_perf_s'] <= STATE_MAX_AGE):
            self.reason = '等待新鲜主控状态'
            return None
        if not self.pending:
            self.reason = '等待连续3帧确认近处圆柱'
            return None
        age = now - self.pending['frame_pc_perf_s']
        if not 0 <= age <= FRAME_MAX_AGE:
            self.pending = None
            self.reason = '等待新的相机观测'
            return None
        if state['v']==1 and (self.pending['gap_mm']>500 or self.pending['width_px']<49 or self.pending['bbox_width']<50):
            self.reason='旧主控仅接收500mm内近距观测；新分段避障需V260'
            return None
        key = self.pending['frame_key']
        if key == self.last_sent_key or now - self.last_sent_time < .20:
            return None
        self.sequence = (self.sequence + 1) & 0xffffffff
        gap, right = round(self.pending['gap_mm']), round(self.pending['right_mm'])
        command = 'vobs %d %d %d %d %d' % (self.expected_session, self.sequence, gap, right, round(age * 1000))
        self.last_sent_key = key
        self.last_sent_time = now
        self.reason = '已上报圆柱：车头净距约%dmm，右向%+dmm' % (gap, right)
        self.last_command = command
        self.sent += 1
        return command

    def summary(self):
        return {'phase': self.phase, 'state': dict(self.state) if self.state else None,
                'reason': self.reason, 'sent': self.sent, 'last_command': self.last_command}
