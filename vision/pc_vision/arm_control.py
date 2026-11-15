"""Arm diagnostics, calibrated degree mapping and acknowledged bench actions.

No serial IO here: the existing SerialLog remains the sole UART owner.
"""
from collections import deque
import copy
import math
import re
import time
from arm_rates import RateState, timeout_factor, POINTS


def fields(line):
    return dict(re.findall(r'(\w+)=([^\s]+)', line))


def reply_line(line):
    """UART prompts have no newline and can precede any asynchronous reply."""
    line = line.strip()
    while line.startswith('ZDT>'):
        line = line[4:].lstrip()
    return line


def number(data, key):
    value = float(data[key])
    if not math.isfinite(value):
        raise ValueError('主控返回非有限数值：'+key)
    return value


def valid_snapshot(data):
    try:
        if data['version'].get('v') != '1': return False
        for axis in ('5', '6'):
            record = data['axes'][axis]
            if not number(record, 'span') or number(record, 'travel') <= 0: return False
            number(record, 'angle')
        rotation, guard = data['rotation'], data['guard']
        if number(rotation, 'min') >= number(rotation, 'max'): return False
        number(rotation, 'angle')
        for key in ('threshold', 'min', 'max'): number(guard, key)
        if guard.get('valid') == '1': number(guard, 'height')
        return 'state' in data
    except (KeyError, ValueError, TypeError):
        return False


class ArmState:
    def __init__(self):
        self.data = {}
        self.points = {}
        self.disc_points = {}
        self._disc_points = {}
        self._pending = None
        self._points = {}
        self.sequence = 0
        self.rates = RateState()
        self.events = deque(maxlen=150)

    def receive(self, line, now):
        line = reply_line(line)
        self.sequence += 1
        self.events.append((self.sequence, now, line))
        self.rates.receive(line,now)
        if line == '[ARM DISC POINTS] BEGIN':
            self._disc_points = {}
        elif line.startswith('[ARM DISC POINT]'):
            record = fields(line)
            if record.get('pose') in ('wait', 'grab', 'tray', 'traypick'):
                try:
                    for key in ('base_l', 'base_r', 'base_d', 'dl', 'dr', 'dd'): number(record, key)
                    self._disc_points[record['pose']] = record
                except (KeyError, ValueError): pass
        elif line == '[ARM DISC POINTS] END':
            expected = {'wait', 'grab', 'tray'}
            if self.data.get('version', {}).get('disc_pose_count') == '4': expected.add('traypick')
            if set(self._disc_points) == expected: self.disc_points = self._disc_points
            self._disc_points = {}
        if line.startswith('[ARM UI BEGIN]'):
            self._pending = {'version': fields(line), 'axes': {}, 'servos': {}}
        elif line == '[ARM UI END]':
            if self._pending is not None and valid_snapshot(self._pending):
                self._pending['received'] = now
                self.data = self._pending
            self._pending = None
        elif self._pending is not None:
            data = fields(line)
            if line.startswith('[ARM UI AXIS]'):
                self._pending['axes'][data.get('id')] = data
            elif line.startswith('[ARM UI ROT]'):
                self._pending['rotation'] = data
            elif line.startswith('[ARM UI GUARD]'):
                self._pending['guard'] = data
            elif line.startswith('[ARM UI STATE]'):
                self._pending['state'] = data
            elif line.startswith('[ARM] grip ') or line.startswith('[ARM] table '):
                self._pending['servos'][line.split()[1]] = data
            elif line.startswith('[ARM PRESETS]'):
                self._pending['presets'] = data
        if line.startswith('[ARM POINT]'):
            data = fields(line)
            key = (data.get('area'), data.get('ring'))
            if key[0] in ('rough', 'temp') and key[1] in ('1', '2', '3'):
                try:
                    for field in ('base_l', 'base_r', 'base_d', 'dl', 'dr', 'dd'): number(data, field)
                    self._points[key] = data
                except (KeyError, ValueError): pass
        elif line == '[ARM POINTS] END':
            if len(self._points) == 6:
                self.points = self._points
            self._points = {}

    def snapshot(self):
        return copy.deepcopy({'data': self.data, 'points': self.points, 'disc_points': self.disc_points, 'rates':self.rates.snapshot(),
                              'sequence': self.sequence, 'events': list(self.events)})

    def manual_cancel(self, text):
        self.receive('[PC MANUAL CANCEL] '+text, time.perf_counter())


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('角度必须是有限数值')
    return value


def degree_span(data, axis):
    return number(data['axes'][axis], 'span') * 360 / 65536


def displacement(data, axis, degrees):
    span = degree_span(data, axis)
    if abs(span) < 1e-6:
        raise ValueError('请先读取有效行程标定')
    return finite(degrees) / span * number(data['axes'][axis], 'travel')


def pose_check(data, pose, allow_retract_reference=False):
    """Check a final pose using the same calibrated endpoint/height convention."""
    lift, reach, dm = map(finite, pose)
    for axis, angle in (('5', lift), ('6', reach)):
        span = degree_span(data, axis)
        percent = angle / span if span else -1
        minimum = -3.0/number(data['axes']['6'], 'travel') if axis == '6' and allow_retract_reference else 0
        if not minimum <= percent <= 1:
            raise ValueError(('升降' if axis == '5' else '伸缩')+'目标超出标定行程')
    rot = data['rotation']
    if not number(rot, 'min') <= dm <= number(rot, 'max'):
        raise ValueError('旋转目标超出主控角度范围')
    guard = data['guard']
    if guard.get('enabled') == '1':
        drop = displacement(data, '5', lift)
        height = -drop if guard.get('top') == '1' else number(data['axes']['5'], 'travel')-drop
        if height < number(guard, 'threshold') and not number(guard, 'min') <= dm <= number(guard, 'max'):
            raise ValueError('目标下降高度已进入旋转限制区，请调整高度或旋转角度')
    return lift, reach, dm


def move_command(data, pose, allow_retract_reference=False, point=None):
    if point is not None and point not in dict(POINTS):raise ValueError('未知速率点位')
    return 'arm move '+' '.join('%.3f' % v for v in pose_check(data, pose, allow_retract_reference))+(' '+point if point else '')


def axis_command(data, axis, value):
    value = finite(value)
    if axis == 'dm':
        rot, guard = data['rotation'], data['guard']
        if rot.get('valid') != '1':
            raise ValueError('旋转实际角度无效，请刷新主控状态')
        if not number(rot, 'min') <= value <= number(rot, 'max'):
            raise ValueError('旋转角度超出主控范围')
        if guard.get('enabled') == '1':
            if guard.get('valid') != '1':
                raise ValueError('当前高度尚未确认，请先刷新主控状态')
            if number(guard, 'height') < number(guard, 'threshold'):
                low, high = number(guard, 'min'), number(guard, 'max')
                if not low <= value <= high or not low-.5 <= number(rot, 'angle') <= high+.5:
                    raise ValueError('当前下降高度仅允许在主控规定的角度区间内旋转')
        return 'arm rotate %.3f' % value
    span = degree_span(data, axis)
    if not span or not 0 <= value/span <= 1:
        raise ValueError('目标超出标定行程')
    if data['axes'][axis].get('valid') != '1':
        raise ValueError('当前轴反馈无效，请刷新主控状态')
    if axis == '5':
        # A direct descent cannot raise first like the three-axis sequencer.
        guard = data['guard']
        drop = displacement(data, '5', value)
        height = -drop if guard.get('top') == '1' else number(data['axes']['5'], 'travel')-drop
        if guard.get('enabled') == '1' and height < number(guard, 'threshold') and data['rotation'].get('valid') != '1':
            raise ValueError('低位下降需要有效旋转角度，请先刷新主控状态')
        if guard.get('enabled') == '1' and height < number(guard, 'threshold'):
            if not number(guard, 'min') <= number(data['rotation'], 'angle') <= number(guard, 'max'):
                raise ValueError('目标下降高度要求旋转轴位于主控允许的角度区间')
        return 'arm pct 5 %.4f' % (value/span*100)
    return 'arm pos 6 %.3f' % value


def axis_steps(data, axis, value, read_state):
    """An explicit send requests enable even if the hardware reports enabled.

    V196 separately tracks its software enable request; it is not present in
    the UI snapshot. Reuse its existing enable handshake before sending a goal.
    """
    value = finite(value)
    if axis == 'dm':
        if not number(data['rotation'], 'min') <= value <= number(data['rotation'], 'max'):
            raise ValueError('旋转角度超出主控范围')
        guard = data['guard']
        if guard.get('enabled') == '1' and guard.get('valid') == '1' and number(guard, 'height') < number(guard, 'threshold'):
            if not number(guard, 'min') <= value <= number(guard, 'max'):
                raise ValueError('当前下降高度仅允许在主控规定的角度区间内旋转')
    elif axis in ('5', '6'):
        span = degree_span(data, axis)
        if not span or not 0 <= value/span <= 1:
            raise ValueError('目标超出标定行程')
    else:
        raise ValueError('未知机械臂轴')
    steps = []
    if axis == '5' and data['guard'].get('enabled') == '1':
        guard = data['guard']
        drop = displacement(data, '5', value)
        height = -drop if guard.get('top') == '1' else number(data['axes']['5'], 'travel')-drop
        if height < number(guard, 'threshold'):
            steps.append(('dm query', '[DM] QUERY'))
    query_after = [0.0]

    def query():
        query_after[0] = time.perf_counter()
        return 'arm ui'

    steps += [('arm enable '+axis, '[DM] ON confirmed' if axis == 'dm' else '[ARM] HAL=0'),
              (query, '[ARM UI END]')]

    def goal():
        current = read_state()
        if not current or current.get('received', 0) < query_after[0] or time.perf_counter()-current.get('received', 0) > 3:
            raise ValueError('使能后未收到新的主控状态，未发送目标')
        if current['state'].get('busy') == '1':
            raise ValueError('主控已有机械臂任务，未发送单轴目标')
        actual = current['rotation'] if axis == 'dm' else current['axes'][axis]
        if actual.get('enabled') != '1':
            raise ValueError('主控尚未确认该轴使能，未发送目标')
        return axis_command(current, axis, value)

    steps.append((goal, '[ARM] HAL=0'))
    return steps


def point_pose(point, offsets=None):
    base = tuple(number(point, key) for key in ('base_l', 'base_r', 'base_d'))
    delta = tuple(number(point, key) for key in ('dl', 'dr', 'dd')) if offsets is None else tuple(map(finite, offsets))
    return base, tuple(a+b for a, b in zip(base, delta))


class ArmActions:
    """A button chain advances on actual firmware completion, never a delay."""
    def __init__(self, send):
        self.send = send
        self.steps = deque()
        self.active = None
        self.last_sequence = 0
        self.sent_at = 0
        self.message = '未执行动作'
        self.completed_message = '动作完成（主控已回执）'

    def cancel(self, reason='后续步骤已取消'):
        self.steps.clear()
        self.active = None
        self.message = reason

    def start(self, steps, sequence, completed_message='动作完成（主控已回执）'):
        if self.active:
            raise ValueError('上一组动作仍在等待主控完成')
        self.last_sequence = sequence
        self.completed_message = completed_message
        self.steps = deque(steps)
        self._next()

    def _next(self):
        if not self.steps:
            self.active = None
            self.message = self.completed_message
            return
        command, result = self.steps.popleft()
        try:
            if callable(command): command = command()
            self.send(command)
        except Exception as exc:
            self.cancel('发送失败，后续步骤取消：'+str(exc))
            raise
        self.active = (command, result)
        self.sent_at = time.perf_counter()
        self.message = '等待主控：'+command

    def update(self, snapshot):
        for seq, stamp, line in snapshot['events']:
            if seq <= self.last_sequence:
                continue
            self.last_sequence = seq
            if not self.active or stamp < self.sent_at:
                continue
            command, expected = self.active
            if line.startswith('[PC MANUAL CANCEL]'):
                self.cancel('已按手动停止/复位请求取消后续步骤')
                break
            # Firmware refusals/failed composite jobs must never run the next action.
            if any(word in line for word in ('FAILED', 'REJECTED', 'refused:', 'busy;', 'invalid zero/span/reference')) or re.search(r'\bHAL=[123]\b', line):
                self.cancel('主控未完成动作：'+line)
                break
            if expected in line:
                try: self._next()
                except (ValueError, RuntimeError, KeyError): break
        if self.active and time.perf_counter()-self.sent_at > 125*timeout_factor(snapshot):
            self.cancel('等待回执超时，后续步骤取消；请核对日志与机械臂状态')
