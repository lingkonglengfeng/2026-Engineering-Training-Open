"""Read-only per-run timing, retained even when the map opens late."""
import copy
import re
import time

STAGES = (('LEAVE', '出库', None), ('QR', '二维码', 'qr'),
          ('RAW_BATCH1', '一轮圆盘', 'raw'), ('ROUGH_BATCH1', '一轮ROUGH', 'rough'),
          ('TEMP_BATCH1', '一轮TEMP', 'temp'), ('RAW_BATCH2', '二轮圆盘', 'raw'),
          ('ROUGH_BATCH2', '二轮ROUGH', 'rough'), ('TEMP_BATCH2', '二轮TEMP', 'temp'),
          ('RETURN_START', '回库', None))
KEYS = {key for key, _, _ in STAGES}


def seconds(ms):
    return '—' if ms is None else '%.2fs' % (ms / 1000)


class MatchTiming:
    def __init__(self):
        self.origin = self.current_at = self.current = self.frozen_at = None
        self.rows = {}; self.state = '等待发车'; self.revision = 0

    def receive(self, line, stamp):
        if line.startswith('[ARM] CAMERA_DEPLOY_V'):
            self.__init__(); return
        stage = re.match(r'^\[MATCH STAGE\] (\w+) from_start_ms=(\d+) stage_ms=(\d+)\b', line)
        if stage and stage[1] in KEYS:
            key = stage[1]; total, duration = int(stage[2]), int(stage[3])
            if not 0 <= duration <= total <= 0xffffffff: return
            self.rows[key] = dict(duration_ms=duration, cumulative_ms=total, source='主控')
            if self.origin is None: self.origin = stamp-total/1000
            self.revision += 1; return
        total = re.match(r'^\[MATCH TIME\] total_ms=(\d+)\b', line)
        if total:
            self.origin = stamp-int(total[1])/1000; self.frozen_at = stamp; return
        match = re.match(r'^\[MATCH\] (LEAVING|MOVING|WAIT_OK|FINISHED|FAULT|IDLE|PREPARING|WAIT_START_OK)\s+(\w+)\b', line)
        if not match: return
        state, key = match.groups()
        if state == 'LEAVING':
            self.__init__(); self.origin = self.current_at = stamp; self.current = 'LEAVE'
            self.state = '运行中'; self.revision += 1; return
        if state == 'MOVING' and key in KEYS:
            # Old firmware lacks live milestones. Preserve explicitly marked
            # PC estimates until its exact end-of-run records arrive.
            self._complete(stamp)
            self.current, self.current_at = key, stamp; self.frozen_at = None; self.state = '运行中'
        elif state == 'WAIT_OK':
            self._complete(stamp); self.current = None; self.state = '等待继续'
        elif state in ('FINISHED', 'FAULT', 'IDLE', 'PREPARING', 'WAIT_START_OK'):
            if state == 'FINISHED': self._complete(stamp)
            self.frozen_at = stamp
            self.state = {'FINISHED':'已完成', 'FAULT':'已停止', 'IDLE':'空闲',
                          'PREPARING':'准备中', 'WAIT_START_OK':'等待发车'}[state]
            if state in ('IDLE', 'PREPARING', 'WAIT_START_OK'): self.current = None
        self.revision += 1

    def _complete(self, stamp):
        if self.current and self.current not in self.rows and self.origin is not None:
            self.rows[self.current] = dict(duration_ms=round((stamp-self.current_at)*1000),
                cumulative_ms=round((stamp-self.origin)*1000), source='PC估算')

    def snapshot(self, now=None):
        now = time.perf_counter() if now is None else now
        end = self.frozen_at if self.frozen_at is not None else now
        return dict(rows=copy.deepcopy(self.rows), current=self.current, state=self.state,
            elapsed_ms=None if self.origin is None else max(0, round((end-self.origin)*1000)),
            current_ms=None if self.current_at is None or self.origin is None else max(0, round((end-self.current_at)*1000)),
            revision=self.revision)


def draw_stage_times(canvas, xy, snapshot, points):
    for station in ('qr', 'raw', 'rough', 'temp'):
        labels = []
        for key, title, place in STAGES:
            if place != station: continue
            row = snapshot.get('rows', {}).get(key)
            if row: labels.append('%s %s%s' % (title, seconds(row['duration_ms']), '≈' if row['source']=='PC估算' else ''))
            elif snapshot.get('current') == key: labels.append(title+'进行中')
        if labels:
            p = points[station]; x, y = xy(p['x'], p['y'])
            canvas.create_text(x, y+12, text='\n'.join(labels), anchor='n', fill='#7c3aed', tags='stage_time')
