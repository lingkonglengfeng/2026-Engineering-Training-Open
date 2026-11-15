"""CAM2 near-field black-cylinder experiment, using the user's two range anchors.

Input is the unannotated, upright 640x480 frame. Range is vehicle FRONT EDGE
to cylinder NEAR WALL, not optical-axis depth. The lateral estimate assumes a
50mm cylinder and a camera parallel to the vehicle, 100mm left of its centre.
"""
import time

import cv2
import numpy as np


CALIBRATION = {
    'resolution': [640, 480], 'distance_reference': 'front_edge_to_near_wall',
    'anchors': [{'width_px': 78., 'gap_mm': 300.}, {'width_px': 122., 'gap_mm': 200.}],
    'empirical_K_mm_px': 23900., 'range_formula': 'front_gap_mm approximately 23900 / width_px',
    'camera_body_right_mm': -100., 'diameter_mm': 50., 'principal_u_px': 319.5,
    'camera_yaw_assumed_deg': 0., 'independently_validated': False,
    'trigger_min_width_px': 50., 'max_front_gap_mm': 500.,
}


class ObstacleDetector:
    """No saved target is returned as a new observation after a missing frame."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.last_key = None
        self.last_time = None
        self.previous = None
        self.stable = 0
        self.report = None

    @staticmethod
    def usable_geometry(item):
        # The dark-pixel median can lose one edge pixel to JPEG/antialiasing;
        # use both its body width and the measured component width at the
        # threshold, while keeping the median for distance estimation.
        return (not item['side_clipped'] and item['width_px'] >= CALIBRATION['trigger_min_width_px'] - 1 and
                item['bbox'][2] >= CALIBRATION['trigger_min_width_px'] and
                50 <= item['front_gap_mm'] <= CALIBRATION['max_front_gap_mm'] and abs(item['body_right_mm']) <= 190)

    @staticmethod
    def split_foot(gray, labels, index, bounds):
        """Recover a rounded foot connected to a narrow dark background leg.

        Search for a wide run followed by a falling, rounded silhouette. Only
        use it when the ordinary whole-component geometry did not work; short
        chair legs and flat-ended floor marks cannot form the required arc.
        """
        x,y,w,h=map(int,bounds[:4])
        if w < 48 or w > 210 or h < 18:
            return None
        candidates=[]
        for row in range(y, min(y+h-4, 471)):
            xs=np.flatnonzero(labels[row,x:x+w] == index) + x
            if xs.size < 48:
                continue
            for run in np.split(xs, np.where(np.diff(xs)>1)[0]+1):
                if not 48 <= len(run) <= 208:
                    continue
                left,right=int(run[0]),int(run[-1])+1
                # When the arc is already narrowing, at most one pixel is
                # missing at each rim. The base still supplies the identity.
                if len(run)<50:
                    left=max(0,left-1);right=min(640,right+1)
                width=right-left
                if width < 50 or width > 210:
                    continue
                profile=np.mean(gray[row:row+28,left:right] < 65,axis=1)
                tips=np.flatnonzero(profile < .15)
                if not len(tips):
                    continue
                tip=int(tips[0])
                if tip < 4 or tip > 26 or profile[0] < .80:
                    continue
                changes=np.diff(profile[:tip+1])
                if np.count_nonzero(changes < -.04) < 4 or np.count_nonzero(changes > .08) > 1:
                    continue
                half=max(2,tip//2)
                if np.mean(-changes[-half:]) < np.mean(-changes[:half])*1.35+.005:
                    continue
                # A merged background leg separates into two significant dark
                # runs near the base. Exclude that compound width rather than
                # letting it inflate distance geometry.
                compound=False
                for scan in range(row,min(row+tip,480)):
                    dark=np.flatnonzero(gray[scan,left:right] < 65)
                    runs=np.split(dark,np.where(np.diff(dark)>1)[0]+1)
                    if sum(len(part)>=3 for part in runs)>1:
                        compound=True;break
                if compound:
                    continue
                foot_pixels=np.flatnonzero(gray[row+tip-2,left:right] < 65)+left
                foot_runs=[part for part in np.split(foot_pixels,np.where(np.diff(foot_pixels)>1)[0]+1)
                           if len(part)>=3]
                if not foot_runs:
                    continue
                proposed_center=(left+right-1)/2.
                foot=min(foot_runs,key=lambda part:abs((part[0]+part[-1])/2.-proposed_center))
                foot_center=(foot[0]+foot[-1])/2.
                if abs(foot_center-proposed_center)>max(2.,width*.05):
                    continue
                bottom=row+tip
                if bottom-275 < max(18,int(width*.20)):
                    continue
                # The foot must be bordered by floor on both sides. A narrow
                # neighbouring leg may obscure part of either margin; require
                # a contiguous bright patch, not a permissive pixel average.
                body=float(np.median(gray[row,left:right]))
                sides=(gray[row,max(0,left-45):max(0,left-3)],
                       gray[row,min(640,right+3):min(640,right+45)])
                bright=[np.convolve(np.uint8(side>body+35),np.ones(8,dtype=np.uint8),'valid')
                        for side in sides if len(side)>=8]
                if len(bright)!=2 or any(not np.any(side==8) for side in bright):
                    continue
                u=(left+right-1)/2.;gap=CALIBRATION['empirical_K_mm_px']/width
                top=max(230,bottom-int(width*2.0))
                candidate={'bbox':[left,top,width,bottom-top], 'width_px':float(width),
                           'center_u_px':round(u,2), 'front_gap_mm':round(gap,1),
                           'body_right_mm':round(-100.+50.*(u-319.5)/width,1),
                           'bottom_clipped':False, 'side_clipped':left<=2 or right>=638,
                           'range_state':('FAR_UNCALIBRATED' if width<78 else
                                          'NEAR_UNCALIBRATED' if width>122 else 'INITIAL_INTERVAL'),
                           'extrapolated':width<78 or width>122, 'rounding_evidence':'BASE_SPLIT',
                           'partial_top':True,
                           'contrast':round(min(float(np.percentile(side,80))-body for side in sides),1)}
                candidates.append(candidate)
        if not candidates:
            return None
        # Prefer a repeated, stable side-wall width. A single wider run can
        # include an attached leg, and a narrow run can be just the tip.
        counts={candidate['width_px']:sum(other['width_px']==candidate['width_px'] for other in candidates)
                for candidate in candidates}
        return max(candidates,key=lambda item:(counts[item['width_px']],item['width_px']))

    @staticmethod
    def candidates(bgr):
        if bgr is None or bgr.shape != (480, 640, 3):
            return []
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        # This floor ROI is tied to the tested installation; far chair bodies
        # are deliberately excluded. Colour alone does not establish identity.
        dark = (gray < 65)
        mask = np.uint8(dark) * 255
        # The floor starts around row 275 in the supplied CAM2 images. A
        # farther cylinder can have only its rounded foot below the horizon;
        # keep that section while leaving the chair bodies outside the ROI.
        mask[:275] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 3), np.uint8))
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        found = []
        accepted_components=set()
        for index, values in enumerate(stats[1:], 1):
            x, y, w, h, area = map(int, values)
            if area < 600 or w < 30 or w > 210 or h < 18 or area / (w * h) < .75:
                continue
            clipped_side = x <= 2 or x + w >= 638
            clipped_bottom = y + h >= 480
            rows, contrast = [], []
            for row in range(y + min(8, h // 8), y + h - min(6, h // 8)):
                xs = np.flatnonzero(labels[row] == index)
                if len(xs) < w * .65:
                    continue
                left, right = int(xs[0]), int(xs[-1])
                rows.append((left, right))
                left_bg = gray[row, max(0, left - 18):max(0, left - 4)]
                right_bg = gray[row, min(640, right + 5):min(640, right + 19)]
                if len(left_bg) and len(right_bg):
                    body = float(np.median(gray[row, left:right + 1]))
                    contrast.append(min(float(np.median(left_bg)), float(np.median(right_bg))) - body)
            if len(rows) < max(10, min(35, int(w * .25))) or not contrast or np.median(contrast) < 35:
                continue
            widths = np.asarray([b - a + 1 for a, b in rows])
            width = float(np.median(widths))
            lefts, rights = np.asarray(rows).T
            if np.percentile(widths, 90) / max(1., np.percentile(widths, 10)) > 1.18:
                continue
            if max(np.std(lefts), np.std(rights)) > max(4., width * .08):
                continue
            top = y
            while top > 230 and np.mean(dark[top - 1, x + w // 5:x + w - w // 5]) > .8:
                top -= 1
            visible_height = y + h - top
            ratio = visible_height / width
            # The upper portion of a distant object can overlap dark objects
            # behind the floor boundary. Retain a partial-top candidate only
            # when the visible foot still has enough side pixels and its base
            # is independently rounded below. This does not add a new frame
            # or bypass the normal contrast and three-frame confirmation.
            partial_top = (top <= 231 and not clipped_bottom and
                           h >= max(18, int(w * .30)))
            if not partial_top and (not (1.25 if clipped_bottom else 1.65) <= ratio <= 3.5 or top <= 231):
                continue
            # Require a rounded visible base, or a rounded top for a base that
            # extends out of the image. A flat black rectangle is not enough.
            fractions = np.mean(gray[:, x:x + w] < 110, axis=1)
            if clipped_bottom:
                full_top = y
                while full_top > 230 and fractions[full_top - 1] > .70:
                    full_top -= 1
                cap = fractions[max(230, full_top - 6):full_top + 2]
                rounded = np.count_nonzero((cap > .10) & (cap < .80)) >= 2
                rounding = 'TOP'
            else:
                base = fractions[max(y, y + h - 6):y + h]
                rounded = len(base) >= 4 and np.median(base[-3:]) < .80 and base[0] - base[-1] > .15
                rounding = 'BASE'
            if not rounded:
                continue
            u = float(np.median([(a + b) / 2 for a, b in rows]))
            # The single inverse model is deliberately approximate; keeping
            # its near extrapolation avoids silently ignoring a closer object.
            gap = CALIBRATION['empirical_K_mm_px'] / width
            right = -100. + 50. * (u - 319.5) / width
            range_state = ('FAR_UNCALIBRATED' if width < 78 else
                           'NEAR_UNCALIBRATED' if width > 122 else 'INITIAL_INTERVAL')
            found.append({'bbox': [x, top, w, visible_height], 'width_px': round(width, 2),
                          'center_u_px': round(u, 2), 'front_gap_mm': round(gap, 1),
                          'body_right_mm': round(right, 1), 'bottom_clipped': clipped_bottom,
                          'side_clipped': clipped_side, 'range_state': range_state,
                          'extrapolated': range_state != 'INITIAL_INTERVAL',
                          'rounding_evidence': rounding, 'partial_top': partial_top,
                          'contrast': round(float(np.median(contrast)), 1)})
            accepted_components.add(index)
        for index, values in enumerate(stats[1:], 1):
            if index in accepted_components or int(values[4]) < 600:
                continue
            candidate=ObstacleDetector.split_foot(gray,labels,index,values)
            if candidate is not None:
                found.append(candidate)
        return sorted(found, key=lambda item: item['front_gap_mm'])

    def update(self, bgr, frame_key, frame_perf_s):
        start = time.perf_counter()
        # Re-publishing a frame, including a QR service idle tick, is not a new
        # confirming frame. A repeated key never advances the stable count.
        if frame_key == self.last_key:
            return self.report
        changed_epoch = self.last_key is not None and frame_key[0] != self.last_key[0]
        non_monotonic = (self.last_key is not None and not changed_epoch and
                         (frame_key[1] <= self.last_key[1] or frame_key[2] < self.last_key[2]))
        if changed_epoch or non_monotonic or self.last_time is not None and not 0 < frame_perf_s - self.last_time <= .35:
            self.previous = None
            self.stable = 0
        self.last_key, self.last_time = frame_key, frame_perf_s
        candidates = [] if non_monotonic else self.candidates(bgr)
        usable = [item for item in candidates if self.usable_geometry(item)]
        # A nearer cropped/sideways object must not hide a complete obstacle
        # in the vehicle corridor. Unusable candidates remain visible.
        selected = usable[0] if usable else candidates[0] if candidates else None
        if selected is not None:
            old = self.previous
            matched = old is not None and abs(old['center_u_px'] - selected['center_u_px']) < max(24., selected['width_px'] * .5)
            matched = matched and .65 < selected['width_px'] / old['width_px'] < 1.6
            self.stable = self.stable + 1 if matched else 1
        else:
            self.stable = 0
        self.previous = selected
        confirmed = selected is not None and self.stable >= 3
        # The 50px trigger is extrapolated beyond the two measured anchors.
        # Never invent a lateral position for an object cropped by either side.
        reportable = bool(confirmed and self.usable_geometry(selected))
        self.report = {'status': 'CONFIRMED' if confirmed else 'ACQUIRE' if selected else 'NO_CYLINDER',
                       'confirmed_frames': self.stable, 'confirmed': bool(confirmed),
                       'reportable': reportable, 'selected': selected, 'candidates': candidates,
                       'frame_key': list(frame_key), 'frame_pc_perf_s': frame_perf_s,
                       'calibration': CALIBRATION, 'detect_ms': (time.perf_counter() - start) * 1000}
        return self.report

    def draw(self, image):
        report = self.report
        if image is None or not report:
            return
        for item in report['candidates']:
            x, y, w, h = item['bbox']
            selected = item is report['selected']
            color = (40, 220, 80) if selected and report['confirmed'] else (0, 180, 255)
            cv2.rectangle(image, (x, y), (x + w - 1, y + h - 1), color, 2)
            label = 'CYL %.0fpx / gap~%.0fmm / R%+.0fmm' % (item['width_px'], item['front_gap_mm'], item['body_right_mm'])
            cv2.putText(image, label, (max(4, min(x, 280)), max(73, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, .43, color, 1, cv2.LINE_AA)
        cv2.putText(image, 'OBS ' + report['status'] + ' %d/3' % min(3, report['confirmed_frames']),
                    (10, 77), cv2.FONT_HERSHEY_SIMPLEX, .50, (80, 235, 255), 1, cv2.LINE_AA)
