"""Offline feasibility + timing analysis for the 90-degree ARC corner (圆弧过弯).

Field question (2026-09-13): "如果采用圆弧过弯的话是不是会更快" + "设置个参数：90 度圆弧
过弯需要的时间（时间越短半径越小），让我调整，以及速度".

This script answers both without touching the car:

1. GEOMETRY - for every 90-degree corner of every standard route, what is the largest
   arc radius the car can actually drive? The car is 300x300, so while it rotates its
   footprint sweeps a *rotating* square: at mid-arc that square needs 212 mm (the
   circumscribed radius), not the 150 mm the grid inflation assumes for an axis-aligned
   car. The test here is exact: an OBB (the rotated body) vs every forbidden AABB
   (the four 450x450 blocks, the four PAD tables/QR board, the locked start zone, the
   field edge), via the separating-axis theorem.
2. TIMING - given `&tarc <ms>` (arc time, shorter = smaller radius) and `&aspd <scale>`
   (arc body speed), the radius follows from R = 2*v*T/pi. Then the wheels must supply
   v + 300*omega (the mecanum spin term, omega = v/R), which is capped by the motor
   (250 RPM = 1065 mm/s), and the chassis ramp makes the body accelerate slower during
   the arc by the same mix factor. Finally the whole route is timed twice (stop-turn-go
   vs arc) with the same trapezoid model the firmware uses.

Keep the numbers in sync with Core/Src/zdt_motor.c / nav_params.c:
  CHASSIS_MAX_WHEEL_RPM 250, ZDT_MILEAGE_MM_PER_REV 255.694226,
  CHASSIS_ACCEL_RPM_PER_S 600, CHASSIS_DECEL_RPM_PER_S 900,
  NAV_TURN90_DEFAULT_MS 1000, NAV_TURN180_DEFAULT_MS 2000.
"""

import argparse
import math
import sys

import check_map as cm

# ---------------------------------------------------------------- drive model
MAX_WHEEL_RPM = 250.0
MM_PER_REV = 255.694226
MM_S_PER_RPM = MM_PER_REV / 60.0                 # 4.2616 mm/s per RPM
V_MAX_MM_S = MAX_WHEEL_RPM * MM_S_PER_RPM        # 1065.4 mm/s
ACCEL_RPM_PER_S = 600.0
DECEL_RPM_PER_S = 900.0
SPIN_ARM_MM = 300.0                              # half wheelbase + half track (150+150)

CAR = 300.0
HALF = CAR / 2.0
FIELD = 2400.0

BLOCK_LO, BLOCK_HI = 550.0, 1850.0
CROSS_LO, CROSS_HI = 1000.0, 1400.0
# The four 450x450 yellow blocks. Do NOT merge the two in the same column: the
# 400 mm cross corridor (y 1000..1400 / x 1000..1400) runs BETWEEN them and is free.
BLOCKS = (
    (BLOCK_LO, CROSS_LO, BLOCK_LO, CROSS_LO),
    (CROSS_HI, BLOCK_HI, BLOCK_LO, CROSS_LO),
    (BLOCK_LO, CROSS_LO, CROSS_HI, BLOCK_HI),
    (CROSS_HI, BLOCK_HI, CROSS_HI, BLOCK_HI),
)
FORBIDDEN = BLOCKS + tuple((float(a), float(b), float(c), float(d))
                           for a, b, c, d in cm.PADS)


def check_firmware_rects():
    """The firmware now owns the same rectangles (grid_nav.c, for the rotated-body
    collision test). Parse them back out and compare, so the two can never drift -
    a silent mismatch here would let the car arc into a table."""
    import os
    import re

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "Core", "Src", "grid_nav.c")
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    problems = []
    for name, expected in (("grid_nav_zone_block_rects", BLOCKS),
                           ("grid_nav_pad_rects", cm.PADS)):
        match = re.search(re.escape(name) + r"\s*\[4\]\[4\]\s*=\s*\{(.*?)\n\};",
                          text, re.S)
        if match is None:
            problems.append("%s not found in grid_nav.c" % name)
            continue
        rows = re.findall(r"\{\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\}",
                          match.group(1))
        got = [tuple(int(v) for v in row) for row in rows]
        want = [tuple(int(v) for v in row) for row in expected]
        if got != want:
            problems.append("%s mismatch\n      firmware: %s\n      checker : %s"
                            % (name, got, want))
    if problems:
        print()
        print("!! firmware arc-collision rectangles do not match this checker:")
        for item in problems:
            print("   " + item)
        return False
    print()
    print("firmware rectangles : grid_nav_zone_block_rects / grid_nav_pad_rects match"
          " this checker (4+4 rects)")
    return True


def plan_seconds(distance_mm, speed_mm_s, accel_mm_s2, decel_mm_s2):
    """Same trapezoid/triangle solution as ZDT_Nav_PlanSeconds()."""
    if speed_mm_s < 1.0:
        speed_mm_s = 1.0
    accel_mm = speed_mm_s * speed_mm_s / (2.0 * accel_mm_s2)
    decel_mm = speed_mm_s * speed_mm_s / (2.0 * decel_mm_s2)
    if distance_mm >= accel_mm + decel_mm:
        return (speed_mm_s / accel_mm_s2 + speed_mm_s / decel_mm_s2
                + (distance_mm - accel_mm - decel_mm) / speed_mm_s)
    peak = math.sqrt(2.0 * distance_mm / (1.0 / accel_mm_s2 + 1.0 / decel_mm_s2))
    return peak / accel_mm_s2 + peak / decel_mm_s2


STRAIGHT_ACCEL = ACCEL_RPM_PER_S * MM_S_PER_RPM
STRAIGHT_DECEL = DECEL_RPM_PER_S * MM_S_PER_RPM

# Mirror of the firmware's clamping constants (zdt_motor.c, `&tarc` block)
ARC_MIN_RADIUS_MM = 50.0
ARC_RADIUS_STEP_MM = 5.0
ARC_LEG_MARGIN_MM = 50.0


def arc_radius_used(r_req, corner, u, w, leg_cur_mm, leg_next_mm):
    """Firmware's ZDT_Nav_TryArmArc(): leg margin -> 5 mm steps -> min radius.

    Returns (R, delta_yaw_deg) or (0, 0) when the corner must keep the in-place turn.
    """
    r = min(r_req, min(leg_cur_mm, leg_next_mm) - ARC_LEG_MARGIN_MM)
    if r < ARC_MIN_RADIUS_MM:
        return 0.0, 0.0
    delta = 0.0
    while r >= ARC_MIN_RADIUS_MM:
        poses = arc_poses(corner, u, w, r, 9)        # ZDT_NAV_ARC_SAMPLES = 9
        if all(pose_free(x, y, yaw) for (x, y), yaw in poses):
            delta = arc_delta_yaw(corner, u, w, r)
            return r, delta
        r -= ARC_RADIUS_STEP_MM
    return 0.0, 0.0


def arc_delta_yaw(corner, u, w, radius):
    px, py = corner
    c = (px + radius * (w[0] - u[0]), py + radius * (w[1] - u[1]))
    s = (px - radius * u[0], py - radius * u[1])
    e = (px + radius * w[0], py + radius * w[1])
    a0 = math.atan2(s[1] - c[1], s[0] - c[0])
    a1 = math.atan2(e[1] - c[1], e[0] - c[0])
    sweep = ((a1 - a0 + math.pi) % (2.0 * math.pi)) - math.pi
    return math.degrees(sweep)


def arc_plan_ms(radius, v_mm_s, scale):
    """Firmware's ZDT_Nav_TryArmArc() timing: mix clamp + trapezoid on the arc."""
    mix = scale * (1.0 + SPIN_ARM_MM / radius)
    if mix < 1.0:
        mix = 1.0                      # chassis Mix only scales when > 1
    v_used = v_mm_s / mix
    accel = ACCEL_RPM_PER_S * MM_S_PER_RPM / mix
    decel = DECEL_RPM_PER_S * MM_S_PER_RPM / mix
    ms = plan_seconds((math.pi / 2.0) * radius, v_used, accel, decel) * 1000.0
    rpm = v_used * (1.0 + SPIN_ARM_MM / radius) / MM_S_PER_RPM
    return v_used, ms, rpm


# ------------------------------------------------------------------ collision
def obb_overlaps_aabb(cx, cy, yaw_deg, rect):
    """Exact OBB(300x300, rotated) vs AABB test by the separating-axis theorem."""
    yaw = math.radians(yaw_deg)
    ax = (math.cos(yaw), math.sin(yaw))            # body long axis
    ay = (-math.sin(yaw), math.cos(yaw))           # body short axis
    x0, x1, y0, y1 = rect
    rx = (x0 + x1) / 2.0
    ry = (y0 + y1) / 2.0
    hx = (x1 - x0) / 2.0
    hy = (y1 - y0) / 2.0
    dx = cx - rx
    dy = cy - ry
    axes = ((1.0, 0.0), (0.0, 1.0), ax, ay)
    for ux, uy in axes:
        gap = abs(dx * ux + dy * uy)
        ra = hx * abs(ux) + hy * abs(uy)
        rb = HALF * (abs(ax[0] * ux + ax[1] * uy) + abs(ay[0] * ux + ay[1] * uy))
        if gap > ra + rb:
            return False
    return True


def pose_free(cx, cy, yaw_deg):
    for rect in FORBIDDEN:
        if obb_overlaps_aabb(cx, cy, yaw_deg, rect):
            return False
    yaw = math.radians(yaw_deg)
    ax = (math.cos(yaw), math.sin(yaw))
    ay = (-math.sin(yaw), math.cos(yaw))
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            px = cx + HALF * (sx * ax[0] + sy * ay[0])
            py = cy + HALF * (sx * ax[1] + sy * ay[1])
            if px < 0.0 or py < 0.0 or px > FIELD or py > FIELD:
                return False
    return True


def arc_poses(corner, u, w, radius, samples=17):
    """(position, yaw) along the quarter circle that replaces the in-place turn.

    Geometry (verified against the tangency conditions, both turn directions):
      start point S = corner - R*u   (R before the corner, on the incoming lane)
      end   point E = corner + R*w   (R after the corner, on the outgoing lane)
      centre      C = corner + R*(w - u)
    The turn direction is *implied* by w-u, so no separate CW/CCW branch is needed;
    the sweep angle comes out as +/-90 deg from the two radius angles. The heading
    rotates with the path (a car-like arc), which is what makes the body sweep the
    full 212 mm circumscribed radius at mid-arc.
    """
    px, py = corner
    c = (px + radius * (w[0] - u[0]), py + radius * (w[1] - u[1]))
    s = (px - radius * u[0], py - radius * u[1])
    e = (px + radius * w[0], py + radius * w[1])
    a0 = math.atan2(s[1] - c[1], s[0] - c[0])
    a1 = math.atan2(e[1] - c[1], e[0] - c[0])
    sweep = ((a1 - a0 + math.pi) % (2.0 * math.pi)) - math.pi      # = +/- pi/2
    yaw0 = math.degrees(math.atan2(u[1], u[0]))
    out = []
    for i in range(samples):
        t = i / float(samples - 1)
        a = a0 + sweep * t
        out.append(((c[0] + radius * math.cos(a), c[1] + radius * math.sin(a)),
                    yaw0 + math.degrees(sweep) * t))
    return out


def max_radius(corner, u, w, r_hi=600.0, step=5.0):
    """Largest R (mm) whose whole swept footprint stays clear; 0 = cannot arc."""
    best = 0.0
    r = step
    while r <= r_hi:
        if all(pose_free(x, y, yaw) for (x, y), yaw in arc_poses(corner, u, w, r)):
            best = r
        elif r > 60.0:
            # not monotone in theory (a bigger arc can leave a tight spot), but in
            # this field it is: stop after the first clear run breaks
            pass
        r += step
    return best


# --------------------------------------------------------------- route walker
DIRS = {"R": (1.0, 0.0), "L": (-1.0, 0.0), "U": (0.0, 1.0), "D": (0.0, -1.0)}
OPPOSITE = {"U": "D", "D": "U", "L": "R", "R": "L"}


def corners_of(segs, start_mm, heading=None):
    """Replicate the executor's leg/corner structure and return corners in mm.

    A corner exists where the executor would turn in place: a direction change of
    90 degrees between two legs that are both longer than the strafe-jog limit.
    Each entry is (corner_mm, u, w, leg_mm, leg_index) where leg_index is the index
    of the leg that ENDS at that corner (the arc eats R off that leg and R off the next).
    """
    out = []
    x, y = start_mm
    for index, (letter, cells, mm) in enumerate(segs):
        if heading is None:
            heading = letter
            x, y = x + mm * DIRS[letter][0], y + mm * DIRS[letter][1]
            continue
        if letter == heading:
            x, y = x + mm * DIRS[letter][0], y + mm * DIRS[letter][1]
            continue
        if cells <= cm.STRAFE_MAX_CELLS:
            # pure strafe jog: no turn, heading unchanged
            x, y = x + mm * DIRS[letter][0], y + mm * DIRS[letter][1]
            continue
        out.append(((x, y), DIRS[heading], DIRS[letter], abs(mm), index - 1))
        heading = letter
        x, y = x + mm * DIRS[letter][0], y + mm * DIRS[letter][1]
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="90-degree arc corner feasibility + timing")
    ap.add_argument("--tarc", type=float, default=1000.0, help="&tarc (ms) for 90 deg")
    ap.add_argument("--aspd-ratio", type=float, default=1.00,
                    help="&aspd (arc speed = spd x ratio, firmware default 1.00)")
    ap.add_argument("--spd", type=float, default=0.20, help="&spd (straight-line scale)")
    ap.add_argument("--turn90", type=float, default=1000.0, help="&t90 (ms) in-place turn")
    args = ap.parse_args(argv)

    v_straight = args.spd * V_MAX_MM_S
    v_arc = args.aspd_ratio * v_straight
    r_req = 2.0 * v_arc * (args.tarc / 1000.0) / math.pi

    print("=" * 78)
    print("90-DEG ARC CORNER - feasibility & timing  (&tarc %.0f ms, &aspd %.2f, &spd %.2f)"
          % (args.tarc, args.aspd_ratio, args.spd))
    print("=" * 78)
    print("requested radius from the knob : R = 2*v*T/pi = %.0f mm  (v=%.0f mm/s)"
          % (r_req, v_arc))
    print("wheels needed at that R        : v*(1+300/R) = %.0f mm/s = %.0f RPM "
          "(cap %.0f)" % (v_arc * (1 + SPIN_ARM_MM / max(r_req, 1.0)),
                          v_arc * (1 + SPIN_ARM_MM / max(r_req, 1.0)) / MM_S_PER_RPM,
                          MAX_WHEEL_RPM))
    print("in-place 90 turn for reference : %.0f ms (nav param), wheels 120 RPM max, 0 mm"
          % args.turn90)
    print()

    kind = cm.build_kind()
    walk = cm.build_walkable(kind)
    legs = [("h1", "exit1"), ("exit1", "raw"), ("raw", "rough2"), ("rough2", "temp2"),
            ("temp2", "qr"), ("qr", "h2"), ("h2", "exit2"), ("exit2", "rough2"),
            ("h2", "h1"), ("temp", "rough"), ("rough", "temp")]

    print("%-6s -> %-6s | %-4s | %-30s | %s" %
          ("from", "to", "turn", "corner (mm) / direction", "arc: R, time, wheels"))
    total_now = total_arc = 0.0
    for a, b in legs:
        pa = cm.mm_to_cell(*cm.WAYPOINTS[a])
        pb = cm.mm_to_cell(*cm.WAYPOINTS[b])
        path = cm.astar(walk, pa, pb)
        if path is None:
            continue
        segs = cm.segments(path)
        corners = corners_of(segs, (float(cm.WAYPOINTS[a][0]), float(cm.WAYPOINTS[a][1])))
        lengths = [float(mm) for _, _, mm in segs]

        # walk the legs and attach an arc (or the in-place turn) to each corner
        # exactly like the executor: leg i loses R at its start and R at its end.
        trims = [0.0] * (len(lengths) + 1)   # trims[b] = mm cut at leg BOUNDARY b
        arc_ms = []
        rows = []
        for (corner, u, w, leg_len, idx) in corners:
            leg_cur = lengths[idx] if 0 <= idx < len(lengths) else 0.0
            leg_next = lengths[idx + 1] if 0 <= (idx + 1) < len(lengths) else 0.0
            r, delta = arc_radius_used(r_req, corner, u, w, leg_cur, leg_next)
            if r <= 0.0:
                rows.append((corner, u, w, 0.0, 0.0, 0.0, 0.0, False))
                arc_ms.append(None)
                continue
            v_used, ms, rpm = arc_plan_ms(r, v_arc, args.aspd_ratio * args.spd)
            # trims[] is indexed by LEG BOUNDARY: a corner at the end of leg idx is
            # boundary idx+1, and that boundary gives up R (only R!) from each of the
            # two legs that share it -- see the per-leg sum below.
            trims[idx + 1] += r
            rows.append((corner, u, w, r, delta, ms, rpm, True))
            arc_ms.append(ms / 1000.0)

        t_now = sum(plan_seconds(L, v_straight, STRAIGHT_ACCEL, STRAIGHT_DECEL)
                    for L in lengths) + len(corners) * (args.turn90 / 1000.0)
        t_arc = 0.0
        for i, L in enumerate(lengths):
            trimmed = L - trims[i] - trims[i + 1]
            if trimmed < 0.0:
                trimmed = 0.0
            t_arc += plan_seconds(trimmed, v_straight, STRAIGHT_ACCEL, STRAIGHT_DECEL)
        for entry in arc_ms:
            if entry is None:
                t_arc += args.turn90 / 1000.0     # no arc -> in-place turn stays
            else:
                t_arc += entry
        total_now += t_now
        total_arc += t_arc

        print("%-6s -> %-6s | %-4d |" % (a, b, len(corners)))
        for (corner, u, w, r, delta, ms, rpm, took) in rows:
            if took:
                print("%-6s    %-6s |      | (%4.0f,%4.0f) in(%+.0f,%+.0f) out(%+.0f,%+.0f)"
                      " | R=%.0fmm %+.0fdeg t=%.0fms outer=%.0frpm"
                      % ("", "", corner[0], corner[1], u[0], u[1], w[0], w[1],
                         r, delta, ms, rpm))
            else:
                print("%-6s    %-6s |      | (%4.0f,%4.0f) in(%+.0f,%+.0f) out(%+.0f,%+.0f)"
                      " | NO ARC (geometry/tight) -> turn in place"
                      % ("", "", corner[0], corner[1], u[0], u[1], w[0], w[1]))
        print("%-6s    %-6s |      | route total                    | %.1fs -> %.1fs (%+.1fs)"
              % ("", "", t_now, t_arc, t_arc - t_now))

    print()
    print("TOTAL over all routes: %.1fs -> %.1fs  (%+.1f s, %+.0f%%)" %
          (total_now, total_arc, total_arc - total_now,
           100.0 * (total_arc - total_now) / total_now))
    print()
    print("R knob table (aspd=%.2f, spd=%.2f) - the firmware clamps per corner, so a"
          " smaller R is not always faster:" % (args.aspd_ratio, args.spd))
    print("  %-8s %-9s %-11s %-11s %s" % ("t_arc", "R_req", "wheels RPM", "arc time", "note"))
    for t_ms in (400.0, 600.0, 800.0, 1000.0, 1500.0, 2000.0):
        r = 2.0 * v_arc * (t_ms / 1000.0) / math.pi
        rpm = (v_arc * (1.0 + SPIN_ARM_MM / r)) / MM_S_PER_RPM
        v_used, ms, _ = arc_plan_ms(r, v_arc, args.aspd_ratio * args.spd)
        note = "wheel-capped -> body slows" if v_used < (v_arc - 1.0) else "ok"
        print("  %-8.0f %-9.0f %-11.0f %-11.0f %s" % (t_ms, r, rpm, ms, note))
    return 0 if check_firmware_rects() else 1


if __name__ == "__main__":
    sys.exit(main())
