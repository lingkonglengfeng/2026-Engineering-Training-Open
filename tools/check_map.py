"""Offline replica of the firmware's 48x48 field map, to verify geometry,
car-body inflation, connectivity and A* routes **without hardware**.

Keep this in sync with Core/Src/grid_nav.c (GridNav_BuildFieldMap /
GridNav_CellPassable). Field numbers come from the competition rules:
  2400x2400 field, cross corridor 400 wide on axes X=1200/Y=1200,
  four 450x450 forbidden (yellow) blocks, outer ring band 550 wide,
  two 300x300 start/stop zones (top-right / bottom-right),
  car footprint 280x280.
Coordinate frame: origin = field left-bottom, x right, y UP, unit mm.
Internal cell row cy increases downward (cy = 47 - floor(y/50)) so that the
firmware's verified "dy>0 = forward" direction chain stays valid.

`--mapspec <path>` additionally dumps that same geometry as JSON, which is the
single source of truth for the MATLAB simulator (see sim/). Only *semantic*
facts are emitted (field rectangles, zones, pads, lanes, A* cost model) - never
the derived `kind` or `walkable` grids - so the MATLAB side has to redo the
inflation and A* itself and can then be cross-checked against this script.
"""

import argparse
import json

SIZE = 48
CELL = 50
CAR = 280
FIELD = 2400

ROAD, RING, BLOCKED, START, WORK, PAD = 0, 1, 2, 3, 4, 5

# named waypoints, same table as grid_nav.c. The work-zone points are APPROACH
# points in front of each pad, not the pad centre: the pad itself is NO-CONTACT,
# so the closest a car centre may get is 140 mm from its edge, and these sit on
# the centre line of that free lane.
WAYPOINTS = {
    "center": (1200, 1200),
    "raw": (1200, 2050),
    "rough": (1200, 350),
    "rough1": (1050, 350), "rough2": (1200, 350), "rough3": (1350, 350),
    "temp": (350, 1200),
    "temp1": (350, 1350), "temp2": (350, 1200), "temp3": (350, 1050),
    "qr": (2125, 1130),
    "h1": (2250, 150), "h2": (2250, 2250),
    "exit1": (2125, 450), "exit2": (2125, 1950),
}

# physical work tables / QR board: the car body must never touch these.
# Same rectangles as the firmware's GRID_CELL_PAD overlay.
PADS = (
    (1000, 1400, 2250, 2400),   # raw-material table (top edge centre, 150 deep)
    (1000, 1400, 0, 150),       # rough-machining table (bottom edge centre)
    (0, 150, 1050, 1350),       # temporary-storage table (left edge centre, 150 wide)
    (2350, 2400, 1100, 1300),   # QR board (right edge centre, 50 wide)
)


def in_range(v, lo, hi):
    return lo <= v < hi


def build_kind():
    kind = [[BLOCKED] * SIZE for _ in range(SIZE)]
    for cy in range(SIZE):
        for cx in range(SIZE):
            x = cx * CELL + CELL // 2
            y = (SIZE - 1 - cy) * CELL + CELL // 2
            cross = in_range(x, 1000, 1400) or in_range(y, 1000, 1400)
            ring = (x < 550) or (x >= 1850) or (y < 550) or (y >= 1850)
            kind[cy][cx] = ROAD if cross else (RING if ring else BLOCKED)

            start = (in_range(x, 2100, 2400) and in_range(y, 2100, 2400)) or \
                    (in_range(x, 2100, 2400) and in_range(y, 0, 300))
            if start:
                kind[cy][cx] = START
            elif kind[cy][cx] != BLOCKED:
                if in_range(x, 1000, 1400) and in_range(y, 1850, 2400):
                    kind[cy][cx] = WORK      # raw-material functional area
                elif in_range(x, 1000, 1400) and in_range(y, 0, 300):
                    kind[cy][cx] = WORK      # rough-machining area
                elif in_range(x, 0, 300) and in_range(y, 850, 1550):
                    kind[cy][cx] = WORK      # temp-storage area
                elif in_range(x, 2100, 2400) and in_range(y, 1000, 1400):
                    kind[cy][cx] = WORK      # QR area

                if any(in_range(x, x0, x1) and in_range(y, y0, y1)
                       for (x0, x1, y0, y1) in PADS):
                    kind[cy][cx] = PAD       # NO-CONTACT table itself
    return kind


def build_walkable(kind, obstacles=frozenset()):
    walk = [[False] * SIZE for _ in range(SIZE)]
    for cy in range(SIZE):
        for cx in range(SIZE):
            if kind[cy][cx] in (BLOCKED, PAD) or (cx, cy) in obstacles:
                continue
            x0 = cx * CELL + CELL // 2 - CAR // 2
            x1 = x0 + CAR
            y0 = cy * CELL + CELL // 2 - CAR // 2
            y1 = y0 + CAR
            if x0 < 0 or y0 < 0 or x1 > FIELD or y1 > FIELD:
                continue
            ok = True
            for yy in range(y0 // CELL, y1 // CELL + 1):
                for xx in range(x0 // CELL, x1 // CELL + 1):
                    if kind[yy][xx] in (BLOCKED, PAD) or (xx, yy) in obstacles:
                        ok = False
                        break
                if not ok:
                    break
            walk[cy][cx] = ok
    return walk


def mm_to_cell(x, y):
    cx = min(SIZE - 1, x // CELL)
    cy = SIZE - 1 - min(SIZE - 1, y // CELL)
    return cx, cy


def cell_to_mm(cx, cy):
    return cx * CELL + CELL // 2, (SIZE - 1 - cy) * CELL + CELL // 2


def snap(walk, cell):
    """replica of GridNav_SnapToWalkable: nearest passable cell by ring search"""
    cx, cy = cell
    if walk[cy][cx]:
        return cell
    for r in range(1, 9):
        best, bestd = None, 1 << 30
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < SIZE and 0 <= ny < SIZE and walk[ny][nx]:
                    d = dx * dx + dy * dy
                    if d < bestd:
                        bestd, best = d, (nx, ny)
        if best:
            return best
    return cell


def astar(walk, start, goal):
    import heapq
    start = snap(walk, start)
    goal = snap(walk, goal)
    if not walk[start[1]][start[0]] or not walk[goal[1]][goal[0]]:
        return None
    openq = [(0, start)]
    came = {start: None}
    cost = {start: 0}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            return path[::-1]
        cx, cy = cur
        for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0)):
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < SIZE and 0 <= ny < SIZE) or not walk[ny][nx]:
                continue
            prev = came.get(cur)
            if prev is None:
                turn = 0
            else:
                pdx, pdy = cur[0] - prev[0], cur[1] - prev[1]
                if (pdx, pdy) == (dx, dy):
                    turn = 0                       # straight on
                elif (pdx, pdy) == (-dx, -dy):
                    turn = TURN180_COST            # U-turn: twice the time of a corner
                else:
                    turn = TURN90_COST             # 90 deg corner
            # lane-centre preference: mirror GRID_NAV_LANE_COST_PER_CELL
            g = cost[cur] + MOVE_COST + turn + LANE_COST_PER_CELL * lane_deviation_cells(nx, ny)
            if (nx, ny) not in cost or g < cost[(nx, ny)]:
                cost[(nx, ny)] = g
                came[(nx, ny)] = cur
                h = abs(nx - goal[0]) + abs(ny - goal[1])
                heapq.heappush(openq, (g + h, (nx, ny)))
    return None


def segments(path):
    """merge same-direction cells; returns [(letter, cells, mm)]"""
    out = []
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        letter = "R" if dx > 0 else "L" if dx < 0 else "D" if dy > 0 else "U"
        if out and out[-1][0] == letter:
            out[-1][1] += 1
        else:
            out.append([letter, 1])
    return [(l, c, c * CELL) for l, c in out]


STRAFE_MAX_CELLS = 3      # ZDT_NAV_STRAFE_MAX_CELLS
LANE_COST_PER_CELL = 3    # GRID_NAV_LANE_COST_PER_CELL — soft "keep to the middle" cost
# Mirror the firmware's cost model EXACTLY (2026-09-13). It used to be 1 here while
# the firmware uses 10, which made this script weight turns (and the lane penalty)
# 10x too heavily relative to moves - so the routes it reported were not the routes
# the car would drive: it under-reported turns and over-hugged the lane centre.
MOVE_COST = 10            # GRID_NAV_MOVE_COST
# Cost model now mirrors the firmware's *time-based* turn cost: the executor calls
# GridNav_SetCostModel() before planning with
#     turn90_units = MOVE_COST * (ZDT_TURN_MS_PER_90_DEG  / ms_to_drive_one_cell)
#     turn180_units = MOVE_COST * (ZDT_TURN_MS_PER_180_DEG / ms_to_drive_one_cell)
# so a U-turn costs twice a corner and the weighting follows the current spd.
SPEED_SCALE = 0.20        # the field setting (`&spd 0.2`)
CELL_MS_AT_SPEED = {      # ms to drive one 50 mm cell, per speed scale
    0.20: 235.0, 0.50: 94.0,
}
TURN_MS_PER_90_DEG = 1000.0     # NAV_TURN90_DEFAULT_MS  (field: ~1 s)
TURN_MS_PER_180_DEG = 2000.0    # NAV_TURN180_DEFAULT_MS (field: ~2 s)
_cell_ms = CELL_MS_AT_SPEED.get(SPEED_SCALE, 235.0)
TURN90_COST = max(MOVE_COST, min(400, round(MOVE_COST * TURN_MS_PER_90_DEG / _cell_ms)))
TURN180_COST = max(TURN90_COST, min(800, round(MOVE_COST * TURN_MS_PER_180_DEG / _cell_ms)))
LANES = (275, 1200, 2125)  # GRID_NAV_LANE_NEAR/MID/FAR_MM
# NOTE (2026-09-13): the start zones used to be lockable ("after &leave the left zone
# must not be touched again"), which removed every cell whose inflated body overlapped
# the zone. The field has since confirmed the zones ARE drivable, so the whole lock -
# and its mirror here - is gone. Only physical constraints remain: field edge, the four
# 450x450 yellow blocks and the four work-table pads (PAD).


def lane_mm(mm):
    return min(LANES, key=lambda lane: abs(lane - mm))


def lane_deviation_cells(cx, cy):
    """Mirror of GridNav_LaneDeviationCells()."""
    x = cx * CELL + CELL // 2
    y = (SIZE - 1 - cy) * CELL + CELL // 2
    return min(abs(x - lane_mm(x)), abs(y - lane_mm(y))) // CELL


def plan_maneuvers(segs, heading=None):
    """Replicate the executor's turn/strafe decision, to show what the car will
    actually do and how many in-place turns the strafe-jog rule saves.

    Firmware rule (ZDT_Nav_SegmentStrafeSign):
      a segment shorter than or equal to STRAFE_MAX_CELLS that is PERPENDICULAR
      to the current heading is driven as a pure strafe - the heading does not
      change, so no turn is needed before or after it. Longer perpendicular
      segments and reversals (U-turns) still turn in place.
    Returns (turns, strafes, description list).
    """
    turns = 0
    turns90 = 0
    turns180 = 0
    strafes = 0
    actions = []
    OPPOSITE = {"U": "D", "D": "U", "L": "R", "R": "L"}
    for letter, cells, mm in segs:
        if heading is None:
            # first segment: the car turns to face it (unless it is a tiny jog,
            # which the firmware also strafes - just show it as a strafe)
            if cells <= STRAFE_MAX_CELLS and letter not in "UD":
                strafes += 1
                actions.append("strafe %s%d" % (letter, mm))
                continue
            heading = letter
            turns += 1
            turns90 += 1          # the initial alignment is a turn, not a U-turn
            actions.append("turn+go %s%d" % (letter, mm))
            continue
        if letter == heading:
            actions.append("go %s%d" % (letter, mm))
        elif cells <= STRAFE_MAX_CELLS:
            strafes += 1
            actions.append("strafe %s%d (no turn)" % (letter, mm))
        else:
            turns += 1
            if OPPOSITE[heading] == letter:
                turns180 += 1
                actions.append("TURN180+go %s%d" % (letter, mm))
            else:
                turns90 += 1
                actions.append("turn+go %s%d" % (letter, mm))
            heading = letter
    return turns, strafes, actions, turns90, turns180


def leg_time_s(cells, turns90, turns180):
    """Estimated time of a leg in seconds, using the same model the planner uses."""
    return (cells * MOVE_COST * _cell_ms / MOVE_COST
            + turns90 * TURN_MS_PER_90_DEG + turns180 * TURN_MS_PER_180_DEG) / 1000.0


FWD, BACK, LEFT, RIGHT, TURN = "FWD", "BACK", "LEFT", "RIGHT", "TURN180"

ZONE_X, LANE_X = 2250, 2125
LEAVE_MM, STRAFE_MM = 300, 125
ZONE_Y = {1: 150, 2: 2250}


def cross_steps(src, dst):
    """Replicate ZDT_Script_StartCross() and return (steps, final map position).

    Each step is (action, mm, map_dx, map_dy) exactly as the firmware builds it.
    The nose binding decides which BODY action yields which MAP direction, so a
    flipped entry here would drive the car off the mat - hence this check.

    The straight run is between the two EXIT points, not between the two zone
    centres: leaving already moves 300 mm into the field and entering moves the
    other 300, so the lane run is |dz| - 2*300 = 1500 mm (an earlier version used
    2100 and the car would have driven 600 mm into the far wall).
    """
    from_exit = ZONE_Y[src] + (300 if src == 1 else -300)
    to_exit = ZONE_Y[dst] - (300 if dst == 2 else -300)
    run = to_exit - from_exit
    if src == 1:                                  # nose toward map +y
        leave_fwd, leave_strafe = FWD, LEFT      # LEFT = map -x
        run_action = FWD if run >= 0 else BACK
    else:                                         # nose toward map -y
        leave_fwd, leave_strafe = FWD, RIGHT     # RIGHT = map -x
        run_action = FWD if run <= 0 else BACK
    if dst == 2:                                  # after the 180 turn: nose -y
        enter_strafe, enter_back = LEFT, BACK    # LEFT = map +x, BACK = map +y
    else:                                         # after the turn: nose +y
        enter_strafe, enter_back = RIGHT, BACK   # RIGHT = map +x, BACK = map -y

    steps = [
        (leave_fwd, LEAVE_MM, 0, 300 if src == 1 else -300),
        (leave_strafe, STRAFE_MM, -125, 0),
        (run_action, abs(run), 0, run),
        (TURN, 0, 0, 0),
        (enter_strafe, STRAFE_MM, 125, 0),
        (enter_back, LEAVE_MM, 0, 300 if dst == 2 else -300),
    ]
    x, y = ZONE_X, ZONE_Y[src]
    for _, _, dx, dy in steps:
        x, y = x + dx, y + dy
    return steps, (x, y)


def export_mapspec(path):
    """Write the field geometry + cost model as JSON (the MATLAB data contract).

    Deliberately exports raw rectangles/constants instead of the built grids:
    the consumer must rebuild `kind` / `walkable` and can then be diffed against
    this script's own baseline numbers.
    """
    spec = {
        "format": "field-mapspec",
        "version": 1,
        "generated_by": "tools/check_map.py",
        "note": "Geometry is the frozen project口径; cy increases DOWNWARD internally.",
        "field_mm": FIELD,
        "cell_mm": CELL,
        "size": SIZE,
        "car_mm": CAR,
        "car_half_cells": (CAR // 2 + CELL - 1) // CELL,
        "field_kind": {
            "mid_mm": 1200,          # GRID_NAV_LANE_MID_MM: the cross axis
            "cross_half_mm": 200,    # GRID_NAV_CROSS_HALF: 400mm corridor / 2
            "block_lo_mm": 550,      # GRID_NAV_BLOCK_LO
            "block_hi_mm": 1850,     # GRID_NAV_BLOCK_HI = FIELD - BAND
            "band_mm": 550,          # GRID_NAV_BAND_MM (outer ring band)
            # RULE RISK, not settled geometry. The rules (p3) say only
            # "行车道为灰色，中间十字灰色行车道宽度为 400mm" and "其余区域为亚光白色等
            # 浅色底色" - there is NO stated "outer ring band", and the rule
            # book map shows no differently-coloured band (the band is the SAME
            # grey as the rest of the floor, so it is not visually separable).
            # 550 is DERIVED: (2400 - 400 - 2*450)/2, not a rule value.
            # Leaving the lane ENDS THE RUN, so whether the band is drivable is
            # a rules-interpretation question with a large consequence.
            # true  = current firmware behaviour (the band is drivable)
            # false = strict reading (only the 400mm cross + start zones)
            # test_ring_scope.m quantifies what each choice costs.
            "ring_drivable": True,
        },
        "area_types": {
            "start": [[2100, 2400, 0, 300], [2100, 2400, 2100, 2400]],
            "work": [[1000, 1400, 1850, 2400], [1000, 1400, 0, 300],
                     [0, 300, 850, 1550], [2100, 2400, 1000, 1400]],
        },
        "pads": [list(r) for r in PADS],
        "zones": {
            "1": {"center_mm": [2250, 150], "rect_mm": [2100, 2400, 0, 300],
                  "nose_map": [0, 1], "exit_mm": [2125, 450]},
            "2": {"center_mm": [2250, 2250], "rect_mm": [2100, 2400, 2100, 2400],
                  "nose_map": [0, -1], "exit_mm": [2125, 1950]},
            "leave_mm": LEAVE_MM,
            "strafe_mm": STRAFE_MM,
            "left_action": "strafe_left",
            "lane_x_mm": LANE_X,
        },
        "waypoints_mm": {k: list(v) for k, v in WAYPOINTS.items()},
        "lanes_mm": {"near": 275, "mid": 1200, "far": 2125},
        "astar": {"move_cost": MOVE_COST, "turn90_cost": TURN90_COST,
                   "turn180_cost": TURN180_COST, "speed_scale": SPEED_SCALE,
                  "lane_cost_per_cell": LANE_COST_PER_CELL},
        "executor": {"strafe_max_cells": STRAFE_MAX_CELLS},
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(spec, fh, ensure_ascii=False, indent=2, sort_keys=True)
        fh.write("\n")
    return spec


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="offline replica of the firmware field map + route check")
    parser.add_argument("--mapspec", metavar="PATH",
                        help="also write the field geometry JSON used by sim/")
    args = parser.parse_args(argv)

    kind = build_kind()
    walk = build_walkable(kind)
    total_walk = sum(sum(1 for v in row if v) for row in walk)

    from collections import deque
    start = snap(walk, mm_to_cell(*WAYPOINTS["h1"]))
    seen = [[False] * SIZE for _ in range(SIZE)]
    seen[start[1]][start[0]] = True
    q = deque([start])
    while q:
        cx, cy = q.popleft()
        for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < SIZE and 0 <= ny < SIZE and walk[ny][nx] and not seen[ny][nx]:
                seen[ny][nx] = True
                q.append((nx, ny))
    reach = sum(sum(1 for v in row if v) for row in seen)

    print("=" * 66)
    print("FIELD MAP CHECK (48x48 @50mm, car 280mm -> millimetre footprint inflation)")
    print("=" * 66)
    print("walkable cells      : %d / %d (%.0f%%)" % (total_walk, SIZE * SIZE, 100.0 * total_walk / (SIZE * SIZE)))
    print("reachable from H1   : %d  -> %s" % (reach, "ALL CONNECTED" if reach == total_walk else "!! ISOLATED ISLANDS"))
    print()
    print("waypoint            mm              cell      walkable  reachable")
    for name, (x, y) in WAYPOINTS.items():
        cx, cy = mm_to_cell(x, y)
        print("  %-8s (%4d,%4d)  (%2d,%2d)   %-8s  %s" %
              (name, x, y, cx, cy, walk[cy][cx], seen[cy][cx]))
    print()
    legs = [("h1", "exit1"), ("exit1", "raw"), ("raw", "rough2"), ("rough2", "temp2"),
            ("temp2", "qr"), ("qr", "h2"), ("h2", "exit2"), ("exit2", "rough2"), ("h2", "h1"),
            ("temp", "rough"), ("rough", "temp")]
    for a, b in legs:
        pa = mm_to_cell(*WAYPOINTS[a])
        pb = mm_to_cell(*WAYPOINTS[b])
        path = astar(walk, pa, pb)
        if path is None:
            print("  %-6s -> %-6s  NO PATH" % (a, b))
            continue
        segs = segments(path)
        turns, strafes, actions, t90, t180 = plan_maneuvers(segs)
        print("  %-6s -> %-6s  %2d cells, %d legs, %d turns (90deg x%d, 180deg x%d), "
              "%d strafes, ~%.1fs: %s" %
              (a, b, len(path) - 1, len(segs), turns, t90, t180, strafes,
               leg_time_s(len(path) - 1, t90, t180), " | ".join(actions)))
    print()
    print("  turn cost model: move=%d/cell (%.0f ms at spd=%.2f), 90deg=%d (%.0f ms), "
          "180deg=%d (%.0f ms)" %
          (MOVE_COST, _cell_ms, SPEED_SCALE, TURN90_COST, TURN_MS_PER_90_DEG,
           TURN180_COST, TURN_MS_PER_180_DEG))
    print()
    print("&cross (start zone -> other start zone) end-position check:")
    cross_ok = True
    for src in (1, 2):
        dst = 2 if src == 1 else 1
        steps, end = cross_steps(src, dst)
        expected = (ZONE_X, ZONE_Y[dst])
        good = end == expected
        cross_ok = cross_ok and good
        print("  zone %d -> zone %d : ends at %s (expected %s) %s" %
              (src, dst, end, expected, "OK" if good else "!! WRONG"))
        print("      " + " | ".join("%s%d%s" % (a, mm, "" if (dx == 0 and dy == 0)
                                                else " map(%+d,%+d)" % (dx, dy))
                                    for a, mm, dx, dy in steps))
    if not cross_ok:
        print("  !! a &cross step table sends the car somewhere other than the "
              "destination zone centre")

    print()
    print("start zones are DRIVABLE (2026-09-13 field correction): the old NO-TOUCH lock")
    print("is gone, so both zones and the whole ring band are usable by A*:")
    for name, (x0, y0, x1, y1) in (("zone 1 (bottom-right)", (2100, 0, 2400, 300)),
                                   ("zone 2 (top-right)", (2100, 2100, 2400, 2400))):
        inside = sum(1 for cy in range(SIZE) for cx in range(SIZE)
                     if x0 <= cx * CELL + CELL // 2 < x1
                     and y0 <= (SIZE - 1 - cy) * CELL + CELL // 2 < y1 and walk[cy][cx])
        print("  %-20s %2d walkable cell centres inside the zone" % (name, inside))
    for name in ("exit1", "exit2", "h1", "h2"):
        cell = mm_to_cell(*WAYPOINTS[name])
        note = ""
        if not walk[cell[1]][cell[0]]:
            # whole-mm waypoints land on cell BOUNDARIES: the cell centre is 25 mm off,
            # so the 280 mm body pokes past the field edge and the planner snaps <=25 mm
            note = "  (waypoint sits on a cell boundary -> planner snaps <=25mm)"
        print("  %-6s cell (%2d,%2d) walkable=%s%s" %
              (name, cell[0], cell[1], walk[cell[1]][cell[0]], note))

    print()
    print("decimated 2:1 map (top = y up 2400).  # forbidden/PAD  o road/ring  S start  H1/H2 car")
    for cy in range(SIZE - 1, -1, -2):
        row = ""
        for cx in range(0, SIZE, 2):
            blk = any(kind[yy][xx] in (BLOCKED, PAD) for yy in (cy, max(0, cy - 1)) for xx in (cx, min(SIZE - 1, cx + 1)))
            car = (mm_to_cell(*WAYPOINTS["h1"]) == (cx, cy)) or (mm_to_cell(*WAYPOINTS["h2"]) == (cx, cy))
            row += "H" if car else ("#" if blk else "o")
        print("   ", row)

    if args.mapspec:
        export_mapspec(args.mapspec)
        print()
        print("mapspec written     : %s" % args.mapspec)


if __name__ == "__main__":
    main()
