"""Sign round-trip check for the body/map direction mappings.

Why this exists: the firmware keeps a nose vector F in the INTERNAL frame (grid
rows increase downward, so internal_y = -map_y) and derives body commands with

    R = (-F.y, F.x)            (body-right in the internal frame)
    vx = d . F                 vy = d . R          (d is an INTERNAL displacement)

Anything that starts from a RULE-BOOK mm offset (y up) must therefore flip y
before dotting with R. The lane-centring helper did not, and because x is not
flipped the mistake was invisible whenever the correction axis was x - but when
the correction axis was y the sign inverted and the car strafed the WRONG WAY.
Field symptom: on the final approach to `raw` (driving along the top edge, which
needs a y correction) the car strafed backwards and crossed a line.

This script checks both directions for all four nose bindings and all four
in-plane offsets, and it also asserts that the OLD (buggy) formula fails - so the
check is known to be able to catch the bug it was written for.

Exit code 0 = every mapping round-trips with a consistent sign.
"""

from __future__ import annotations

import sys

# Nose bindings exactly as ZDT_Nav_SetNoseDir() defines them (INTERNAL frame).
NOSE_DIRS = {
    0: (0, 1),    # map +y (down in the internal frame)
    1: (0, -1),   # map -y
    2: (1, 0),    # map +x
    3: (-1, 0),   # map -x
}

# Rule-book mm offsets (y UP) of magnitude 100 mm.
OFFSET_X = ((100, 0), (-100, 0))
OFFSET_Y = ((0, 100), (0, -100))

# Only physically reachable combinations matter: the executor aligns the car to
# the segment direction first, so
#   * travelling along x  => nose is along +-x (dirs 2/3) => the lane correction
#     axis is y  (this is the case that used to be reversed)
#   * travelling along y  => nose is along +-y (dirs 0/1) => correction axis is x
REACHABLE = (
    (2, OFFSET_Y), (3, OFFSET_Y),     # nose +x / -x, correcting y
    (0, OFFSET_X), (1, OFFSET_X),     # nose +y / -y (internal), correcting x
)

DEADBAND = 40
MAGNITUDE = 100


def body_sign_fixed(map_dx, map_dy, nose):
    """Mirror of the FIXED ZDT_Nav_ComputeLaneSnap() sign computation."""
    di_x = map_dx
    di_y = -map_dy                       # map mm -> internal frame
    r_x, r_y = -nose[1], nose[0]
    return 1 if (di_x * r_x + di_y * r_y) > 0 else -1


def body_sign_old(map_dx, map_dy, nose):
    """The buggy version: rule-book delta dotted with an internal-frame R."""
    r_x, r_y = -nose[1], nose[0]
    return 1 if (map_dx * r_x + map_dy * r_y) > 0 else -1


def body_to_map(body_sign, nose):
    """Forward mapping: a pure lateral body command (vx=0, vy=sign) in map mm."""
    r_x, r_y = -nose[1], nose[0]
    vx, vy = 0.0, float(body_sign)
    int_x = vx * nose[0] + vy * r_x
    int_y = vx * nose[1] + vy * r_y
    return int_x, -int_y                 # internal -> map mm


def main() -> int:
    failures = []
    caught_old = 0
    checked = 0

    for name, offsets in REACHABLE:
        nose = NOSE_DIRS[name]
        for map_dx, map_dy in offsets:
            checked += 1
            for label, sign_fn in (("fixed", body_sign_fixed), ("old", body_sign_old)):
                sign = sign_fn(map_dx, map_dy, nose)
                mx, my = body_to_map(sign, nose)
                dot = mx * map_dx + my * map_dy
                ok = dot > 0
                if label == "fixed" and not ok:
                    failures.append(
                        "nose dir %d %s: offset (%+d,%+d) -> body sign %+d maps to "
                        "(%+.0f,%+.0f), the WRONG direction" %
                        (name, nose, map_dx, map_dy, sign, mx, my))
                if label == "old" and not ok:
                    caught_old += 1
            print("  nose %d %-8s offset (%+4d,%+4d): fixed=%+d old=%+d%s" %
                  (name, str(nose), map_dx, map_dy,
                   body_sign_fixed(map_dx, map_dy, nose),
                   body_sign_old(map_dx, map_dy, nose),
                   "" if body_sign_fixed(map_dx, map_dy, nose) ==
                          body_sign_old(map_dx, map_dy, nose)
                   else "   <-- old formula was REVERSED here"))

    print()
    print("checked %d reachable (nose, offset) combinations" % checked)
    print("the old buggy formula fails %d of them "
          "(expected 4: every y-offset on a nose that points along +-x)"
          % caught_old)

    if failures:
        print()
        print("FAIL: %d sign inconsistencies in the FIXED mapping" % len(failures))
        for item in failures:
            print("  - " + item)
        return 1
    if caught_old == 0:
        print()
        print("FAIL: the negative control never failed - this check cannot catch "
              "anything, fix the check")
        return 1

    print()
    print("PASS: every offset round-trips with a consistent sign, and the "
          "negative control reproduces the field bug")
    return 0


if __name__ == "__main__":
    sys.exit(main())
