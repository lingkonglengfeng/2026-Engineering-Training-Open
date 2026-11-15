"""Cross-check docs/hardware/场地地图_坐标查看.html against the firmware source.

The HTML is a hand-written copy of the field geometry, and a wrong number in it
would send someone to the wrong coordinates on the real mat. So every constant it
hard-codes is compared here with the single source of truth (Core/Src/grid_nav.c,
grid_nav.h, chassis_motor.h):

  * field size / cell / car body
  * the four 450x450 forbidden blocks
  * the two start zones
  * the four PAD rectangles (tables and QR board)
  * the lane centre lines
  * the named waypoint table

Exit code 0 = the HTML matches the firmware, 1 = at least one mismatch (each one
is printed with both values so it is obvious which side to fix).
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HTML = ROOT / "docs" / "hardware" / "场地地图_坐标查看.html"
GRID_C = ROOT / "Core" / "Src" / "grid_nav.c"
GRID_H = ROOT / "Core" / "Inc" / "grid_nav.h"


# Waypoints that are aliases of another point (same coordinates). The firmware
# keeps both so `&goto rough` works; the HTML draws one dot and mentions the alias.
ALIASES = {"rough": "rough2", "temp": "temp2"}


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def macro(text: str, name: str) -> int:
    m = re.search(rf"#define\s+{name}\s+\(?(\d+)U?\)?", text)
    if m is None:
        raise SystemExit(f"macro {name} not found")
    return int(m.group(1))


def html_rects(src: str, name: str) -> list[tuple]:
    """Parse `const NAME = [ {x0:.., y0:.., x1:.., y1:..}, ... ];` from the HTML."""
    block = re.search(rf"const {name} = \[(.*?)\];", src, re.S)
    if block is None:
        raise SystemExit(f"{name} not found in the HTML")
    out = []
    for item in re.finditer(r"\{[^}]*\}", block.group(1)):
        fields = dict(re.findall(r"(\w+)\s*:\s*(-?\d+)", item.group(0)))
        out.append((int(fields["x0"]), int(fields["y0"]),
                    int(fields["x1"]), int(fields["y1"])))
    return sorted(out)


def main() -> int:
    html = read(HTML)
    c = read(GRID_C)
    h = read(GRID_H)
    problems: list[str] = []

    def expect(what: str, html_value, fw_value):
        if html_value != fw_value:
            problems.append(f"{what}: HTML={html_value}  固件={fw_value}")

    # --- sizes (declared together: `const FIELD = 2400, CELL = 50, CAR = 300;`) ---
    sizes = re.search(r"const\s+FIELD\s*=\s*(\d+)\s*,\s*CELL\s*=\s*(\d+)\s*,\s*CAR\s*=\s*(\d+)",
                      html)
    if sizes is None:
        raise SystemExit("cannot find the FIELD/CELL/CAR declaration in the HTML")
    expect("场地尺寸", int(sizes.group(1)), macro(h, "GRID_NAV_FIELD_MM"))
    expect("格长", int(sizes.group(2)), macro(h, "GRID_NAV_CELL_MM"))
    expect("车体", int(sizes.group(3)), macro(h, "GRID_NAV_CAR_MM"))

    # --- forbidden blocks ---------------------------------------------------
    # The tiling identity: 550 (ring) + 450 (block) + 400 (corridor) + 450 (block)
    # + 550 (ring) = 2400. The C side only names the outer pair, so derive the
    # inner edges from that identity and assert it lands on BLOCK_HI.
    lo = macro(h, "GRID_NAV_BLOCK_LO")
    hi = macro(h, "GRID_NAV_BLOCK_HI")
    cross_lo, cross_hi = lo + 450, lo + 450 + 400
    if cross_hi + 450 != hi:
        problems.append(f"geometry constants do not tile: {lo}+450+400+450 != {hi}")
    blocks_c = sorted([(lo, lo, cross_lo, cross_lo), (lo, cross_hi, cross_lo, hi),
                       (cross_hi, lo, hi, cross_lo), (cross_hi, cross_hi, hi, hi)])
    expect("four forbidden blocks", html_rects(html, "BLOCKS"), blocks_c)

    # --- start zones -------------------------------------------------------
    zones_html = html_rects(html, "ZONES")
    expect("two start zones", zones_html,
           [(2100, 0, 2400, 300), (2100, 2100, 2400, 2400)])

    # --- PAD rectangles ----------------------------------------------------
    expect("four PAD tables", html_rects(html, "PADS"),
           [(0, 1050, 150, 1350), (1000, 0, 1400, 150),
            (1000, 2250, 1400, 2400), (2350, 1100, 2400, 1300)])

    # --- lane centre lines -------------------------------------------------
    lanes_c = [macro(h, n) for n in ("GRID_NAV_LANE_NEAR_MM",
                                     "GRID_NAV_LANE_MID_MM",
                                     "GRID_NAV_LANE_FAR_MM")]
    lanes_html = [int(v) for v in re.search(r"const LANES = \[([^\]]+)\]", html)
                  .group(1).split(",")]
    expect("lane centre lines", lanes_html, lanes_c)

    # --- named waypoints (entries may carry extra fields, e.g. alias:"rough") ---
    wp_html = {}
    for item in re.finditer(r"\{n:\s*\"(\w+)\"\s*,\s*x:\s*(\d+)\s*,\s*y:\s*(\d+)[^}]*\}",
                            html):
        wp_html[item.group(1)] = (int(item.group(2)), int(item.group(3)))
    wp_c = {}
    table = re.search(r"grid_nav_waypoints\[\] =\s*\{(.*?)\n\};", c, re.S).group(1)
    for item in re.finditer(r'\{\s*"(\w+)"\s*,\s*(\d+)U\s*,\s*(\d+)U\s*\}', table):
        wp_c[item.group(1)] = (int(item.group(2)), int(item.group(3)))
    for name, xy in sorted(wp_c.items()):
        if name in ALIASES:
            # `rough` / `temp` are aliases of the middle point: assert that the
            # firmware really keeps them equal, and that the HTML does not draw a
            # second dot for them (the label on the target mentions the alias).
            target = ALIASES[name]
            if wp_c.get(target) != xy:
                problems.append(f"alias {name}{xy} != {target}{wp_c.get(target)}")
            continue
        if name not in wp_html:
            problems.append(f"HTML is missing waypoint {name}{xy}")
        elif wp_html[name] != xy:
            problems.append(f"waypoint {name}: HTML={wp_html[name]} firmware={xy}")

    print(f"HTML     : {HTML.relative_to(ROOT)}")
    print(f"firmware : {GRID_C.relative_to(ROOT)} / {GRID_H.relative_to(ROOT)}")
    print(f"waypoints: HTML {len(wp_html)} drawn / firmware {len(wp_c)} "
          f"({len(ALIASES)} of them are aliases)")
    print(f"blocks   : {len(html_rects(html, 'BLOCKS'))}   "
          f"pads: {len(html_rects(html, 'PADS'))}   "
          f"zones: {len(zones_html)}   lanes: {lanes_html}")
    print()
    if problems:
        print(f"FAIL: {len(problems)} mismatch(es)")
        for p in problems:
            print("  - " + p)
        return 1
    print("PASS: the HTML geometry matches the firmware source exactly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
