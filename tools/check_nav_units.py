"""Recompute the navigation time model from the **actual firmware constants** and
print the `&nav` line the firmware should print, so a field log can be compared
without hardware.

Why this exists: the seconds/milliseconds boundary between
`ZDT_Nav_PlanSeconds()` (returns SECONDS) and everything downstream (wants
MILLISECONDS) was crossed once. The symptom was not a rounding difference but
`k=1023.964 (理论2ms)` in the boot log, and - much worse - `navreset` / an
invalid flash record would have installed `t1000 = 2.05 ms`, clamping every
segment to ZDT_NAV_DRIVE_MIN_MS so the car barely moves, with no error at all.

Run it after touching any of:
  CHASSIS_MAX_WHEEL_RPM / CHASSIS_ACCEL_RPM_PER_S / CHASSIS_DECEL_RPM_PER_S
  ZDT_MILEAGE_MM_PER_REV / ZDT_NAV_REF_SPEED_SCALE / ZDT_NAV_REF_DISTANCE_MM
and before trusting a distance complaint.

Exit code 0 = the reference time is inside the guard range the firmware accepts.
"""

from __future__ import annotations

import math
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CHASSIS_H = ROOT / "Core" / "Inc" / "chassis_motor.h"
ZDT_H = ROOT / "Core" / "Inc" / "zdt_motor.h"
ZDT_C = ROOT / "Core" / "Src" / "zdt_motor.c"
NAV_C = ROOT / "Core" / "Src" / "nav_params.c"


def macro(path: Path, name: str) -> float:
    match = re.search(rf"#define\s+{name}\s+\(?(-?[\d.]+)f?\)?",
                      path.read_text(encoding="utf-8"))
    if match is None:
        raise SystemExit(f"macro {name} not found in {path}")
    return float(match.group(1))


def main() -> int:
    mm_per_rev = macro(ZDT_H, "ZDT_MILEAGE_MM_PER_REV")
    max_rpm = macro(CHASSIS_H, "CHASSIS_MAX_WHEEL_RPM")
    accel_rpm = macro(CHASSIS_H, "CHASSIS_ACCEL_RPM_PER_S")
    decel_rpm = macro(CHASSIS_H, "CHASSIS_DECEL_RPM_PER_S")

    zdt_src = ZDT_C.read_text(encoding="utf-8")
    ref_scale = float(re.search(r"ZDT_NAV_REF_SPEED_SCALE\s+\(([\d.]+)f\)",
                                zdt_src).group(1))
    ref_dist = float(re.search(r"ZDT_NAV_REF_DISTANCE_MM\s+\(([\d.]+)f\)",
                               zdt_src).group(1))

    nav_src = NAV_C.read_text(encoding="utf-8")
    default_speed = struct.unpack("f", struct.pack("f", macro(NAV_C, "NAV_SPEED_DEFAULT")))[0]
    default_time = macro(NAV_C, "NAV_T1000_DEFAULT_MS")
    default_compensation = macro(NAV_C, "NAV_DISTANCE_COMP_DEFAULT")
    guard_min = macro(NAV_C, "NAV_REF_PLAN_MIN_MS")
    guard_max = macro(NAV_C, "NAV_REF_PLAN_MAX_MS")

    mm_s_per_rpm = mm_per_rev / 60.0
    accel = accel_rpm * mm_s_per_rpm
    decel = decel_rpm * mm_s_per_rpm
    ref_speed = ref_scale * max_rpm * mm_s_per_rpm

    def plan_seconds(distance_mm: float, speed_mm_s: float) -> float:
        """Mirror of ZDT_Nav_PlanSeconds(). Returns SECONDS."""
        accel_mm = speed_mm_s ** 2 / (2.0 * accel)
        decel_mm = speed_mm_s ** 2 / (2.0 * decel)
        if distance_mm >= accel_mm + decel_mm:
            return (speed_mm_s / accel) + (speed_mm_s / decel) + \
                   ((distance_mm - accel_mm - decel_mm) / speed_mm_s)
        peak = math.sqrt(2.0 * distance_mm / ((1.0 / accel) + (1.0 / decel)))
        return (peak / accel) + (peak / decel)

    ref_seconds = plan_seconds(ref_dist, ref_speed)
    ref_ms = 1000.0 * ref_seconds      # ZDT_Nav_ReferencePlanMs()

    print(f"mm/rev={mm_per_rev:.4f}  A={accel:.1f} mm/s^2  D={decel:.1f} mm/s^2")
    print(f"reference condition: {ref_dist:.0f} mm at spd={ref_scale:.2f} "
          f"({ref_speed:.1f} mm/s)")
    print(f"ZDT_Nav_PlanSeconds() = {ref_seconds:.4f} s")
    print(f"ZDT_Nav_ReferencePlanMs() = {ref_ms:.1f} ms")
    print()

    problems = []
    if not guard_min <= ref_ms <= guard_max:
        problems.append(
            f"reference plan {ref_ms:.3f} ms is outside the guard range "
            f"{guard_min:.0f}~{guard_max:.0f} ms: Nav_Params_SetReferencePlanMs() "
            f"would reject it and print a WARNING at boot "
            f"(a seconds/milliseconds slip looks exactly like this)"
        )
    if 1.0 <= ref_ms <= 20.0:
        problems.append(f"reference plan {ref_ms:.3f} looks like SECONDS, not ms")

    for t1000 in (ref_ms, default_time):
        k = t1000 / ref_ms
        flag = "" if 0.5 <= k <= 2.0 else "   <-- implausible, check the units"
        print(f"  with t1000={t1000:7.0f}ms -> k={k:.3f}{flag}")

    print()
    print("expected boot line for the current calibration:")
    rpm = default_speed * max_rpm
    print(f"  [NAV] spd={default_speed:.3f} ({rpm:.0f} RPM, {rpm * mm_s_per_rpm:.0f} mm/s)"
          f"  t1000={default_time:.0f}ms  k={default_time / ref_ms:.3f}"
          f" (theory{ref_ms:.0f}ms)  cnt=65536/rev")
    print(f"  [NAV] dcomp={default_compensation:.3f}: 1000mm -> "
          f"{1000.0 * (1.0 + default_compensation):.0f}mm before timing")

    print()
    if problems:
        print(f"FAIL: {len(problems)} problem(s)")
        for item in problems:
            print(f"  - {item}")
        return 1
    print(f"PASS: reference plan {ref_ms:.1f} ms is inside "
          f"{guard_min:.0f}~{guard_max:.0f} ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
