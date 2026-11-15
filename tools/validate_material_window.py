"""Render the production window overlay with synthetic targets for layout inspection.

OpenCV drawing approximates Maix fonts; this is not a hardware/FPS validation.
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("window_material", ROOT / "vision/maixcam_material/main.py")
material = importlib.util.module_from_spec(spec)
spec.loader.exec_module(material)
spec = importlib.util.spec_from_file_location("window_debug", ROOT / "vision/maixcam_material/_threshold_debug_app.py")
debug = importlib.util.module_from_spec(spec)
spec.loader.exec_module(debug)
IMAGE = SimpleNamespace(COLOR_BLACK=(0, 0, 0), COLOR_WHITE=(255, 255, 255),
                        COLOR_GREEN=(0, 255, 0), COLOR_YELLOW=(0, 255, 255))


class Raster:
    def __init__(self, array): self.array = array
    def width(self): return self.array.shape[1]
    def height(self): return self.array.shape[0]
    def draw_rect(self, x, y, w, h, color, thickness=1):
        cv2.rectangle(self.array, (x, y), (x+w-1, y+h-1), color, thickness)
    def draw_line(self, x0, y0, x1, y1, color, thickness=1):
        cv2.line(self.array, (x0, y0), (x1, y1), color, thickness)
    def draw_circle(self, x, y, r, color, thickness=1):
        cv2.circle(self.array, (x, y), r, color, thickness)
    def draw_string(self, x, y, text, color):
        cv2.putText(self.array, text, (x, y+11), cv2.FONT_HERSHEY_SIMPLEX, .37, color, 1, cv2.LINE_AA)


def render():
    out = ROOT / "output/material_window_v20_2026-10-08"
    out.mkdir(parents=True, exist_ok=True)
    panels = []
    for label, target in (("IN: synthetic target", (160, 90, 18)),
                          ("OUT: synthetic target", (230, 145, 18)),
                          ("NO TARGET", None)):
        shown = Raster(np.zeros((240, 320, 3), np.uint8))
        if target:
            cv2.circle(shown.array, target[:2], target[2], (255, 255, 255), -1)
        shown.draw_string(4, 4, label, IMAGE.COLOR_WHITE)
        material.draw_pick_window(shown, target, 1, IMAGE)
        panels.append(shown.array)
    shown = Raster(np.full((240, 320, 3), 80, np.uint8))
    cv2.circle(shown.array, (160, 90), 18, (40, 40, 220), -1)
    buttons = {name: (i*80+2, 202, 76, 34) for i, name in enumerate(("color", "view", "learn", "lock"))}
    debug.draw_view(shown, (160, 90, 18), {"reason": "OK"}, material.COLORS[0],
                    material.COLORS[0][2], "DEFAULT", 0, True, False, False,
                    0, "", IMAGE, buttons, material.draw_pick_window)
    panels.append(shown.array)
    canvas = np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))
    path = out / "window_layout_simulation.png"
    cv2.imencode(".png", canvas)[1].tofile(str(path))
    print(path)


if __name__ == "__main__": render()
