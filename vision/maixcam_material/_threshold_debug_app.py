"""Source template for the standalone MaixCAM material threshold viewer."""
import time

_BUNDLED_MATERIAL = ""


def load_detector():
    namespace = {"__name__": "material_detector"}
    exec(_BUNDLED_MATERIAL, namespace)
    return namespace


def threshold_lines(color, threshold, status):
    return (
        "%s  L:%d..%d A:%d..%d" % (
            color[1], threshold[0], threshold[1], threshold[2], threshold[3]),
        "B:%d..%d  %s" % (threshold[4], threshold[5], status),
    )


def print_threshold(color, threshold, status):
    first, second = threshold_lines(color, threshold, status)
    print("[THRESHOLD] %s | %s" % (first, second), flush=True)


def draw_view(source, target, diagnostics, color, threshold, status, fps,
              raw_view, learning, locked, sample_count, message, image, buttons,
              window_draw=None):
    width, height = source.width(), source.height()
    if target is not None:
        cx, cy, radius = target
        source.draw_circle(round(cx), round(cy), round(radius) + 2,
                           image.COLOR_GREEN if raw_view else image.COLOR_WHITE)
        source.draw_line(round(cx) - 5, round(cy), round(cx) + 5, round(cy), image.COLOR_WHITE)
        source.draw_line(round(cx), round(cy) - 5, round(cx), round(cy) + 5, image.COLOR_WHITE)
    # Leave y=50..130 clear so the whole pickup box stays visible.
    source.draw_rect(0, 0, width, 50, image.COLOR_BLACK, thickness=-1)
    first, second = threshold_lines(color, threshold, status)
    source.draw_string(4, 2, first, image.COLOR_WHITE)
    source.draw_string(4, 19, second, image.COLOR_WHITE)
    source.draw_string(4, 36, "%s %s FPS:%.1f TX:OFF" % (
        "LEARN" if learning else "DEBUG", "RAW" if raw_view else "MASK", fps),
        image.COLOR_WHITE)
    if target is not None:
        detail = "dx:%+.1f dy:%+.1f %s" % (
            target[0] - width / 2, target[1] - height / 2,
            diagnostics.get("reason", "OK"))
    else:
        detail = "NO TARGET %s B:%d" % (
            diagnostics.get("reason", "NO_BLOBS"), diagnostics.get("count", 0))
    if window_draw is not None:
        window_draw(source, target, color[0], image, label_y=134)
    source.draw_string(4, height-69, detail, image.COLOR_WHITE)
    if message:
        source.draw_string(4, height - 56, message, image.COLOR_WHITE)
    for name, rect in buttons.items():
        x, y, w, h = rect
        source.draw_rect(x, y, w, h, image.COLOR_BLACK, thickness=-1)
        source.draw_rect(x, y, w, h, image.COLOR_WHITE)
        label = {"color": "Color", "view": "View", "learn": "Done" if learning else "Learn",
                 "lock": "Sample" if learning else ("Auto" if locked else "Lock")}[name]
        source.draw_string(x + 4, y + 8, label, image.COLOR_WHITE)
    if learning:
        source.draw_string(width - 54, 36, "N:%d" % sample_count, image.COLOR_WHITE)


def main():
    from maix import app, camera, display, image, touchscreen, err
    from maix.peripheral import gpio, pinmap

    detector = load_detector()
    print("[THRESHOLD DEBUG] UART OFF; for vehicle pickup run maixcam_service/main.py",
          flush=True)
    # Match production MATERIAL mode: A19/PWM7 at 20 kHz and 20 percent.
    detector["LIGHT_PWM_ENABLED"] = True
    light = cam = disp = touch = None
    try:
        light = detector["init_light_on"](gpio, pinmap, err)
        disp = display.Display()
        cam = camera.Camera(detector["CAM_WIDTH"], detector["CAM_HEIGHT"],
                            image.Format.FMT_RGB888, fps=detector["CAM_FPS"])
        touch = touchscreen.TouchScreen()
        fw, fh = cam.width(), cam.height()
        geometry = detector["fit_geometry"](fw, fh, disp.width(), disp.height())
        buttons = {name: (i * 80 + 2, fh - 38, 76, 34)
                   for i, name in enumerate(("color", "view", "learn", "lock"))}
        controller = detector["TouchButtons"]()
        samples = detector["load_samples"]()
        colors = detector["COLORS"]
        index, raw_view, learning, locked = 0, True, False, False
        replace_samples = False
        previous, fps = time.monotonic(), 0.0
        message = ""
        last_printed = None
        while not app.need_exit():
            sample_point = None
            for _ in range(16):
                if not touch.available():
                    break
                tx, ty, pressed = touch.read0()
                fx, fy = detector["frame_point"](tx, ty, fw, fh, geometry)
                rising = bool(pressed) and not controller.pressed
                action = controller.update(fx, fy, pressed, time.monotonic(), buttons)
                if action == "color":
                    index = (index + 1) % len(colors)
                    learning, replace_samples, message = False, False, ""
                elif action == "view":
                    raw_view = not raw_view
                elif action == "learn":
                    learning = not learning
                    replace_samples, message = learning, ""
                    if learning:
                        raw_view = True
                elif action == "lock" and learning:
                    sample_point = fw / 2, fh / 2
                elif action == "lock":
                    try:
                        if locked:
                            cam.exp_mode(camera.AeMode.Auto)
                            cam.awb_mode(camera.AwbMode.Auto)
                        else:
                            detector["camera_lock"](cam)
                        locked = not locked
                        message = ""
                    except Exception as exc:
                        print("[THRESHOLD] camera lock failed:", exc, flush=True)
                        locked = False
                        message = "LOCK FAIL"
                elif learning and rising and 50 <= fy < fh - 38:
                    sample_point = fx, fy
            frame = cam.read()
            if frame is None:
                time.sleep(0.005)
                continue
            color = colors[index]
            if learning and sample_point is not None:
                value = detector["sample_lab"](frame, *sample_point)
                if color[0] in (3, 6) and (value[0] < 5 or value[0] > 95 or value[2] >= -3):
                    message = "AVOID GLARE / DARK"
                else:
                    if replace_samples:
                        samples[str(color[0])] = []
                        replace_samples = False
                    values = samples.setdefault(str(color[0]), [])
                    values.append(value)
                    del values[:-detector["MAX_SAMPLES"]]
                    message = "SAMPLE %d,%d,%d" % tuple(value)
                    try:
                        detector["save_samples"](samples)
                    except OSError as exc:
                        print("[THRESHOLD] save failed:", exc, flush=True)
                        message = "SAVE FAIL: RAM ONLY"
            threshold, status = detector["calibrated_threshold"](color, samples)
            current_threshold = (color[0], tuple(threshold), status)
            if current_threshold != last_printed:
                print_threshold(color, threshold, status)
                last_printed = current_threshold
            diagnostics = {}
            # Binary conversion is in place; keep the unmodified frame for RAW.
            target, mask = detector["detect_material_center"](
                frame.copy() if raw_view else frame, threshold, image, diagnostics)
            if status == "OVERLAP":
                target = None
                diagnostics["reason"] = "CAL_OVERLAP"
            now = time.monotonic()
            if now > previous:
                instant = 1.0 / (now - previous)
                fps = instant if fps == 0 else fps * 0.8 + instant * 0.2
            previous = now
            shown = frame if raw_view else mask
            if shown.format() != image.Format.FMT_RGB888:
                shown = shown.to_format(image.Format.FMT_RGB888)
            draw_view(shown, target, diagnostics, color, threshold, status, fps,
                      raw_view, learning, locked, len(samples.get(str(color[0]), [])),
                      message, image, buttons, detector["draw_pick_window"])
            disp.show(shown, fit=image.Fit.FIT_CONTAIN)
    finally:
        detector["light_off_on_exit"](light)
        for device in (touch, cam, disp):
            if device is not None:
                try:
                    device.close()
                except Exception as exc:
                    print("[THRESHOLD] close failed:", exc, flush=True)


if __name__ == "__main__":
    main()
