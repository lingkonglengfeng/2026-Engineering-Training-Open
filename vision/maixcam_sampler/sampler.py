"""CAM1 dataset sampler; camera/light only, no detectors or vehicle UART.

Generate standalone main.py with tools/build_vision_sampler.py.
"""
import json
import math
import os
from pathlib import Path
import time

# LIGHT_DRIVER_INSERT
# CAMERA_CONFIG_INSERT
SAVE_ROOT = '/root/vision_samples'
AUTO_INTERVAL_S = 1.0
LIGHT_SETTLE_S = 0.5
MAX_SAMPLES = 1000
SAMPLER_BUILD = '2026-10-13-sampling-audit-v2'
PREVIEW_WIDTH, PREVIEW_HEIGHT = 320, 240


class Sampler:
    def __init__(self, light, directory, clock=None):
        self.light, self.directory, self.clock = light, Path(directory), clock or time.monotonic
        self.mode, self.count, self.auto = None, 0, False
        self.epoch=0;self.light_ok=False;self.needs_flush=True;self.next_number=1
        self.ready_at = self.next_save = 0.0
        self.message = 'Choose RING / PLATE'
        self.switch('ring')

    def switch(self, mode):
        if mode not in ('ring', 'plate'):
            raise ValueError('Unknown sampling profile')
        percent = RING_LIGHT_BRIGHTNESS_PERCENT if mode == 'ring' else LIGHT_BRIGHTNESS_PERCENT
        self.light_ok=False;self.auto=False
        self.light.set_brightness(percent)  # Failed writes inhibit capture until a successful reselect.
        self.light_ok=True;self.epoch+=1;self.needs_flush=True
        self.mode = mode
        self.ready_at = self.clock() + LIGHT_SETTLE_S
        self.next_save = self.ready_at
        self.message = '%s lamp %.0f%% settling' % (mode.upper(), percent)

    def flush_if_ready(self,cam):
        if not self.light_ok or self.clock()<self.ready_at:return False
        if not self.needs_flush:return True
        clear=getattr(cam,'clear_buff',None)
        if not callable(clear):
            self.auto=False;self.message='No clear_buff; capture inhibited'
            return False
        clear()  # Flush AFTER settle, then read a new frame, not the previous mode's frame.
        self.needs_flush=False
        self.message='%s ready; SNAP / AUTO'%self.mode.upper()
        return True

    def reserve(self,profile):
        while True:
            stem='%s_%06d'%(self.mode,self.next_number);self.next_number+=1
            if any(profile.glob(stem+'*')):continue
            marker=profile/(stem+'.reserved')
            try:
                with marker.open('x'):pass
                return stem,marker
            except FileExistsError:continue

    @staticmethod
    def save_image(frame,path):
        result=frame.save(str(path))
        if result is not None and int(result)!=0:raise RuntimeError('Image save failed: %s'%result)
        if not path.is_file() or path.stat().st_size==0:raise RuntimeError('Image writer created no data')

    def append_record(self,record):
        data=(json.dumps(record,ensure_ascii=False)+'\n').encode('utf-8')
        with (self.directory/'samples.jsonl').open('ab',buffering=0) as stream:
            before=stream.tell()
            try:
                if stream.write(data)!=len(data):raise OSError('Short metadata write')
                stream.flush()
            except Exception:
                stream.seek(before);stream.truncate(before)
                raise

    def capture(self, frame, frame_started=None, frame_epoch=None):
        if (not self.light_ok or self.needs_flush
                or (frame_started is not None and frame_started<self.ready_at)
                or (frame_epoch is not None and frame_epoch!=self.epoch)):
            self.message='Wait for fresh frame / confirmed lamp'
            return False
        if self.clock() < self.ready_at:
            self.message = 'Wait for lamp/AE to settle'
            return False
        if self.count >= MAX_SAMPLES:
            self.auto = False
            self.message = 'Sample limit reached'
            return False
        if (frame.width(), frame.height()) != (CAPTURE_WIDTH, CAPTURE_HEIGHT):
            raise RuntimeError('Unexpected camera dimensions')
        profile = self.directory / self.mode
        profile.mkdir(parents=True, exist_ok=True)
        stem,marker=self.reserve(profile)
        native = profile / (stem + '.png')
        pending = profile / (stem + '.pending.png')
        # Save raw first; draw UI only on a separate resized preview.
        processing = None
        processing_pending=profile/(stem+'_320x240.pending.png')
        record = dict(file=native.relative_to(self.directory).as_posix(), mode=self.mode,
                      timestamp=time.time(), capture=[CAPTURE_WIDTH, CAPTURE_HEIGHT],
                      processing_file=str(processing.relative_to(self.directory)) if processing else None,
                      lamp_percent=RING_LIGHT_BRIGHTNESS_PERCENT if self.mode == 'ring' else LIGHT_BRIGHTNESS_PERCENT,
                      lamp_pin=LIGHT_PIN, pwm_id=LIGHT_PWM_ID, pwm_hz=LIGHT_PWM_FREQ_HZ,
                      camera_fps_requested=CAPTURE_FPS, annotations=False)
        try:
            self.save_image(frame,pending)
            if self.mode=='plate':
                processing=profile/(stem+'_320x240.png')
                self.save_image(frame.resize(320,240),processing_pending)
            os.replace(str(pending),str(native))
            if processing is not None:os.replace(str(processing_pending),str(processing))
            record.update(processing_file=processing.relative_to(self.directory).as_posix() if processing else None,
                          frame_read_start_monotonic=frame_started,profile_epoch=self.epoch)
            self.append_record(record)
        except Exception:
            self.auto=False
            for path in (pending,native,processing_pending,processing):
                if path is not None:
                    try:path.unlink()
                    except FileNotFoundError:pass
            raise  # Keep reservation so retry cannot overwrite/reuse a failed sample's number.
        try:marker.unlink()
        except OSError as exc:print('[SAMPLE] reservation retained:',exc,flush=True)
        self.count += 1
        self.next_save = self.clock() + AUTO_INTERVAL_S
        self.message = 'Saved %s #%d' % (self.mode.upper(), self.count)
        print('[SAMPLE]', json.dumps(record, ensure_ascii=False), flush=True)
        return True

    def action(self, x, y, frame,frame_started=None,frame_epoch=None):
        if not 0<=x<320 or not 0<=y<240:return None
        if y < 32:
            self.switch('ring' if x < 160 else 'plate')
            return 'switch'
        if y >= 208:
            if x < 160:
                self.capture(frame,frame_started,frame_epoch)
            else:
                self.auto = not self.auto
                self.next_save = max(self.clock(), self.ready_at)
            return 'capture'
        return None


def run_device():
    from maix import app, camera, display, image, err, touchscreen
    from maix.peripheral import gpio, pinmap
    cam = disp = touch = light = None
    try:
        light = init_light_on(gpio, pinmap, err)
        directory = Path(SAVE_ROOT) / (time.strftime('%Y%m%d_%H%M%S') + '_%06d' % (time.time_ns() % 1000000))
        directory.mkdir(parents=True, exist_ok=False)
        state = Sampler(light, directory)
        cam = camera.Camera(CAPTURE_WIDTH, CAPTURE_HEIGHT, image.Format.FMT_RGB888, fps=CAPTURE_FPS)
        state.ready_at=max(state.ready_at,time.monotonic()+LIGHT_SETTLE_S)
        state.next_save=state.ready_at
        disp, touch = display.Display(), touchscreen.TouchScreen()
        pressed_before = False
        print('[SAMPLE] build=%s camera/light only; PNG directory=%s' % (SAMPLER_BUILD,directory), flush=True)
        while not app.need_exit():
            try:state.flush_if_ready(cam)
            except Exception as exc:
                state.auto=False;state.message='BUFFER ERROR: '+str(exc)[:22]
            frame_started=time.monotonic();frame_epoch=state.epoch
            frame = cam.read(block=True, block_ms=250)
            if frame is None:
                time.sleep(.005)
                continue
            switched = False
            for _ in range(16):
                if not touch.available():
                    break
                x, y, pressed = touch.read0()
                if pressed and not pressed_before:
                    # Display.show uses FIT_CONTAIN; map physical screen to 320x240 canvas.
                    scale = min(disp.width() / PREVIEW_WIDTH, disp.height() / PREVIEW_HEIGHT)
                    x = (x - (disp.width() - PREVIEW_WIDTH * scale) / 2) / scale
                    y = (y - (disp.height() - PREVIEW_HEIGHT * scale) / 2) / scale
                    if 0 <= x < PREVIEW_WIDTH and 0 <= y < PREVIEW_HEIGHT:
                        try:
                            switched = state.action(x, y, frame,frame_started,frame_epoch) == 'switch' or switched
                        except Exception as exc:
                            state.auto = False
                            state.message = 'ERROR: ' + str(exc)[:32]
                            print('[SAMPLE ERROR]', exc, flush=True)
                pressed_before = bool(pressed)
            if state.auto and not switched and time.monotonic() >= max(state.ready_at, state.next_save):
                try:
                    state.capture(frame,frame_started,frame_epoch)
                except Exception as exc:
                    state.auto = False
                    state.message = 'SAVE ERROR: ' + str(exc)[:27]
                    print('[SAMPLE ERROR]', exc, flush=True)
            shown = frame.resize(PREVIEW_WIDTH, PREVIEW_HEIGHT)
            shown.draw_rect(0, 0, 320, 32, image.COLOR_BLACK, thickness=-1)
            shown.draw_rect(0, 208, 320, 32, image.COLOR_BLACK, thickness=-1)
            shown.draw_string(6, 6, 'RING %g%%' % RING_LIGHT_BRIGHTNESS_PERCENT, image.COLOR_WHITE)
            shown.draw_string(166, 6, 'PLATE %g%%' % LIGHT_BRIGHTNESS_PERCENT, image.COLOR_WHITE)
            shown.draw_string(6, 210, 'SNAP', image.COLOR_WHITE)
            shown.draw_string(166, 210, 'AUTO ' + ('ON' if state.auto else 'OFF'), image.COLOR_WHITE)
            shown.draw_string(6, 188, state.message, image.COLOR_WHITE)
            result = disp.show(shown, fit=image.Fit.FIT_CONTAIN)
            if result is not None:
                err.check_raise(result, 'Sampler display failed')
    finally:
        light_off_on_exit(light)
        for device in (cam, touch, disp):
            if device is not None:
                close = getattr(device, 'close', None)
                if close is not None:
                    try:
                        close()
                    except Exception as exc:
                        print('[SAMPLE CLOSE]', exc)


if __name__ == '__main__':
    run_device()
