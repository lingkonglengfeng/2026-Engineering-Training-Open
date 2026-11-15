"""普通MaixCAM A19/PWM7 50%补光试验：请用USB，预览60秒后关闭。

20kHz只是可修改的试验值，先确认灯板EN允许的PWM频率；不收发车辆串口。
官方提示PWM4~9不建议与Wi-Fi同时使用。本文件独立上传，不与视觉主程序并行。
"""
import time

LIGHT_PIN, LIGHT_GPIO, PWM_ID = 'A19', 'GPIOA19', 7
PWM_FREQ_HZ = 20000  # Hz；按灯板允许范围修改，不代表已验证灯板支持。
PWM_DUTY_PERCENT = 50.0
TEST_SECONDS = 60


def run_device():
    from maix import app, camera, display, image, err
    from maix.peripheral import pwm, gpio, pinmap
    out=cam=disp=light=None
    try:
        print('[LIGHT PWM] A19/PWM7: use USB; Wi-Fi resources may conflict',flush=True)
        err.check_raise(pinmap.set_pin_function(LIGHT_PIN,LIGHT_GPIO),'GPIO mapping failed')
        light=gpio.GPIO(LIGHT_GPIO,gpio.Mode.OUT,gpio.Pull.PULL_DOWN)
        light.value(0)
        out=pwm.PWM(PWM_ID,freq=PWM_FREQ_HZ,duty=PWM_DUTY_PERCENT,enable=False)
        err.check_raise(pinmap.set_pin_function(LIGHT_PIN,'PWM7'),'PWM7 mapping failed')
        err.check_raise(out.enable(),'PWM enable failed')
        print('[LIGHT PWM] %.1f%% %dHz; raw preview; auto stop after %ds' % (
            PWM_DUTY_PERCENT,PWM_FREQ_HZ,TEST_SECONDS),flush=True)
        disp=display.Display()
        cam=camera.Camera(480,360,image.Format.FMT_RGB888,fps=30)
        try:display.set_trans_image_quality(75)
        except (AttributeError,RuntimeError):pass
        deadline=time.monotonic()+TEST_SECONDS
        while not app.need_exit() and time.monotonic()<deadline:
            frame=cam.read(block=True,block_ms=250)
            if frame is None:continue
            # 不加十字/轮廓/文字，便于保存补光后的原始画面评估对比度。
            shown=frame.resize(320,240)
            result=disp.show(shown,fit=image.Fit.FIT_CONTAIN)
            if result is not None and int(result)!=0:raise RuntimeError('display failed')
    finally:
        if out is not None:
            try:
                if out.duty(0)<0:print('[LIGHT PWM] unable to set zero duty')
            except Exception as exc:print('[LIGHT PWM] duty cleanup:',exc)
            try:err.check_raise(out.disable(),'PWM disable failed')
            except Exception as exc:print('[LIGHT PWM] disable cleanup:',exc)
            out=None
        try:
            err.check_raise(pinmap.set_pin_function(LIGHT_PIN,LIGHT_GPIO),'Restore GPIO failed')
            if light is None:light=gpio.GPIO(LIGHT_GPIO,gpio.Mode.OUT,gpio.Pull.PULL_DOWN)
            light.value(0)
            print('[LIGHT PWM] A19 restored GPIO LOW/OFF',flush=True)
        except Exception as exc:print('[LIGHT PWM] GPIO cleanup:',exc)
        for device in (cam,disp):
            if device is not None:
                try:device.close()
                except Exception as exc:print('[LIGHT PWM] close:',exc)


if __name__=='__main__':run_device()
