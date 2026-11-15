"""普通MaixCAM A19补光控制：设备入口启动开启，退出关闭。"""
LIGHT_PIN = "A19"
LIGHT_GPIO = "GPIOA19"
LIGHT_OFF_LEVEL = 0  # 按高电平使能灯板；低电平使能板需改为1。
LIGHT_PWM_ENABLED = False  # 由各入口配置 GPIO/PWM；亮度为灯板占空比。
LIGHT_PWM_ID, LIGHT_PWM_FREQ_HZ = 7, 20000
LIGHT_BRIGHTNESS_PERCENT = 50.0


def _init_light(gpio, pinmap, err, enabled):
    if LIGHT_OFF_LEVEL not in (0, 1):
        raise ValueError("LIGHT_OFF_LEVEL must be 0 or 1")
    if enabled and LIGHT_PWM_ENABLED:
        return _init_pwm_light(gpio,pinmap,err)
    err.check_raise(pinmap.set_pin_function(LIGHT_PIN, LIGHT_GPIO), "Light GPIO pin mapping failed")
    level = 1 - LIGHT_OFF_LEVEL if enabled else LIGHT_OFF_LEVEL
    # SDK输出模式的Pull决定初始值；构造时直接设为所需电平。
    pull = gpio.Pull.PULL_DOWN if level == 0 else gpio.Pull.PULL_UP
    light = gpio.GPIO(LIGHT_GPIO, gpio.Mode.OUT, pull)
    light.value(level)
    print("[LIGHT] %s output=%d (%s; active level must match module)" % (
        LIGHT_GPIO, level, "ON" if enabled else "OFF"), flush=True)
    return light


def init_light_on(gpio, pinmap, err):
    return _init_light(gpio, pinmap, err, True)


def init_light_off(gpio, pinmap, err):
    return _init_light(gpio, pinmap, err, False)


class _PwmLight:
    def __init__(self,output,gpio,pinmap,err):
        self.output,self.gpio,self.pinmap,self.err=output,gpio,pinmap,err
        self.off_gpio=None

    def set_brightness(self, percent):
        import math
        if not isinstance(percent,(int,float)) or not math.isfinite(percent) or not 0 <= percent <= 100:
            raise ValueError('Invalid light brightness percent')
        if self.output is None:
            raise RuntimeError('Light PWM is disabled')
        duty = percent if LIGHT_OFF_LEVEL == 0 else 100-percent
        actual = self.output.duty(duty)
        if (not isinstance(actual,(int,float)) or not math.isfinite(actual)
                or not 0 <= actual <= 100 or abs(actual-duty)>0.1):
            raise RuntimeError('Light PWM duty request=%g returned=%r' % (duty,actual))
        return actual

    def value(self,level):
        if level != LIGHT_OFF_LEVEL:
            raise ValueError('PWM light handle only supports OFF cleanup')
        failure=None
        if self.output is not None:
            try:
                duty=self.output.duty(0 if LIGHT_OFF_LEVEL==0 else 100)
                if duty<0:raise RuntimeError('PWM duty OFF failed')
            except Exception as exc:failure=exc
            try:self.err.check_raise(self.output.disable(),'PWM disable failed')
            except Exception as exc:failure=exc
            self.output=None
        try:
            self.err.check_raise(self.pinmap.set_pin_function(LIGHT_PIN,LIGHT_GPIO),'Restore light GPIO failed')
            self.off_gpio.value(LIGHT_OFF_LEVEL)  # 复用已持有GPIO，避免重复申请同一引脚。
        except Exception as exc:failure=exc
        if failure is not None:raise failure


def _init_pwm_light(gpio,pinmap,err):
    from maix.peripheral import pwm
    if not 0 <= LIGHT_BRIGHTNESS_PERCENT <= 100 or LIGHT_PWM_FREQ_HZ <= 0:
        raise ValueError('Invalid light PWM configuration')
    # 用户确认灯板支持PWM。A19/PWM7按官方说明应使用USB验证Wi-Fi资源兼容。
    off=_init_light(gpio,pinmap,err,False)
    out=None
    try:
        duty=LIGHT_BRIGHTNESS_PERCENT if LIGHT_OFF_LEVEL==0 else 100-LIGHT_BRIGHTNESS_PERCENT
        print('[LIGHT PWM] A19/PWM7 %dHz %.1f%%; USB test recommended (Wi-Fi resources)' % (
            LIGHT_PWM_FREQ_HZ,LIGHT_BRIGHTNESS_PERCENT),flush=True)
        out=pwm.PWM(LIGHT_PWM_ID,freq=LIGHT_PWM_FREQ_HZ,duty=duty,enable=False)
        err.check_raise(pinmap.set_pin_function(LIGHT_PIN,'PWM%d'%LIGHT_PWM_ID),'Light PWM mapping failed')
        err.check_raise(out.enable(),'Light PWM enable failed')
    except Exception:
        if out is not None:
            try:out.disable()
            except Exception:pass
            out=None
        err.check_raise(pinmap.set_pin_function(LIGHT_PIN,LIGHT_GPIO),'Restore light GPIO failed')
        off.value(LIGHT_OFF_LEVEL)
        raise
    handle=_PwmLight(out,gpio,pinmap,err)
    handle.off_gpio=off
    return handle


def light_off_on_exit(light):
    if light is not None:
        try:
            light.value(LIGHT_OFF_LEVEL)
        except Exception as exc:
            print("[LIGHT] unable to set OFF on exit:", exc)
