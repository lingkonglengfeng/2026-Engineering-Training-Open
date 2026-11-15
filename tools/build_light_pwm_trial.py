"""生成独立/统一视觉PWM50试验副本；不把PWM7默认启用到Wi-Fi正式版。"""
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
TARGETS={
    'material_pwm50.py':'vision/maixcam_material/main.py',
    'plate_pwm50.py':'vision/maixcam_material/plate_tangent.py',
    'middle_ring_pwm50.py':'vision/maixcam_ring/middle_ring.py',
    'main_pwm50.py':'vision/maixcam_service/main.py',
}


def build():
    out=ROOT/'output/maixcam_light';out.mkdir(parents=True,exist_ok=True)
    for name,target in TARGETS.items():
        source=(ROOT/target).read_text(encoding='utf-8')
        begin=source.index('\n# LIGHT_GPIO_BEGIN\n')
        end=source.index('\n# LIGHT_GPIO_END',begin)
        block=source[begin:end]
        if block.count('LIGHT_PWM_ENABLED = False')+block.count('LIGHT_PWM_ENABLED = True')!=1:
            raise ValueError('Unexpected light helper')
        block=block.replace('LIGHT_PWM_ENABLED = False','LIGHT_PWM_ENABLED = True',1)
        source=source[:begin]+block+source[end:]
        source=source.replace('2026-10-05-weak-middle-v10','2026-10-05-weak-middle-pwm50-v10',1)
        (out/name).write_text('# Generated PWM50 USB trial; edit source then rebuild\n'+source,encoding='utf-8')
    return out


if __name__=='__main__':print(build())
