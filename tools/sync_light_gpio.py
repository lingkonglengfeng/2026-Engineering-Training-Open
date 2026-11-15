"""嵌入同一A19控制代码，维持各相机程序单文件运行。--check校验副本。"""
from pathlib import Path
import argparse

ROOT = Path(__file__).resolve().parents[1]
START, END = "# LIGHT_GPIO_BEGIN", "# LIGHT_GPIO_END"
TARGETS = ("vision/maixcam_service/service.py", "vision/maixcam_material/plate_tangent.py",
           "vision/maixcam_material/main.py", "vision/maixcam_ring/middle_ring.py")
PWM_TARGETS = {"vision/maixcam_service/service.py", "vision/maixcam_material/main.py"}


def sync(check=False):
    source = (ROOT / "vision/light_gpio.py").read_text(encoding="utf-8").split("\n", 1)[1].strip()
    for target in TARGETS:
        configured=source.replace('LIGHT_PWM_ENABLED = False','LIGHT_PWM_ENABLED = True',1) if target in PWM_TARGETS else source
        if target == "vision/maixcam_service/service.py":
            configured = configured.replace('LIGHT_BRIGHTNESS_PERCENT = 50.0',
                'LIGHT_BRIGHTNESS_PERCENT = 50.0\nRING_LIGHT_BRIGHTNESS_PERCENT = 15.0\n'
                'MATERIAL_LIGHT_BRIGHTNESS_PERCENT = 20.0  # Material color detection and calibration.\n'
                'PLATE_LIGHT_BRIGHTNESS_PERCENT = 30.0', 1)
        elif target == "vision/maixcam_material/main.py":
            configured = configured.replace('LIGHT_BRIGHTNESS_PERCENT = 50.0',
                                            'LIGHT_BRIGHTNESS_PERCENT = 20.0', 1)
        elif target == "vision/maixcam_material/plate_tangent.py":
            configured = configured.replace('LIGHT_PWM_ENABLED = False','LIGHT_PWM_ENABLED = True',1)
            configured = configured.replace('LIGHT_BRIGHTNESS_PERCENT = 50.0',
                                            'LIGHT_BRIGHTNESS_PERCENT = 30.0', 1)
        expected = START + "\n" + configured + "\n" + END
        path = ROOT / target
        body = path.read_text(encoding="utf-8")
        if START in body:
            begin, finish = body.index(START), body.index(END) + len(END)
            if check and body[begin:finish] != expected:
                raise SystemExit("Light GPIO differs: " + target)
            if not check:
                path.write_text(body[:begin] + expected + body[finish:], encoding="utf-8")
        elif check:
            raise SystemExit("Missing light GPIO: " + target)
        else:
            # 协议和GPIO必须独立嵌入，不能让sync_protocol8把灯板默认OFF一起覆盖掉。
            at = body.index("# UART_PROTOCOL8_BEGIN") if "# UART_PROTOCOL8_BEGIN" in body else body.index("\n\ndef ")
            path.write_text(body[:at] + "\n\n" + expected + body[at:], encoding="utf-8")


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("--check", action="store_true")
    sync(args.parse_args().check)
