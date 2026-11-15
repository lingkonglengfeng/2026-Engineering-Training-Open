"""将同一协议源码嵌入三个独立 MaixCAM 文件；--check 验证副本一致。"""
from pathlib import Path
import argparse

ROOT = Path(__file__).resolve().parents[1]
START, END = "# UART_PROTOCOL8_BEGIN", "# UART_PROTOCOL8_END"
TARGETS = ("vision/maixcam_material/main.py", "vision/maixcam_material/plate_tangent.py",
           "vision/maixcam_ring/middle_ring.py")


def embedded_source():
    source = (ROOT / "vision/protocol8.py").read_text(encoding="utf-8")
    # 模块说明不嵌入，避免在程序中间形成额外模块 docstring。
    source = source.split("\n", 1)[1].strip()
    return START + "\n" + source + "\n" + END


def sync(check=False):
    expected = embedded_source()
    for target in TARGETS:
        path = ROOT / target
        source = path.read_text(encoding="utf-8")
        if START in source:
            begin, end = source.index(START), source.index(END) + len(END)
            if check and source[begin:end] != expected:
                raise SystemExit("Protocol8 copy differs: " + target)
            if not check:
                path.write_text(source[:begin] + expected + source[end:], encoding="utf-8")
        elif check:
            raise SystemExit("Missing Protocol8 copy: " + target)
        else:
            at = source.index("\n\ndef ")
            path.write_text(source[:at] + "\n\n" + expected + source[at:], encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    sync(parser.parse_args().check)
