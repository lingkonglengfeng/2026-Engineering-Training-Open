"""把独立遮挡算法同步进单文件环识别器，避免设备上依赖额外模块。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
START, END = '# BEGIN OCCUPIED_RING\n', '# END OCCUPIED_RING\n'


def sync():
    helper = (ROOT/'vision/maixcam_ring/occupied_ring.py').read_text(encoding='utf-8')
    helper = helper[helper.index('def occupied_sample_group'):]
    path = ROOT/'vision/maixcam_ring/middle_ring.py'
    source = path.read_text(encoding='utf-8')
    block = START+helper.rstrip()+'\n'+END
    if START in source:
        begin = source.index(START)
        finish = source.index(END, begin)+len(END)
        source = source[:begin]+block+source[finish:]
    else:
        source = source.replace('\nclass MiddleTracker:', '\n'+block+'\n\nclass MiddleTracker:', 1)
    path.write_text(source, encoding='utf-8')
    return path


if __name__ == '__main__':
    print(sync())
