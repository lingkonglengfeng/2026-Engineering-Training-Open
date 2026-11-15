"""Bundle both versioned camera apps and current PC sources; no local logs/settings."""
from pathlib import Path
import hashlib,subprocess,zipfile,argparse,shutil
from package_pc_vision import SOURCES
from package_pc_vision_bridge import package as cameras

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'output/camera_screen_pc267_2026-11-01'
DOC='docs/electrical/2026-11-01_相机屏幕同步电脑识别标注_PC266.md'

def package(committed=False):
    STAGE.mkdir(parents=True,exist_ok=True)
    apps=cameras();sources=set(SOURCES+[DOC,'vision/maixcam_bridge/README.md'])
    for path in (ROOT/'vision/pc_vision').iterdir():
        if path.is_file() and (path.suffix in ('.py','.ps1','.md') or path.name in ('requirements.txt','config.example.json','config.cam2.example.json')):
            sources.add(path.relative_to(ROOT).as_posix())
    for path in (ROOT/'docs/electrical').glob('*.md'):
        if any(tag in path.name for tag in ('V238','V239','V240','V241','V242','V243','V244','PC262')):sources.add(path.relative_to(ROOT).as_posix())
    sources.update(path.relative_to(ROOT).as_posix() for path in apps)
    pc=STAGE/'PC2.6.7_相机识别标注回显.zip'
    with zipfile.ZipFile(pc,'w',zipfile.ZIP_DEFLATED) as z:
        for name in sorted(sources):z.write(ROOT/name,'PC上位机/'+name)
    for app in apps:shutil.copy2(app,STAGE/app.name)
    shutil.copy2(ROOT/DOC,STAGE/'使用说明.md')
    note='PC2.6.7 + CAM1桥接1.3.1 + CAM2桥接1.4.1。分别安装对应相机应用，确认开机自启，然后重启上位机。主控无需更新，W25参数不变。标注回显最高10FPS/320x240，可在上位机关闭；没有电脑标注约1秒后回退本地原图。会增加处理和无线开销，实机FPS/延迟尚未测量。相机包中的main.py已生成，无需在相机安装OpenCV。未代为安装、烧录或操作机器人。\n'
    if committed:note+='源码提交：'+subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()+'\n'
    (STAGE/'安装说明.txt').write_text(note,encoding='utf-8')
    files=[pc,STAGE/'使用说明.md',STAGE/'安装说明.txt']+[STAGE/p.name for p in apps]
    (STAGE/'SHA256SUMS.txt').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in files),encoding='utf-8')
    out=ROOT/'output/双相机识别标注回显_PC2.6.7_CAM1-1.3.1_CAM2-1.4.1_2026-11-01.zip'
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
        for p in files+[STAGE/'SHA256SUMS.txt']:z.write(p,'相机标注回显/'+p.name)
    for path in (pc,out,*apps):
        with zipfile.ZipFile(path) as z:assert z.testzip() is None
    print('PASS: camera+PC ZIP CRC/SHA256; bytes='+str(out.stat().st_size))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--committed',action='store_true');package(parser.parse_args().committed)
