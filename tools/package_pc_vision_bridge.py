"""Package separate CAM1/CAM2 bridge apps without changing frozen QR releases."""
from pathlib import Path
import argparse
import zipfile

from build_pc_vision_bridge import build,ROOT


def package(roles=('material','qr')):
    paths=[]
    for role,folder,app_id in (('material','maixcam_bridge','cam1_pc_bridge'),('qr','maixcam_qr_bridge','cam2_pc_bridge')):
        if role not in roles:continue
        app=ROOT/'vision'/folder
        manifest = (app / 'app.yaml').read_text(encoding='utf-8')
        version = next(line.split(':', 1)[1].strip() for line in manifest.splitlines() if line.startswith('version:'))
        destination=app/'dist'/('maix-'+app_id+'-v'+version+'.zip')
        destination.parent.mkdir(exist_ok=True)
        main=build(role)
        compile(main.read_text(encoding='utf-8'),str(main),'exec')
        with zipfile.ZipFile(destination,'w',zipfile.ZIP_DEFLATED) as archive:
            for name in ('app.yaml','app.png','main.py'):
                entry=zipfile.ZipInfo(name,(2026,11,1,0,0,0));entry.compress_type=zipfile.ZIP_DEFLATED
                archive.writestr(entry,(app/name).read_bytes())
        paths.append(destination)
    return paths


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--camera',choices=('material','qr','all'),default='all')
    args=parser.parse_args()
    roles=('material','qr') if args.camera=='all' else (args.camera,)
    for path in package(roles):print(path)
