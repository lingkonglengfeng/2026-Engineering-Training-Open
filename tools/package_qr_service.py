"""Build CAM2 standalone app/install zip and auditable delivery manifest.

Only writes CAM2 source main.py, CAM2 dist, and its named output folder.
Run with standard Python; no OpenCV is needed for packaging.
"""
import hashlib
import json
import re
from pathlib import Path
import shutil
from zipfile import ZipFile, ZIP_DEFLATED

from build_qr_service import build, ROOT

OUTPUT = ROOT / 'output/cam2_fixed_rollback_v103_2026-10-13'
APP = ROOT / 'vision/maixcam_qr'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def package():
    main = build()
    compile(main.read_text(encoding='utf-8'), str(main), 'exec')
    OUTPUT.mkdir(parents=True, exist_ok=True)
    dist = APP / 'dist'
    dist.mkdir(exist_ok=True)
    metadata=dict(line.split(':',1) for line in (APP/'app.yaml').read_text(encoding='utf-8').splitlines() if ':' in line)
    app_zip = dist / ('maix-%s-v%s.zip'%(metadata['id'].strip(),metadata['version'].strip()))
    if metadata['version'].strip()=='1.0.3':
        with ZipFile(app_zip) as archive:
            for name in ('app.yaml','app.png','main.py'):
                if archive.read(name)!=(APP/name).read_bytes():
                    raise ValueError('Frozen1.0.3 mismatch: '+name)
    else:
        with ZipFile(app_zip, 'w', ZIP_DEFLATED) as archive:
            for name in ('app.yaml', 'app.png', 'main.py'):
                archive.write(APP / name, name)
    for path in (main, APP / 'README.md', app_zip):
        shutil.copy2(path, OUTPUT / path.name)
    paths = [main, APP / 'service.py', APP / 'test_qr_service.py',
             APP / 'app.yaml', APP / 'app.png', APP / 'README.md', app_zip,
             Path(__file__), ROOT / 'tools/build_qr_service.py', ROOT / 'tools/validate_qr_release.py']
    paths += [OUTPUT/name for name in ('validation.json', 'tests.log') if (OUTPUT/name).exists()]
    manifest = {
        'build': re.search(r"BUILD='([^']+)'",(APP/'service.py').read_text(encoding='utf-8')).group(1),
        'application_version':metadata['version'].strip(),'frozen_package_preserved':True,
        'camera': 'CAM2 only', 'rotation_degrees': 180,
        'capture': [640, 480], 'preview': [320, 240],
        'decode':[640,480],'fast_decode':None,
        'rotation': 'Frozen1.0.3: rotate640x480 capture180 degrees before decode and preview',
        'uart': {'port': '/dev/ttyS0', 'baud': 115200, 'format': '8N1'},
        'protocol': {'mode': 6, 'station': 4, 'target': 0, 'flags': 0,
                     'result_type': 7, 'group_bytes': 32,
                     'sequence': 'successful SET request sequence'},
        'files': {str(p.relative_to(ROOT)).replace('\\', '/'): sha256(p) for p in paths},
        'verification': 'See validation.json; offline only, hardware not tested',
    }
    (OUTPUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    bundle = OUTPUT.with_suffix('.zip')
    with ZipFile(bundle, 'w', ZIP_DEFLATED) as archive:
        for name in ('main.py', 'README.md', app_zip.name, 'manifest.json', 'validation.json', 'tests.log'):
            path = OUTPUT / name
            if path.exists(): archive.write(path, name)
    return bundle


if __name__ == '__main__':
    print(package())
