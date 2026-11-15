"""Package current CAM1 service; leaves CAM2, old releases and MCU files alone."""
import hashlib
import json
import re
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED
from build_vision_service import build,ROOT

APP=ROOT/'vision/maixcam_service'
OUT=ROOT/'output/ring_occupied_opt_2026-10-15'


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def package():
    main=build()
    compile(main.read_text(encoding='utf-8'),str(main),'exec')
    OUT.mkdir(parents=True,exist_ok=True)
    dist=APP/'dist';dist.mkdir(exist_ok=True)
    meta=dict(line.split(':',1) for line in (APP/'app.yaml').read_text(encoding='utf-8').splitlines() if ':' in line)
    version=meta['version'].strip()
    appzip=dist/('maix-%s-v%s.zip'%(meta['id'].strip(),version))
    with ZipFile(appzip,'w',ZIP_DEFLATED) as archive:
        for name in ('app.png','app.yaml','main.py'):archive.write(APP/name,name)
    for path in (main,APP/'README.md',appzip):shutil.copy2(path,OUT/path.name)
    shutil.copy2(APP/'material_calibration_guide.md',OUT/'material_calibration_guide.md')
    shutil.copy2(APP/'occupied_resources_handoff.md',OUT/'occupied_resources_handoff.md')
    shutil.copy2(APP/'dist/maix-v1-v1.1.14.zip',OUT/'rollback_maix-v1-v1.1.14.zip')
    for picture in APP.glob('calibration_*preview.png'):shutil.copy2(picture,OUT/picture.name)
    paths=[APP/'service.py',main,APP/'app.yaml',APP/'app.png',APP/'README.md',appzip,
           ROOT/'vision/maixcam_ring/middle_ring.py',ROOT/'vision/maixcam_ring/test_ring_profiles.py',
           APP/'test_ring_profiles_service.py',ROOT/'tools/validate_ring_profiles.py',Path(__file__),
           APP/'test_ring_light_settings.py',ROOT/'tools/validate_ring_pwm25.py',
           APP/'test_material_calibration.py',ROOT/'tools/validate_material_calibration.py',
           ROOT/'vision/maixcam_ring/test_occupied_resources.py',ROOT/'tools/validate_occupied_resources.py',
           APP/'material_calibration_guide.md',
           APP/'occupied_resources_handoff.md',
           OUT/'validation.json',OUT/'tests.log']
    paths.extend(OUT.glob('calibration_*preview.png'))
    build_id=re.search(r'SERVICE_BUILD = "([^"]+)"',(APP/'service.py').read_text(encoding='utf-8')).group(1)
    manifest={'build':build_id,'application_version':version,
              'requires':'MCU V171 V155 alignment restored; legacy flags3; CAM2 unchanged',
              'baseline':'Frozen CAM1 1.1.14 model/protocol; occupied resource optimization only',
              'flags':{'0':'EMPTY','1':'EMPTY+AXIS','2':'OCCUPIED','3':'OCCUPIED+AXIS'},
              'ring_context':{'mode':3,'station':[2,3],'target':2,'size':[480,360]},
              'result_types':['03','0A (bit0 only)','83'],'hardware_run':False,
              'ring_count_reserved_byte':0,'new_flags08':False,
              'light_pwm':{'pin':'A19','channel':7,'frequency_hz':20000,'default_duty_percent':50,'ring_duty_percent':15},
              'isp_luma_changed':False,'exposure_changed':False,
              'material_calibration':{'mode':2,'station':1,'flag':4,'pixels_valid':False,
                  'ui':['Color','View','Learn','Save','Exit'],'points':3,'preview':'RGB/MASK',
                  'exit':'save then CAL DONE waiting MCU STOP','file':'~/maixcam_material_colors.json'},
              'files':{str(p.relative_to(ROOT)).replace('\\','/'):digest(p) for p in paths}}
    (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    bundle=OUT.with_suffix('.zip')
    with ZipFile(bundle,'w',ZIP_DEFLATED) as archive:
        for path in sorted(OUT.iterdir()):
            if path.is_file() and path.name not in ('before_service.py','presence_v29_test_snapshot.py'):
                archive.write(path,path.name)
    print(bundle)
    print('main SHA256',digest(main))
    print('app SHA256',digest(appzip))
    print('bundle SHA256',digest(bundle))
    return bundle


if __name__=='__main__':package()
