"""Sampling-first delivery; independent CAM2 update, no MCU firmware changes."""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

from build_vision_sampler import build
from package_qr_service import package

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/vision_sampling_qr_fixed_rollback_2026-10-13'


def main():
    standalone=build()
    qr_bundle=package()
    OUT.mkdir(parents=True,exist_ok=True)
    for source,dest in [(standalone,'vision_sample.py'),
        (ROOT/'vision/maixcam_sampler/README.md','采样使用说明.md'),
        (ROOT/'output/vision_sampler_checked_2026-10-13/tests.log','sampler_tests.log'),
        (ROOT/'output/vision_sampler_checked_2026-10-13/validation.json','sampler_validation.json'),
        (ROOT/'vision/maixcam_qr/main.py','CAM2/main.py'),
        (ROOT/'vision/maixcam_qr/README.md','CAM2/README.md'),
        (ROOT/'vision/maixcam_qr/dist/maix-cam2_qr-v1.0.3.zip','CAM2/maix-cam2_qr-v1.0.3.zip'),
        (ROOT/'vision/maixcam_qr/dist/maix-cam2_qr-v1.0.5.zip','CAM2/reference/experimental_v1.0.5.zip'),
        (ROOT/'output/cam2_fixed_rollback_v103_2026-10-13/validation.json','CAM2/validation.json'),
        (ROOT/'output/cam2_fixed_rollback_v103_2026-10-13/tests.log','CAM2/tests.log')]:
        target=OUT/dest;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    s=(OUT/'sampler_tests.log').read_text(encoding='utf-8',errors='replace')
    assert 'Ran 12 tests' in s and '\nOK' in s
    v=json.loads((OUT/'CAM2/validation.json').read_text(encoding='utf-8'))
    assert v['unit_tests']==27 and not v['unit_failures'] and not v['unit_errors'] and len(v['sample_cases'])==8
    for p in (OUT/'vision_sample.py',OUT/'CAM2/main.py'):compile(p.read_text(encoding='utf-8'),str(p),'exec')
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest=dict(date='2026-10-13',primary='CAM1 camera/lamp-only sampling preserved',secondary='Frozen CAM2 1.0.3 rollback',
                  mcu_changed=False,hardware_tested=False,actual_maix_fps_measured=False,
                  sampler_checks=12,qr_checks=27,qr_sample_cases=8,
                  sampler_capture=[480,360],ring_lamp_percent=15,plate_lamp_percent=50,
                  qr_capture=[640,480],qr_decode=[640,480],qr_fast_decode=None,
                  qr_confirm_frames=3,qr_preview_rotation=180,
                  files={str(p.relative_to(OUT)).replace('\\','/'):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name!='manifest.json'})
    (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    bundle=OUT.with_suffix('.zip')
    with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(OUT.rglob('*')):
            if p.is_file():z.write(p,str(p.relative_to(OUT)).replace('\\','/'))
    with zipfile.ZipFile(bundle) as z:
        assert z.testzip() is None
        for p,digest in manifest['files'].items():assert hashlib.sha256(z.read(p)).hexdigest()==digest,p
        assert json.loads(z.read('manifest.json'))==manifest
    print(json.dumps(dict(bundle=str(bundle),files_verified=len(manifest['files']),cam2_bundle=str(qr_bundle)),ensure_ascii=False))


if __name__=='__main__':main()
