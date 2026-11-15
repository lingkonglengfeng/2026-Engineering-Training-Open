"""Apply the measured red reference samples, preserving every other color."""
import argparse
import copy
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]
REFERENCE=ROOT/'vision/pc_vision/test_data/red_material_calibration.json'


def merge_red_samples(document):
    result=copy.deepcopy(document)
    red=json.loads(REFERENCE.read_text(encoding='utf-8'))['samples']['1']
    result.setdefault('version',1)
    result.setdefault('samples',{})['1']=red
    return result


def apply(calibration):
    document=json.loads(calibration.read_text(encoding='utf-8-sig')) if calibration.exists() else {}
    updated=merge_red_samples(document)
    if calibration.exists():
        backup=calibration.with_name(calibration.stem+'.before_red'+calibration.suffix)
        if not backup.exists():
            shutil.copy2(calibration,backup)
    calibration.parent.mkdir(parents=True,exist_ok=True)
    calibration.write_text(json.dumps(updated,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Updated only RED samples: '+str(calibration))
    print('Restart the PC vision app to reload; camera mode must be MATERIAL for detection.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration',type=Path,default=ROOT/'vision/pc_vision/calibration.json')
    apply(parser.parse_args().calibration)
