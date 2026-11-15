"""Generate eight preliminary-rule task codes and verify their actual pixels."""
from pathlib import Path
import hashlib,json,secrets,sys
import qrcode
from PIL import Image
import cv2
import numpy as np

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'vision/maixcam_qr'))
from service import parse_task
out=root/'output/qr_samples_v103_2026-10-09'
out.mkdir(parents=True,exist_ok=True)
if list(out.glob('*.png')):raise SystemExit('Sample folder already exists; keep its frozen payloads/manifest instead of adding new random images.')
panels=out/'A4_landscape_80mm';panels.mkdir(exist_ok=True)
rng=secrets.SystemRandom();seen=set();samples=[]
for index in range(1,9):
    while True:
        colors=rng.sample(range(1,7),3)
        batch2=rng.sample(colors,3)
        p1=rng.sample([1,2,3],3);p2=rng.sample([1,2,3],3)
        payload='+'.join(''.join(map(str,g)) for g in (colors,p1,batch2,p2))
        if payload not in seen:seen.add(payload);break
    assert parse_task(payload) is not None
    qr=qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M,box_size=30,border=4)
    qr.add_data(payload);qr.make(fit=True)
    im=qr.make_image(fill_color='black',back_color='white').convert('RGB')
    name=f'{index:02d}_{payload.replace("+","_")}.png';png=out/name
    dpi=im.width*25.4/80.0
    im.save(png,dpi=(dpi,dpi))
    arr=cv2.cvtColor(np.array(im),cv2.COLOR_RGB2BGR)
    assert cv2.QRCodeDetector().detectAndDecode(arr)[0]==payload
    assert cv2.QRCodeDetector().detectAndDecode(cv2.rotate(arr,cv2.ROTATE_180))[0]==payload
    matrix=qr.get_matrix();n=len(matrix);unit=80/n
    rectangles=''.join(f'<rect x="{(108.5+x*unit):.6f}" y="{(65+y*unit):.6f}" width="{unit:.6f}" height="{unit:.6f}"/>' for y,row in enumerate(matrix) for x,black in enumerate(row) if black)
    svg=panels/(png.stem+'.svg')
    svg.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="297mm" height="210mm" viewBox="0 0 297 210"><rect width="297" height="210" fill="white"/><g fill="black" shape-rendering="crispEdges">{rectangles}</g><text x="148.5" y="160" text-anchor="middle" font-family="monospace" font-size="5">{payload}</text></svg>',encoding='utf-8')
    samples.append(dict(index=index,payload=payload,png=name,png_sha256=hashlib.sha256(png.read_bytes()).hexdigest(),a4_svg=str(svg.relative_to(out)),colors_rule=[colors,batch2],positions=[p1,p2],size_mm=80,quiet_zone_modules=4,png_dpi=dpi,decoded_upright=True,decoded_rotated180=True))
(out/'manifest.json').write_text(json.dumps(dict(date='2026-10-09',rules='docs/electrical/rules.pdf pages5-8',rule_ids={'1':'red','2':'yellow','3':'blue','4':'green','5':'black','6':'lightblue'},samples=samples),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(out/'说明.txt').write_text('8张随机任务码，四组三位数以+分隔。两批使用相同的三种不同颜色，各自顺序和环号随机。\nPNG含4模块静区，按文件DPI以80×80mm原尺寸打印。A4_landscape_80mm内SVG为A4横向模板，二维码位于中央且包含静区总宽80mm；打印选择原尺寸/100%，关闭适应页面。\n颜色规则编号：1红2黄3蓝4绿5黑6浅蓝；不要将颜色编号与线上5/6映射混淆。\n已逐张用真实图像解码验证正向与180°旋转；实机焦距和光照需现场确认。\n',encoding='utf-8')
print(json.dumps({'folder':str(out),'payloads':[s['payload'] for s in samples],'verified':16},ensure_ascii=False))
