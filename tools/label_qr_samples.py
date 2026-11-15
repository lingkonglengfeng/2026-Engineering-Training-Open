"""Create labeled A4 print pages around the frozen competition QR codes.

Keep the central 80mm QR square (including its quiet zone) untouched.
Requires Pillow and OpenCV; changes no camera or firmware files.
"""
from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from zipfile import ZipFile, ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tmp/maixcam_cv_deps'))
from PIL import Image, ImageDraw, ImageFont
import cv2
import numpy as np

SOURCE=ROOT/'output/qr_samples_v103_2026-10-09'
DEST=ROOT/'output/qr_samples_labeled_2026-10-09'
SCALE=10  # 254 DPI: exactly 10 pixels per mm
NAMES={1:'红色',2:'黄色',3:'蓝色',4:'绿色',5:'黑色',6:'浅蓝色'}
INK={1:'#d52c32',2:'#e0b500',3:'#2455be',4:'#24824b',5:'#252525',6:'#56b9df'}
FONT=Path('C:/Windows/Fonts/msyh.ttc')
FONT_BOLD=Path('C:/Windows/Fonts/msyhbd.ttc')
NS='http://www.w3.org/2000/svg'
ET.register_namespace('',NS)


def run():
    (DEST/'PNG_A4').mkdir(parents=True,exist_ok=True)
    (DEST/'SVG_A4').mkdir(exist_ok=True)
    original=json.loads((SOURCE/'manifest.json').read_text(encoding='utf-8'))
    records=[];thumbs=[]
    for sample in original['samples']:
        page=Image.new('RGB',(2970,2100),'white');draw=ImageDraw.Draw(page)
        qr=Image.open(SOURCE/sample['png']).convert('RGB').resize((800,800),Image.Resampling.NEAREST)
        page.paste(qr,(1085,650))
        svg=ET.parse(SOURCE/sample['a4_svg']).getroot()
        # Replace the old unlabelled payload caption, keeping all QR rectangles unchanged.
        for element in list(svg):
            if element.tag==f'{{{NS}}}text':svg.remove(element)

        def text(x,y,content,size=4.2,center=False,bold=False):
            font=ImageFont.truetype(str(FONT_BOLD if bold else FONT),round(size*SCALE))
            draw.text((round(x*SCALE),round(y*SCALE)),content,font=font,fill='#252525',anchor='mm' if center else 'lm')
            element=ET.SubElement(svg,f'{{{NS}}}text',{
                'x':str(x),'y':str(y),'font-family':'Microsoft YaHei, Noto Sans CJK SC, sans-serif',
                'font-size':str(size),'fill':'#252525','dominant-baseline':'central',
                'text-anchor':'middle' if center else 'start'})
            if bold:element.set('font-weight','bold')
            element.text=content

        def swatch(x,y,color):
            draw.rounded_rectangle((round(x*SCALE),round((y-2)*SCALE),round((x+4)*SCALE),round((y+2)*SCALE)),radius=4,fill=color)
            ET.SubElement(svg,f'{{{NS}}}rect',{'x':str(x),'y':str(y-2),'width':'4','height':'4','rx':'0.4','fill':color})

        text(148.5,20,f'任务二维码  {sample["index"]:02d}',7,True,True)
        text(148.5,33,'数字为比赛颜色编号；箭头后的数字为放置圆环编号',4,True)
        groups=sample['payload'].split('+')
        for batch,x in enumerate((16,201)):
            text(x,56,f'第{batch+1}批物料',5.5,bold=True)
            text(x,66,f'颜色 {groups[batch*2]}  /  环号 {groups[batch*2+1]}',4)
            for slot,(color,position) in enumerate(zip(sample['colors_rule'][batch],sample['positions'][batch])):
                y=84+23*slot
                swatch(x,y,INK[color])
                text(x+7,y,f'第{slot+1}件：{color} {NAMES[color]}',4.3,bold=True)
                text(x+7,y+8,f'→ 放置在 {position} 号圆环',4.1)
        text(148.5,159,sample['payload'],6,True,True)
        text(148.5,174,'颜色编号对照（比赛规则）',4,True)
        for index,color in enumerate(range(1,7)):
            x=30+40*index
            swatch(x,186,INK[color]);text(x+6,186,f'{color} {NAMES[color]}',4)
        text(148.5,201,'A4横向 · 原尺寸100%打印 · 中央二维码含静区80×80 mm',3.5,True)
        name=Path(sample['png']).stem
        png=DEST/'PNG_A4'/f'{name}.png';vector=DEST/'SVG_A4'/f'{name}.svg'
        page.save(png,dpi=(254,254))
        ET.ElementTree(svg).write(vector,encoding='utf-8',xml_declaration=True)
        np.testing.assert_array_equal(np.array(page)[650:1450,1085:1885],np.array(qr))
        # QR detection on complete labeled print pages, including upside-down capture.
        arr=cv2.cvtColor(np.array(page),cv2.COLOR_RGB2BGR)
        arr=cv2.resize(arr,(1485,1050),interpolation=cv2.INTER_AREA)
        for image in (arr,cv2.rotate(arr,cv2.ROTATE_180)):
            decoded=cv2.QRCodeDetector().detectAndDecode(image)[0]
            assert decoded==sample['payload'],(sample['payload'],decoded)
        records.append({'payload':sample['payload'],'png':str(png.relative_to(DEST)),
            'svg':str(vector.relative_to(DEST)),'source_qr_sha256':sample['png_sha256'],
            'png_sha256':hashlib.sha256(png.read_bytes()).hexdigest(),
            'central_qr_unchanged':True,'decode_labeled_page_upright':True,'decode_labeled_page_180':True})
        thumbs.append(page.resize((742,525),Image.Resampling.LANCZOS))
    sheet=Image.new('RGB',(1484,2100),'#dddddd')
    for index,thumb in enumerate(thumbs):sheet.paste(thumb,((index%2)*742,(index//2)*525))
    sheet.save(DEST/'总览.jpg',quality=93)
    (DEST/'manifest.json').write_text(json.dumps({'samples':records,'qr_size_mm':80,
        'page_mm':[297,210],'dpi':254,'rule_colors':NAMES,'note':'No camera/firmware change. Labels outside original QR quiet zone.'},ensure_ascii=False,indent=2),encoding='utf-8')
    (DEST/'打印说明.txt').write_text('原8张任务码保持不变。每页左右标注两批物料数字、中文颜色及对应放置环号，底部有颜色编号对照。\nPNG_A4为带254DPI信息的A4横向打印图片；SVG_A4为可缩放打印模板。请选择A4横向、原尺寸100%，关闭适应页面。中央二维码连静区仍80×80mm。\n两轮TEMP码垛映射由主控处理，本页“环号”解释原二维码的位置组。\n8张带字整页均已通过正向与180度共16次离线扫码验证；打印后仍需现场试扫。\n',encoding='utf-8')
    archive=DEST.with_suffix('.zip')
    with ZipFile(archive,'w',ZIP_DEFLATED) as zipped:
        for path in sorted(DEST.rglob('*')):
            if path.is_file():zipped.write(path,path.relative_to(DEST))
    print(f'{archive}\n8 labeled pages, 16 full-page decode checks PASS')


if __name__=='__main__':run()
