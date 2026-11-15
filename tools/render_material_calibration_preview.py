"""Render production Color/View/Learn UI with synthetic RGB shapes, not a device screenshot."""
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tmp/maixcam_cv_deps'),str(ROOT/'vision/maixcam_service')]
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import service

APP=ROOT/'vision/maixcam_service'
FONT=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',13)


class Frame:
    def __init__(self,picture):self.picture=picture
    def width(self):return self.picture.width
    def height(self):return self.picture.height
    def format(self):return 'rgb'
    def copy(self):return Frame(self.picture.copy())
    def binary(self,thresholds,copy=False):
        lab=cv2.cvtColor(np.array(self.picture),cv2.COLOR_RGB2LAB).astype(float)
        lab[:,:,0]*=100/255;lab[:,:,1:]-=128
        loL,hiL,loA,hiA,loB,hiB=thresholds[0]
        mask=((lab[:,:,0]>=loL)&(lab[:,:,0]<=hiL)&(lab[:,:,1]>=loA)&(lab[:,:,1]<=hiA)&(lab[:,:,2]>=loB)&(lab[:,:,2]<=hiB))
        self.picture=Image.fromarray(np.repeat((mask*255).astype(np.uint8)[:,:,None],3,axis=2))
    def get_statistics(self,roi):
        x,y,w,h=roi
        lab=cv2.cvtColor(np.array(self.picture)[y:y+h,x:x+w],cv2.COLOR_RGB2LAB).astype(float)
        values=[float(np.median(lab[:,:,0]))*100/255,float(np.median(lab[:,:,1]))-128,float(np.median(lab[:,:,2]))-128]
        return SimpleNamespace(l_median=lambda:values[0],a_median=lambda:values[1],b_median=lambda:values[2])
    def draw_rect(self,x,y,w,h,color,thickness=1):
        ImageDraw.Draw(self.picture).rectangle((x,y,x+w-1,y+h-1),
            fill=color if thickness<0 else None,outline=None if thickness<0 else color,width=max(1,thickness))
    def draw_string(self,x,y,text,color):ImageDraw.Draw(self.picture).text((x,y),text,font=FONT,fill=color)


def render():
    picture=Image.new('RGB',(320,240),(175,175,175));draw=ImageDraw.Draw(picture)
    draw.ellipse((118,93,202,177),fill=(20,70,180))
    draw.ellipse((25,107,86,168),fill=(30,120,50))
    draw.ellipse((230,90,294,154),fill=(48,169,210))
    frame=Frame(picture)
    module=service.detector_module('material')
    ui=service.MaterialCalibrationUI(module,{'3':[[60,35,-50]]*3,'6':[[60,-25,-35]]*3},None,3)
    api=SimpleNamespace(COLOR_WHITE=(255,255,255),COLOR_BLACK=(0,0,0),Format=SimpleNamespace(FMT_RGB888='rgb'))
    ui.action('learn',frame);ui.action('surface',frame,(145,115))
    stages=[('learn',ui.render(frame,api).picture)]
    ui.action('surface',frame,(175,130));ui.action('surface',frame,(155,150))
    stages.append(('rgb',ui.render(frame,api).picture))
    ui.action('view',frame);stages.append(('mask',ui.render(frame,api).picture))
    contact=Image.new('RGB',(960,240),'white')
    for index,(name,shown) in enumerate(stages):
        shown.resize((640,480),Image.Resampling.NEAREST).save(APP/f'calibration_{name}_preview.png')
        contact.paste(shown,(320*index,0))
    contact.resize((1920,480),Image.Resampling.NEAREST).save(APP/'calibration_ui_preview.png')
    print('Production UI preview: LEARN1/3, RGB3/3, MASK3/3 (synthetic, not hardware)')


if __name__=='__main__':render()
