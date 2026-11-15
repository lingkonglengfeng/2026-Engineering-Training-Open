"""Render the tested150mm fillet geometry as an SVG engineering diagram."""
from pathlib import Path
import math
ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'output/outer_arc_v130_2026-10-13';out.mkdir(parents=True,exist_ok=True)
scale=.29;margin=65
def xy(x,y):return margin+x*scale,margin+(2400-y)*scale
def line(points,color,width=3,dash=''):
    coords=' '.join(f'{a:.2f},{b:.2f}' for a,b in map(lambda p:xy(*p),points))
    return f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>'
def rect(x0,y0,x1,y1,color):
    x,y=xy(x0,y1);return f'<rect x="{x}" y="{y}" width="{(x1-x0)*scale}" height="{(y1-y0)*scale}" fill="{color}"/>'
def text(x,y,s,size=13,color='#243142'):
    a,b=xy(x,y);return f'<text x="{a}" y="{b}" font-size="{size}" fill="{color}">{s}</text>'
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="850" height="870" viewBox="0 0 850 870">','<rect width="850" height="870" fill="#f7f9fc"/>','<g font-family="Segoe UI, Microsoft YaHei, sans-serif">',
     '<text x="65" y="28" font-size="21" fill="#172b42">V130 外环减速圆弧（几何示意，非实测轨迹）</text>',
     '<text x="65" y="49" font-size="13">R150mm · 弯道上限140mm/s · 车身280×280mm · 蓝色连续轨迹 / 红色原直角</text>',rect(0,0,2400,2400,'#fff')]
for x in (550,1400):
 for y in (550,1400):svg.append(rect(x,y,x+450,y+450,'#f9d981'))
for v in ((1000,2315,1400,2400),(1050,0,1350,150),(0,1050,150,1350),(2350,1050,2400,1350)):svg.append(rect(*v,'#afb9c6'))
routes=[((1150,400),(350,400),(350,1200),'SW'),((350,1280),(350,2100),(1250,2100),'NW'),((2125,1150),(2125,2100),(1250,2100),'NE')]
for start,corner,end,label in routes:
    cx,cy=corner;r=150;u0=(cx-start[0],cy-start[1]);n=math.hypot(*u0);ux,uy=u0[0]/n,u0[1]/n
    v0=(end[0]-cx,end[1]-cy);n=math.hypot(*v0);wx,wy=v0[0]/n,v0[1]/n
    turn=ux*wy-uy*wx;center=(cx+r*(wx-ux),cy+r*(wy-uy));arc=[]
    for i in range(41):
      a=turn*math.pi*.5*i/40;c,s=math.cos(a),math.sin(a)
      arc.append((center[0]-r*(wx*c-wy*s),center[1]-r*(wx*s+wy*c)))
    svg.append(line([start,corner,end],'#dd6767',2,'6 5'));svg.append(line([start,*arc,end],'#176dde',4))
    for i in (0,10,20,30,40):
      yaw=math.atan2(uy,ux)+turn*math.pi*.5*i/40;c,s=math.cos(yaw),math.sin(yaw);px,py=arc[i]
      body=[(px+c*x-s*y,py+s*x+c*y) for x,y in ((-140,-140),(140,-140),(140,140),(-140,140),(-140,-140))]
      svg.append(line(body,'#548abd',1))
    svg.append(text(cx+20,cy+35,f'{label} R150'))
svg += [text(1080,2270,'RAW'),text(1080,170,'ROUGH'),text(165,1180,'TEMP'),text(2130,1200,'QR'),
        text(600,1720,'禁行区'),text(1470,1720,'禁行区'),text(600,760,'禁行区'),text(1470,760,'禁行区')]
for v in range(0,2401,400):
 a,b=xy(v,0);svg.append(f'<text x="{a-12}" y="{b+22}" font-size="12">{v}</text>')
 a,b=xy(0,v);svg.append(f'<text x="{a-45}" y="{b+4}" font-size="12">{v}</text>')
svg += ['<text x="65" y="807" font-size="13">示例显示常用SW/NW/NE方向；程序支持四角双向，只有整车通行检查通过才接圆弧。</text>',
        '<text x="65" y="830" font-size="13">起始对向、工位最终停靠和不适用拐角仍沿用原流程；转盘台面按入场85mm绘制。</text>','</g></svg>']
(out/'outer_arc_preview.svg').write_text('\n'.join(svg),encoding='utf-8')
print(out/'outer_arc_preview.svg')
