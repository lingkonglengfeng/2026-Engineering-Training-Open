"""Compare executable-mm plans with/without the optional outer-centre pass."""
from pathlib import Path
import shutil,subprocess,tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR
ROOT=Path(__file__).resolve().parents[2]
FIXTURE=r'''
#include "grid_nav.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
volatile uint32_t rows[10][6],vertices[10][24][2];
static uint8_t center_accepted;
void ZDT_Console_Print(const char *text){if(strstr(text,"OUTER_CENTER accepted"))center_accepted=1;}
int main(void){
 const uint16_t poses[10][4]={{1240,2110,1200,355},{1200,355,350,1250},{350,1250,1250,2100},{2125,1130,1250,2100},{2125,1130,1250,2100},{1240,2110,350,355},{1240,2110,1200,355},{1200,355,350,1250},{350,1250,1250,2100},{2125,1130,1250,2100}};
 const float yaw[10]={0,180,90,0,0,0,0,180,90,0};
 static grid_nav_point_t path[GRID_NAV_MAX_PATH];static grid_nav_segment_t seg[GRID_NAV_MAX_SEGMENTS];
 for(unsigned k=0;k<10;k++){
  uint16_t n=0,x,y,tx,ty,count=0;uint8_t gx,gy;GridNav_Init();GridNav_SetPoseMm(poses[k][0],poses[k][1]);GridNav_SetInitialHeading(yaw[k]);
  if(k==4){uint16_t ox[2]={2125,1200},oy[2]={1550,762};GridNav_SetManualObstaclesMm(ox,oy,2);}
  if(k==0){uint16_t ox[2]={1200,2125},oy[2]={762,1700};GridNav_SetManualObstaclesMm(ox,oy,2);}
  if(k>=6){uint16_t ox[2]={1200,2096},oy[2]={1210,1709};GridNav_SetManualObstaclesMm(ox,oy,2);}
  center_accepted=0;rows[k][0]=GridNav_PlanTargetMm(poses[k][2],poses[k][3]);
  if(center_accepted && GridNav_GetSegments(seg,GRID_NAV_MAX_SEGMENTS,&count)) {
   int32_t px=poses[k][0],py=poses[k][1];
   for(unsigned i=0;i<count;i++){unsigned d=GridNav_GetSegmentDistanceMm(i);px+=seg[i].dx*(int32_t)d;py-=seg[i].dy*(int32_t)d;}
   if(px!=poses[k][2] || py!=poses[k][3])rows[k][0]=0;
  }
  if(GridNav_GetPlan(path,GRID_NAV_MAX_PATH,&n,&gx,&gy)){
   rows[k][1]=(n-1)*50;
   for(unsigned i=1;i<n;i++){
    x=GridNav_CellCenterMmX(path[i].x);y=GridNav_CellCenterMmY(path[i].y);unsigned d=2400,v;
    if(x<550)d=abs((int)x-275);else if(x>=1850)d=abs((int)x-2125);
    if(y<550){v=abs((int)y-275);if(v<d)d=v;}else if(y>=1850){v=abs((int)y-2125);if(v<d)d=v;}
    if(d<2400)rows[k][2]+=d;
   }
  }
  rows[k][3]=GridNav_PlanCheckedTargetMmHeading(poses[k][2],poses[k][3],yaw[k]);
  if(rows[k][3] && GridNav_GetSegments(seg,GRID_NAV_MAX_SEGMENTS,&count)){
   int32_t px=poses[k][0],py=poses[k][1];vertices[k][0][0]=px;vertices[k][0][1]=py;
   rows[k][4]=count;
   for(unsigned i=0;i<count;i++){unsigned d=GridNav_GetSegmentDistanceMm(i);px+=seg[i].dx*(int32_t)d;py-=seg[i].dy*(int32_t)d;
    if(i<23){vertices[k][i+1][0]=(uint32_t)px;vertices[k][i+1][1]=(uint32_t)py;}rows[k][5]+=d;}
   GridNav_GetTargetMm(&tx,&ty);if(px!=tx || py!=ty)rows[k][3]=0;
  }
 }
 return 0;
}
'''
def run(work,source,label):
    (work/'main.c').write_text(FIXTURE,encoding='utf-8');binary=work/(label+'.elf')
    includes=('Core/Inc','Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32H7xx/Include','Drivers/CMSIS/Include')
    subprocess.run([shutil.which('arm-none-eabi-gcc'),'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-DSTM32H723xx','-DUSE_HAL_DRIVER',*['-I'+str(ROOT/p) for p in includes],'-ffunction-sections','-fdata-sections',str(work/'main.c'),str(source),'-nostartfiles','--specs=nosys.specs','-Wl,--gc-sections','-Wl,-T,'+str(work/'test.ld'),'-lm','-o',str(binary)],check=True)
    emu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);emu.mem_map(0x08000000,0x200000);emu.mem_map(0x20000000,0x200000)
    with binary.open('rb') as stream:
        elf=ELFFile(stream)
        for s in elf.iter_segments():
            if s['p_type']=='PT_LOAD' and s['p_filesz']:emu.mem_write(s['p_vaddr'],s.data())
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()};entry=elf['e_entry']
    sentinel=0x081ff000;done=[]
    def finish(cpu,address,size,user):done.append(True);cpu.emu_stop()
    emu.hook_add(UC_HOOK_CODE,finish,begin=sentinel,end=sentinel);emu.reg_write(UC_ARM_REG_SP,0x201ff000);emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=120_000_000,count=6_000_000_000)
    if not done:raise SystemExit(label+' did not finish')
    def read(name,count):return [int.from_bytes(emu.mem_read(symbols[name]+i*4,4),'little') for i in range(count)]
    values=read('rows',60);points=read('vertices',480)
    result=[values[i:i+6] for i in range(0,60,6)]
    print(label+' ordinary_ok/cell_mm/outer_deviation/checked_ok/segments/exact_mm:',result)
    routes=[[(points[k*48+i*2],points[k*48+i*2+1]) for i in range(min(row[4]+1,24))] for k,row in enumerate(result)]
    for k,route in enumerate(routes):print(label,k,'vertices',route)
    return result,routes
with tempfile.TemporaryDirectory(prefix='outer-center-') as directory:
    work=Path(directory);(work/'test.ld').write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    source=ROOT/'Core/Src/grid_nav.c';baseline=work/'baseline.c'
    baseline.write_text(source.read_text(encoding='utf-8').replace('  GridNav_RefineOuterCenter(sx,sy,gx,gy,path,length);',''),encoding='utf-8')
    old,_=run(work,baseline,'baseline');new,routes=run(work,source,'center')
    for k in range(10):
        assert new[k][0] and new[k][3],(k,new[k])
        assert new[k][1]<=old[k][1]+600,(k,old[k],new[k])
        assert new[k][5]<=old[k][5]+600,(k,old[k],new[k])
        assert new[k][2]<=old[k][2],(k,old[k],new[k])
    assert all(new[k][2]<old[k][2] for k in (0,1,5,6,7,9))
    # Two real obstacles force a long outer detour: keep long tangents near
    # the feasible centre, with at most one PAD-clearance jog on the left.
    assert new[6][4]<=6 and new[9][4]<=6
    for k in (6,9):
        legs=list(zip(routes[k],routes[k][1:]))
        left=[a[0] for a,b in legs if a[0]==b[0] and abs(b[1]-a[1])>=600 and a[0]<550]
        bottom=[a[1] for a,b in legs if a[1]==b[1] and abs(b[0]-a[0])>=600 and a[1]<550]
        assert left and all(275<=x<=325 for x in left),(k,routes[k])
        assert bottom and all(275<=y<=325 for y in bottom),(k,routes[k])
    assert any(a[1]>=2075 and a[1]==b[1] and abs(b[0]-a[0])>=600 for a,b in zip(routes[6],routes[6][1:]))
    print('Outer centring: ten real-mm routes/endpoint/swept-body checks passed; no IO')
