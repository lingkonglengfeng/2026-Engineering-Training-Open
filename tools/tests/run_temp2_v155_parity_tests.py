"""Compare the final TEMP policy against frozen V155 executable C, not a reimplementation."""
from pathlib import Path
import ast,hashlib,json,re,shutil,subprocess,tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR
r=Path(__file__).resolve().parents[2]
base=r/'output/plate_hold_clear_v155_2026-10-14'
old_patch=(base/'source_changes_from_V129.diff').read_text(encoding='utf-8')
helper=next(x for x in ast.parse((r/'tmp/package_lift_brake_v158.py').read_text(encoding='utf-8')).body if isinstance(x,ast.FunctionDef) and x.name=='baseline_text')
exec(compile(ast.Module(body=[helper],type_ignores=[]),'<frozen-source>','exec'))
raw=baseline_text('Core/Src/vision_align.c');old=raw.replace('\r\n','\n')
m=json.loads((base/'manifest.json').read_text(encoding='utf-8'))
assert m['source_sha256']['Core/Src/vision_align.c'] in [hashlib.sha256(s.encode()).hexdigest() for s in (raw,old,old.replace('\n','\r\n'))]
exports=re.findall(r'^(?:void|uint8_t)\s+(VisionAlign_\w+)\(',old,re.M)
renames='\n'.join('#define '+s+' Old_'+s for s in exports)+'\n'
test=r'''
#include "vision_align.h"
#include <math.h>
#include <string.h>
volatile unsigned checks,failures,first_failure;
#define CHECK(x) do{checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
uint8_t Old_VisionAlign_StationConfig(vision_align_config_t*,uint8_t);
uint8_t Old_VisionAlign_Start(vision_align_t*,const vision_align_config_t*,uint32_t,uint32_t);
void Old_VisionAlign_StepScene(vision_align_t*,uint32_t,const vision_pixel_t*,uint32_t,const vision_ring_count_t*,uint32_t,uint8_t,float*,float*);
int main(void){unsigned scenario,k;vision_align_config_t c,old;
 CHECK(VisionAlign_StationConfig(&c,3));CHECK(Old_VisionAlign_StationConfig(&old,3));
 c.tolerance_px=old.tolerance_px=10;c.stable_frames=old.stable_frames=3;
 CHECK(c.search_speed==0.04f);
 CHECK(memcmp(&c,&old,sizeof(c))==0);
 for(scenario=0;scenario<10;scenario++) {
  vision_align_t a,b;vision_pixel_t p={0};vision_ring_count_t s={0};uint32_t now=10000,age=0;float f,g,of,og;
  CHECK(VisionAlign_Start(&a,&c,now,0));CHECK(Old_VisionAlign_Start(&b,&old,now,0));
  p.type=VISION_TYPE_RING_PX;p.id=2;s.id=2;s.station=3;
  for(k=0;k<180;k++) {
   now+=100;p.observation_id=s.observation_id=k+1;p.seq=s.seq=(uint8_t)(k%16);
   s.count=3;s.valid=1;p.valid=1;p.dx_px=30;p.dy_px=-15;age=0;
   switch(scenario){
   case 0:s.count=0;p.valid=0;break;
   case 1:s.count=1;p.valid=0;break;
   case 2:s.count=2;p.valid=0;break;
   case 3:s.valid=0;p.valid=0;break;
   case 4:p.valid=0;break;
   case 5:if(k<20){s.count=2;p.valid=0;}else if(k<25)p.valid=0;else if(k>=40)p.dx_px=p.dy_px=0;break;
   case 6:p.dx_px=p.dy_px=0;break;
   case 7:age=450;break;
   case 8:s.station=2;break;
   case 9:s.seq=(uint8_t)((p.seq+1)%16);break;
   }
   /* The optional v29 flag cannot change the reverted non-opt-in policy. */
   s.green_visible=(uint8_t)(k%2);
   VisionAlign_StepScene(&a,now,&p,age,&s,age,(uint8_t)(k%3!=0),&f,&g);
   Old_VisionAlign_StepScene(&b,now,&p,age,&s,age,(uint8_t)(k%3!=0),&of,&og);
   CHECK(a.state==b.state);CHECK(fabsf(f-of)<1e-6f && fabsf(g-og)<1e-6f);
   CHECK(a.stable_count==b.stable_count && a.drive_ms==b.drive_ms && a.search_ms==b.search_ms);
   CHECK(a.driving==b.driving && a.searching==b.searching && a.target_locked==b.target_locked);
   CHECK(a.last_valid==b.last_valid && fabsf(a.error_x-b.error_x)<1e-6f && fabsf(a.error_y-b.error_y)<1e-6f);
   if(scenario==0 || scenario==1 || scenario==3 || scenario==4)CHECK(f==0 && g==0);
  }
 }
 return failures;
}
'''
with tempfile.TemporaryDirectory(prefix='temp2-v155-parity-') as folder:
    w=Path(folder);(w/'old.c').write_text(renames+old,encoding='utf-8');(w/'test.c').write_text(test,encoding='utf-8')
    (w/'test.ld').write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    elf=w/'test.elf'
    subprocess.run([shutil.which('arm-none-eabi-gcc'),'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror','-I'+str(r/'Core/Inc'),'-ffunction-sections','-fdata-sections',str(w/'test.c'),str(w/'old.c'),str(r/'Core/Src/vision_align.c'),'-nostartfiles','--specs=nosys.specs','-Wl,--gc-sections','-Wl,-T,'+str(w/'test.ld'),'-lm','-o',str(elf)],check=True)
    emu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);emu.mem_map(0x08000000,0x100000);emu.mem_map(0x20000000,0x100000)
    with elf.open('rb') as stream:
        e=ELFFile(stream)
        for seg in e.iter_segments():
            if seg['p_type']=='PT_LOAD' and seg['p_filesz']:emu.mem_write(seg['p_vaddr'],seg.data())
        symbols={s.name:s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()};entry=e['e_entry']
    done=[];sentinel=0x080FF000
    def returned(machine,address,size,user):done.append(True);machine.emu_stop()
    emu.hook_add(UC_HOOK_CODE,returned,begin=sentinel,end=sentinel)
    emu.reg_write(UC_ARM_REG_SP,0x200FF000);emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=30_000_000,count=30_000_000)
    value=lambda name:int.from_bytes(emu.mem_read(symbols[name],4),'little')
    print(f"TEMP2 frozen V155 parity: {value('checks')} checks, {value('failures')} failures; first line={value('first_failure')}")
    if not done or value('failures'):raise SystemExit('V155 parity failed')
