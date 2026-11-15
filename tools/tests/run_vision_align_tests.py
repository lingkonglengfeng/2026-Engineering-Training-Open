"""Run the production XY regressions and actual per-stage config, without IO."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT/'Core/Src/zdt_motor.c').read_text(encoding='utf-8')
start = source.index('static uint8_t ZDT_Align_ConfigForStage(uint8_t station,vision_align_config_t *out)\n{')
brace = source.index('{', start); end = brace+1; depth = 1
while depth:
    depth += (source[end] == '{')-(source[end] == '}'); end += 1
function = source[start:end]
syscalls = (ROOT/'tools/tests/test_arm_driver.c').read_text(encoding='utf-8')
syscalls = syscalls[syscalls.index('int _close('):syscalls.index('volatile unsigned int test_checks')]
fixture = r'''
#include "vision_align.h"
#include "match_control.h"
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t test_checks,test_failures,test_first_failure;
static vision_align_config_t zdt_align_config[3];
static match_control_t zdt_match;
static uint8_t zdt_manual_align_station;
#define CHECK(x) do{test_checks++;if(!(x)){test_failures++;if(!test_first_failure)test_first_failure=__LINE__;}}while(0)
/* STAGE_FUNCTION */
void Test_VisionAlignment(void);
void Test_SmoothVisionAlignment(void);
int main(void){unsigned station;vision_align_config_t c;
 Test_VisionAlignment();Test_SmoothVisionAlignment();
 for(station=1;station<=3;station++){
   CHECK(VisionAlign_StationConfig(&zdt_align_config[station-1],station));
   zdt_match.checkpoint=MATCH_TEMP1;
   CHECK(ZDT_Align_ConfigForStage(station,&c));
   CHECK(c.tolerance_px==(station==1?21.0f:2.0f));
   CHECK(c.max_speed==0.10324832f && c.min_speed==0.030f && c.max_drive_ms==12000);
 }
 zdt_match.checkpoint=MATCH_TEMP2;
 CHECK(ZDT_Align_ConfigForStage(3,&c) && c.tolerance_px==10.0f && c.stable_frames==3);
 CHECK(zdt_align_config[2].tolerance_px==2.0f && zdt_align_config[2].stable_frames==5);
 zdt_manual_align_station=3;
 CHECK(ZDT_Align_ConfigForStage(3,&c) && c.tolerance_px==2.0f && c.stable_frames==5);
 zdt_manual_align_station=0;zdt_match.checkpoint=MATCH_RAW1;
 CHECK(ZDT_Align_ConfigForStage(1,&c) && !c.plate_search_right);
 zdt_match.checkpoint=MATCH_RAW2;
 CHECK(ZDT_Align_ConfigForStage(1,&c) && c.plate_search_right && c.search_speed==0.030f);
 CHECK(!zdt_align_config[0].plate_search_right); /* Runtime direction never contaminates defaults. */
 zdt_manual_align_station=1;
 CHECK(ZDT_Align_ConfigForStage(1,&c) && !c.plate_search_right);
 zdt_manual_align_station=0;zdt_match.checkpoint=MATCH_RAW1;
 CHECK(ZDT_Align_ConfigForStage(1,&c) && !c.plate_search_right);
 zdt_match.checkpoint=MATCH_RAW2;zdt_align_config[0].search_axis=V_ALIGN_SEARCH_OFF;
 CHECK(ZDT_Align_ConfigForStage(1,&c) && !c.search_axis);
 return test_failures;
}
'''.replace('/* STAGE_FUNCTION */', function)
gcc = shutil.which('arm-none-eabi-gcc')
if not gcc: raise SystemExit('arm-none-eabi-gcc required')
with tempfile.TemporaryDirectory(prefix='vision-align-tests-') as directory:
    work = Path(directory); c = work/'test.c'; binary = work/'test.elf'
    c.write_text(fixture+syscalls, encoding='utf-8')
    linker = work/'test.ld'
    linker.write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    includes = ('Core/Inc','Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32H7xx/Include','Drivers/CMSIS/Include')
    subprocess.run([gcc,'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror','-Wno-error=misleading-indentation',
        '-DSTM32H723xx','-DUSE_HAL_DRIVER','-ffunction-sections','-fdata-sections',
        *['-I'+str(ROOT/p) for p in includes],str(c),str(ROOT/'Core/Src/vision_align.c'),str(ROOT/'Core/Src/grid_nav.c'),str(ROOT/'Core/Src/vision_mode_gate.c'),
        str(ROOT/'tools/tests/test_vision_align.c'),'-nostartfiles','--specs=nosys.specs',
        '-Wl,--gc-sections','-Wl,-T,'+str(linker),'-lm','-o',str(binary)],check=True)
    machine=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    machine.mem_map(0x08000000,0x100000);machine.mem_map(0x20000000,0x100000)
    with binary.open('rb') as stream:
        elf=ELFFile(stream)
        for segment in elf.iter_segments():
            if segment['p_type']=='PT_LOAD' and segment['p_filesz']:machine.mem_write(segment['p_vaddr'],segment.data())
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()};entry=elf['e_entry']
    sentinel=0x080ff000;done=[]
    def finish(cpu,address,size,user):done.append(True);cpu.emu_stop()
    machine.hook_add(UC_HOOK_CODE,finish,begin=sentinel,end=sentinel)
    machine.reg_write(UC_ARM_REG_SP,0x200ff000);machine.reg_write(UC_ARM_REG_LR,sentinel|1)
    machine.emu_start(entry|1,sentinel+2,timeout=30_000_000,count=100_000_000)
    def value(name):return int.from_bytes(machine.mem_read(symbols[name],4),'little')
    print('XY alignment: %d checks, %d failures; first line=%d' % (value('test_checks'),value('test_failures'),value('test_first_failure')))
    if not done or value('test_failures') or not value('test_checks'):raise SystemExit('XY alignment failed')
