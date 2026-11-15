"""Exercise production short-map/departure decisions and odometry IO, no hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT/'Core/Src/zdt_motor.c').read_text(encoding='utf-8')
def extract(signature):
    start=source.index(signature);end=source.index('{',start)+1;depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]
functions=(
    'static float ZDT_Vobs_OdoTarget(uint8_t owner,float mm)\n{',
    'static void ZDT_Odo_Cancel(void)\n{',
    'static uint8_t ZDT_Odo_InitBase(',
    'static uint8_t ZDT_Odo_Start(uint8_t owner,float mm,float scale,int8_t forward,int8_t right,\n    float heading,const int32_t *base,uint8_t mask)\n{',
    'static uint8_t ZDT_Odo_StartMap(uint8_t owner,float mm,float scale,float mx,float my,\n    float heading,const int32_t *base,uint8_t mask)\n{',
    'static uint8_t ZDT_Nav_BeginDepartureRolling(float mm,float scale)\n{',
    'static uint8_t ZDT_Odo_OnMileage(uint8_t mask)\n{',
    'static uint8_t ZDT_Odo_Service(uint8_t owner)\n{',
    'static uint8_t ZDT_Nav_ShortTranslationAllowed(const grid_nav_segment_t *segment)\n{',
    'static int8_t ZDT_Nav_SegmentStrafeSign(const grid_nav_segment_t *segment)\n{',
    'static uint8_t ZDT_Nav_TryMapTranslation(const grid_nav_segment_t *segment)\n{',
    'static void ZDT_Departure_PrepareRoll(const zdt_script_step_t *step)\n{',
    'static uint8_t ZDT_Departure_RollStep(void)\n{',
    'static void ZDT_Script_StartLeave(void)\n{')
a=source.index('    case ZDT_NAV_WAIT_START_QUERY:',source.index('static void ZDT_Nav_Service(void)\n{'))
b=source.index('    case ZDT_NAV_ALIGN_TURN:',a)
nav_case='static void Test_NavStartCase(const grid_nav_segment_t *segment){switch(zdt_nav_state){\n'+source[a:b]+'default:break;}}\n'
syscalls=(ROOT/'tools/tests/test_arm_driver.c').read_text(encoding='utf-8')
syscalls='#include <sys/stat.h>\n#include <sys/types.h>\n'+syscalls[syscalls.index('int _close('):syscalls.index('volatile unsigned int test_checks')]
gcc = shutil.which('arm-none-eabi-gcc')
if not gcc:
    raise SystemExit('arm-none-eabi-gcc required')
with tempfile.TemporaryDirectory(prefix='map-owner-') as temp:
    work = Path(temp);c = work/'test.c';linker = work/'test.ld';binary = work/'test.elf'
    (work/'map_owner_under_test.inc').write_text('\n\n'.join(extract(f) for f in functions)+'\n'+nav_case,encoding='utf-8')
    c.write_text((ROOT/'tools/tests/test_map_translation.c').read_text(encoding='utf-8')+'\n'+syscalls,encoding='utf-8')
    linker.write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    subprocess.run([gcc,'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
        '-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER','-I'+str(work),
        *['-I'+str(ROOT/p) for p in ('Core/Inc','Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32H7xx/Include','Drivers/CMSIS/Include')],str(c),
        str(ROOT/'Core/Src/nav_params.c'),str(ROOT/'Core/Src/nav_blend.c'),
        str(ROOT/'Core/Src/nav_odometry.c'),str(ROOT/'Core/Src/nav_feedback.c'),str(ROOT/'Core/Src/nav_frame.c'),
        '-nostartfiles','--specs=nosys.specs','-Wl,--gc-sections','-Wl,-T,'+str(linker),'-lm','-o',str(binary)],check=True)
    machine = Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    machine.mem_map(0x08000000,0x100000);machine.mem_map(0x20000000,0x100000)
    with binary.open('rb') as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment['p_type']=='PT_LOAD' and segment['p_filesz']:
                machine.mem_write(segment['p_vaddr'],segment.data())
        symbols = {s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()};entry = elf['e_entry']
    sentinel = 0x080FF000;completed = []
    def finish(cpu,address,size,user):
        completed.append(True);cpu.emu_stop()
    machine.hook_add(UC_HOOK_CODE,finish,begin=sentinel,end=sentinel)
    machine.reg_write(UC_ARM_REG_SP,0x200FF000);machine.reg_write(UC_ARM_REG_LR,sentinel|1)
    machine.emu_start(entry|1,sentinel+2,timeout=2_000_000,count=1_000_000)
    def value(name):
        return int.from_bytes(machine.mem_read(symbols[name],4),'little')
    print('Map translation owner: %d checks, %d failures; first line=%d; no driving simulation'%(
        value('test_checks'),value('test_failures'),value('test_first_failure')))
    if not completed or not value('test_checks') or value('test_failures'):
        raise SystemExit('Map translation owner checks failed')
