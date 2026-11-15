"""Run existing corner/owner C regressions without the unrelated full match mocks."""
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
syscalls=(ROOT/'tools/tests/test_arm_driver.c').read_text(encoding='utf-8')
syscalls=syscalls[syscalls.index('int _close('):syscalls.index('volatile unsigned int test_checks')]
fixture='''#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t test_checks,test_failures,test_first_failure;
uint32_t detail_count,failed_lines[64];const char *failed_files[64];
void Test_RecordFailure(const char *file,uint32_t line){if(detail_count<64){failed_files[detail_count]=file;failed_lines[detail_count++]=line;}}
void Test_NavCorner(void);void Test_NavCornerOwner(void);void Test_TurnHandoff(void);
int main(void){Test_NavCorner();Test_NavCornerOwner();Test_TurnHandoff();return test_failures;}
'''
gcc=shutil.which('arm-none-eabi-gcc')
if not gcc:raise SystemExit('arm-none-eabi-gcc required')
with tempfile.TemporaryDirectory(prefix='nav-corner-tests-') as directory:
    work=Path(directory);c=work/'test.c';binary=work/'test.elf';linker=work/'test.ld'
    c.write_text(fixture+syscalls,encoding='utf-8')
    (work/'outer_corner_owner_under_test.inc').write_text('\n\n'.join(extract(s) for s in (
        'static uint8_t ZDT_Outer_Clear(float x,float y,float yaw)\n{',
        'static uint8_t ZDT_Outer_Skip(const char *reason,float value)\n{',
        'static void ZDT_Outer_PrepareChain(void)\n{',
        'static void ZDT_Outer_ChainStep(uint32_t now,float yaw)\n{',
        'static uint8_t ZDT_Outer_TryStart(void)\n{',
        'static void ZDT_Outer_WaitStopped(void)\n{',
        'static void ZDT_Outer_OnQuery(uint8_t mask)\n{',
        'static void ZDT_Outer_Service(void)\n{',
        'static uint8_t ZDT_Outer_Command(const char *command)\n{')),encoding='utf-8')
    nav_case=source[source.index('    case ZDT_NAV_ALIGN_TURN:'):source.index('    case ZDT_NAV_DRIVE:')]
    (work/'turn_handoff_under_test.inc').write_text('\n\n'.join(extract(signature) for signature in (
        'static uint8_t ZDT_TurnBandStable(uint8_t owner,float error,uint32_t now)\n{',
        'static uint8_t ZDT_TurnBaselineReady(void)\n{',
        'static void ZDT_TurnBaselinePoll(void)\n{',
        'static void ZDT_Zone2_Done(void)\n{',
        'static void ZDT_Nav_ResumeRolling(void)\n{',
        'static void ZDT_Zone2_Service(void)\n{'))+
        '\nstatic void Test_NavTurnCase(void){const grid_nav_segment_t *segment=&test_segment;switch(zdt_nav_state){\n'+nav_case+'default:break;}}\n',encoding='utf-8')
    linker.write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    includes=('Core/Inc','Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32H7xx/Include','Drivers/CMSIS/Include')
    # Frozen controller/plant fixtures use their original physical yellow map
    # (0mm extra band). The production10mm planning boundary, tight-arc
    # rejection and clear-arc acceptance are covered by test_obstacle_sites.c.
    grid=work/'grid_nav.o'
    subprocess.run([gcc,'-c','-std=c99','-mcpu=cortex-m3','-mthumb','-O1',
        '-DSTM32H723xx','-DUSE_HAL_DRIVER','-DGRID_NAV_YELLOW_MARGIN_MM=0','-ffunction-sections','-fdata-sections',
        *['-I'+str(ROOT/p) for p in includes],str(ROOT/'Core/Src/grid_nav.c'),'-o',str(grid)],check=True)
    sources=('Core/Src/nav_corner.c','Core/Src/nav_frame.c','Core/Src/nav_blend.c',
        'Core/Src/nav_params.c','Core/Src/zone2_route.c','Core/Src/chassis_field.c',
        'tools/tests/test_nav_corner.c','tools/tests/test_nav_corner_owner.c','tools/tests/test_turn_handoff.c')
    subprocess.run([gcc,'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
        '-DSTM32H723xx','-DUSE_HAL_DRIVER','-DGRID_NAV_YELLOW_MARGIN_MM=0','-ffunction-sections','-fdata-sections','-I'+str(work),
        *['-I'+str(ROOT/p) for p in includes],str(c),*[str(ROOT/p) for p in sources],str(grid),
        '-nostartfiles','--specs=nosys.specs','-Wl,--gc-sections','-Wl,-T,'+str(linker),'-lm','-o',str(binary)],check=True)
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
    machine.emu_start(entry|1,sentinel+2,timeout=45_000_000,count=1_600_000_000)
    def value(name):return int.from_bytes(machine.mem_read(symbols[name],4),'little')
    print('Outer arc controller (original0mm physical-map fixtures): %d checks, %d failures; first line=%d; no hardware'%(value('test_checks'),value('test_failures'),value('test_first_failure')))
    if 'test_corner_exit_overshoot' in symbols:
        import struct
        peaks=struct.unpack('<9f',machine.mem_read(symbols['test_corner_exit_overshoot'],36))
        print('Nine existing corner fixtures: peak exit yaw overshoot %.3fdeg; model only'%max(peaks))
    if value('test_failures'):
        import struct
        for i in range(value('detail_count')):
            address=int.from_bytes(machine.mem_read(symbols['failed_files']+i*4,4),'little')
            print(bytes(machine.mem_read(address,240)).split(b'\0',1)[0].decode(errors='replace'),int.from_bytes(machine.mem_read(symbols['failed_lines']+i*4,4),'little'))
        if 'test_corner_metrics' in symbols:
            rows=struct.unpack('<72f',machine.mem_read(symbols['test_corner_metrics'],288))
            for i in range(9):print('corner',i,'time/cross/peak/end/minspeed/progress/odom/stable',rows[i*8:i*8+8])
    if not done:raise SystemExit('Outer arc execution limit reached; not a completed test run')
    if value('test_failures') or not value('test_checks'):raise SystemExit('Outer arc checks failed')
