"""Run actual alignment handoff and onward-route checks with hardware ports mocked."""
from pathlib import Path
import sys
import re
import struct
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR

ROOT=Path(__file__).resolve().parents[2]
source=(ROOT/'Core/Src/zdt_motor.c').read_text(encoding='utf-8')


def extract(signature):
    begin=source.index(signature);brace=source.index('{',begin);end=brace+1;depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[begin:end]


with tempfile.TemporaryDirectory(prefix='align-anchor-tests-') as temp:
    work=Path(temp)
    signatures=[
        'static float ZDT_Nav_GetHeading(void)\n{',
        'static float ZDT_Zone2_RelYaw(float world)\n{',
        'static void ZDT_Cardinal_Align(uint8_t station)\n{',
        'static uint8_t ZDT_Align_UsesYaw(uint8_t station)\n{',
        'static uint8_t ZDT_Align_Flags(uint8_t mode,uint8_t station)\n{',
        'static uint8_t ZDT_RingProfileCommand(const char *command)\n{',
        'static uint8_t ZDT_Align_Station(void)',
        'static uint8_t ZDT_Align_ConfigForStage(uint8_t station,vision_align_config_t *out)\n{',
        'static uint8_t ZDT_Align_Temp2StationaryReady(uint32_t now,uint8_t stopped)\n{',
        'static void ZDT_Align_OnMileage(uint8_t mask)\n{']
    (work/'align_anchor_under_test.inc').write_text('\n\n'.join(map(extract,signatures)),encoding='utf-8')
    params='\n'.join(line for line in source.splitlines() if line.startswith(('#define ZDT_ODO_HOLD_KP ', '#define ZDT_ODO_HOLD_OMEGA_MAX ', '#define ZDT_NAV_HOLD_DEADBAND_DEG ')))
    correction=re.search(r'^  omega=fabsf\(error\).*?;',source,re.M).group(0)
    (work/'odo_heading_under_test.inc').write_text(params+'\nstatic float HeadingCorrection(float error){float omega;'+correction+'return omega;}\n',encoding='utf-8')
    # Reuse established transport/clock/Flash ports; only select a smaller main.
    fixture=(ROOT/'tools/tests/test_match_control.c').read_text(encoding='utf-8')
    (work/'ports.c').write_text(fixture.replace('int main(void)','int LegacyMain(void)'),encoding='utf-8')
    entry = 'Test_NavOdometry' if '--odometry-only' in sys.argv else 'Test_HeadingResponse' if '--heading-response-only' in sys.argv else 'Test_CardinalTurnTransfer' if '--cardinal-transfer-only' in sys.argv else 'Test_FieldWheelLimits' if '--field-limits-only' in sys.argv else 'Test_AlignmentNoApproach'
    (work/'main.c').write_text('extern volatile unsigned int test_failures; void '+entry+'(void); int main(void){'+entry+'();return test_failures;}\n')
    linker=work/'test.ld'
    linker.write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    binary=work/'test.elf'
    includes=['Core/Inc','Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32H7xx/Include','Drivers/CMSIS/Include']
    sources=['nav_params','nav_odometry','nav_feedback','chassis_motor','grid_nav','nav_frame','nav_blend','nav_corner','chassis_field','zone2_route','vision_align','vision_yaw','vision_align_diag','vision_mode_gate']
    subprocess.run([shutil.which('arm-none-eabi-gcc'),'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror','-Wno-error=misleading-indentation',
        '-DSTM32H723xx','-DUSE_HAL_DRIVER','-ffunction-sections','-fdata-sections','-I'+str(work),
        *['-I'+str(ROOT/p) for p in includes],str(work/'main.c'),str(work/'ports.c'),
        *[str(ROOT/'Core/Src'/f'{name}.c') for name in sources],
        *[str(ROOT/'tools/tests'/name) for name in ['test_align_anchor.c','test_chassis_field.c','test_heading_response.c','test_nav_odometry.c']],
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
    label='Odometry residual acceptance' if '--odometry-only' in sys.argv else 'Heading response' if '--heading-response-only' in sys.argv else 'Cardinal/rolling transfer' if '--cardinal-transfer-only' in sys.argv else 'Field wheel limits' if '--field-limits-only' in sys.argv else 'Align handoff/onward route'
    print(label+': %d checks, %d failures; first line=%d'%(value('test_checks'),value('test_failures'),value('test_first_failure')))
    if '--odometry-only' in sys.argv:
        print('Approach4 vs8,20/50/750mm (ms):',struct.unpack('<3I',machine.mem_read(symbols['test_approach4_ms'],12)),struct.unpack('<3I',machine.mem_read(symbols['test_approach8_ms'],12)))
    if '--heading-response-only' in sys.argv:
        print('Ideal10deg response, including100ms in deadband (ms):',struct.unpack('<4I',machine.mem_read(symbols['heading_response_ms'],16)))
    if value('test_failures') and 'test_failure_detail_count' in symbols:
        for i in range(value('test_failure_detail_count')):
            address=int.from_bytes(machine.mem_read(symbols['test_failure_files']+i*4,4),'little')
            print(bytes(machine.mem_read(address,240)).split(b'\0',1)[0].decode(errors='replace'),int.from_bytes(machine.mem_read(symbols['test_failure_lines']+i*4,4),'little'))
    if not done or value('test_failures') or not value('test_checks'):raise SystemExit('Alignment handoff failed')
