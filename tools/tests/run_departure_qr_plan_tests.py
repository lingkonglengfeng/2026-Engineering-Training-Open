"""Check the real departure planner and field geometry without driving simulation."""
from pathlib import Path
import shutil, subprocess, tempfile, re
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR
ROOT=Path(__file__).resolve().parents[2]
gcc=shutil.which('arm-none-eabi-gcc')
if not gcc: raise SystemExit('arm-none-eabi-gcc is required')
source=(ROOT/'Core/Src/zdt_motor.c').read_text(encoding='utf-8')
def function(signature):
    start=source.index(signature); brace=source.index('{',start); pos=brace+1; depth=1
    while depth:
        depth+=(source[pos]=='{')-(source[pos]=='}'); pos+=1
    return source[start:pos]
with tempfile.TemporaryDirectory(prefix='departure-qr-check-') as tmp:
    work=Path(tmp)
    defines=[]
    for name in ('ZDT_NAV_ALIGN_TOLERANCE_DEG','ZDT_DEPARTURE_LANE_TOLERANCE_MM'):
        match=re.search(r'^#define\s+'+name+r'\s+.*$',source,re.M)
        if not match:raise SystemExit('missing '+name)
        defines.append(match.group())
    (work/'departure_qr_under_test.inc').write_text('\n'.join(defines)+'\n'+function('static uint8_t ZDT_Zone2_TryDepartureQrStraight('),encoding='utf-8')
    (work/'tests.ld').write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    binary=work/'tests.elf'
    includes=[ROOT/'Core/Inc',ROOT/'Drivers/STM32H7xx_HAL_Driver/Inc',ROOT/'Drivers/CMSIS/Device/ST/STM32H7xx/Include',ROOT/'Drivers/CMSIS/Include',work]
    command=[gcc,'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER']
    command+=['-I'+str(p) for p in includes]
    command += [str(ROOT/'Core/Src/grid_nav.c'),str(ROOT/'Core/Src/nav_frame.c'),str(ROOT/'tools/tests/test_departure_qr_plan.c'),'-nostartfiles','--specs=nosys.specs','-Wl,--gc-sections','-Wl,-T,'+str(work/'tests.ld'),'-lm','-o',str(binary)]
    subprocess.run(command,check=True)
    emu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);emu.mem_map(0x08000000,0x100000);emu.mem_map(0x20000000,0x100000)
    with binary.open('rb') as stream:
        elf=ELFFile(stream)
        for segment in elf.iter_segments():
            if segment['p_type']=='PT_LOAD' and segment['p_filesz']:emu.mem_write(segment['p_vaddr'],segment.data())
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()};entry=elf['e_entry']
    sentinel=0x080FF000;completed=[]
    def returned(machine,address,size,data):completed.append(True);machine.emu_stop()
    emu.hook_add(UC_HOOK_CODE,returned,begin=sentinel,end=sentinel)
    emu.reg_write(UC_ARM_REG_SP,0x200FF000);emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=45_000_000,count=500_000_000)
    def val(name):return int.from_bytes(emu.mem_read(symbols[name],4),'little')
    checks,failures,first=val('test_checks'),val('test_failures'),val('test_first_failure')
    if not completed or failures:raise SystemExit(f'FAIL: completed={bool(completed)} checks={checks} failures={failures} first line={first}')
    print(f'PASS: {checks} departure planner checks, no driving simulation')
