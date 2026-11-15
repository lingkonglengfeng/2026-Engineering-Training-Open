"""Run existing corner/owner C regressions without the unrelated full match mocks."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

ROOT = Path(__file__).resolve().parents[2]

syscalls=(ROOT/'tools/tests/test_arm_driver.c').read_text(encoding='utf-8')
syscalls=syscalls[syscalls.index('int _close('):syscalls.index('volatile unsigned int test_checks')]
with tempfile.TemporaryDirectory(prefix='motion-params-') as directory:
    work=Path(directory);c=work/'test.c';binary=work/'test.elf';linker=work/'test.ld'
    c.write_text((ROOT/'tools/tests/test_motion_params.c').read_text(encoding='utf-8')+syscalls,encoding='utf-8')
    linker.write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    includes=('Core/Inc','Core/Src','Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32H7xx/Include','Drivers/CMSIS/Include')
    subprocess.run([shutil.which('arm-none-eabi-gcc'),'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror','-DSTM32H723xx','-DUSE_HAL_DRIVER',*['-I'+str(ROOT/p) for p in includes],str(c),'-nostartfiles','--specs=nosys.specs','-Wl,-T,'+str(linker),'-lm','-o',str(binary)],check=True)
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
    print('Motion W25 snapshot: %d checks, %d failures; first line=%d; no hardware'%(value('test_checks'),value('test_failures'),value('test_first_failure')))
    if not done or value('test_failures'):raise SystemExit('motion params failed')
