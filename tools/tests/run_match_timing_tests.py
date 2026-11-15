"""Focused integer-clock checks on production match_control.c; no driving plant."""
from pathlib import Path
import shutil
import subprocess
import tempfile

from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR

ROOT=Path(__file__).resolve().parents[2]
source=(ROOT/'tools/tests/test_match_control.c').read_text(encoding='utf-8')
callbacks=source[source.index('static uint32_t calls;'):source.index('static void complete_route')]
cases=source[source.index('static void run_timer_tests(void)'):source.index('int main(void)')]
prefix='''#include "match_control.h"
#include <stdint.h>
volatile uint32_t checks,failures,first_failure;
#define CHECK(x) do{checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
'''
gcc=shutil.which('arm-none-eabi-gcc')
if not gcc:raise SystemExit('arm-none-eabi-gcc required')
with tempfile.TemporaryDirectory(prefix='match-clock-tests-') as temp:
    work=Path(temp);c=work/'test.c';linker=work/'test.ld';binary=work/'test.elf'
    c.write_text(prefix+callbacks+cases+'int main(void){run_timer_tests();feedback_timeout_tests();return failures;}\n',encoding='utf-8')
    linker.write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    subprocess.run([gcc,'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
        '-ffunction-sections','-fdata-sections','-I'+str(ROOT/'Core/Inc'),str(c),str(ROOT/'Core/Src/match_control.c'),
        '-nostartfiles','--specs=nosys.specs','-Wl,--gc-sections','-Wl,-T,'+str(linker),'-o',str(binary)],check=True)
    machine=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    machine.mem_map(0x08000000,0x100000);machine.mem_map(0x20000000,0x100000)
    with binary.open('rb') as stream:
        elf=ELFFile(stream)
        for segment in elf.iter_segments():
            if segment['p_type']=='PT_LOAD' and segment['p_filesz']:machine.mem_write(segment['p_vaddr'],segment.data())
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()};entry=elf['e_entry']
    sentinel=0x080FF000;completed=[]
    def finish(cpu,address,size,user):completed.append(True);cpu.emu_stop()
    machine.hook_add(UC_HOOK_CODE,finish,begin=sentinel,end=sentinel)
    machine.reg_write(UC_ARM_REG_SP,0x200FF000);machine.reg_write(UC_ARM_REG_LR,sentinel|1)
    machine.emu_start(entry|1,sentinel+2,timeout=2_000_000,count=1_000_000)
    def value(name):return int.from_bytes(machine.mem_read(symbols[name],4),'little')
    print('Match timing: %d checks, %d failures; first line=%d; no driving simulation'%(value('checks'),value('failures'),value('first_failure')))
    if not completed or not value('checks') or value('failures'):raise SystemExit('Match timing checks failed')
