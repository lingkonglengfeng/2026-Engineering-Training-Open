"""Run production DMA log queue and TX ownership with mocked UART hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

root = Path(__file__).resolve().parents[2]
syscalls = (root / "tools/tests/test_arm_driver.c").read_text(encoding="utf-8")
syscalls = syscalls[syscalls.index("int _close("):syscalls.index("volatile unsigned int test_checks")]
block = (root / "tools/tests/test_console_dma.c").read_text(encoding="utf-8")
with tempfile.TemporaryDirectory(prefix="console-dma-tests-") as temp:
    work = Path(temp)
    c = work / "test.c"
    c.write_text(block + syscalls, encoding="utf-8")
    linker = work / "test.ld"
    linker.write_text("ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) }\n .console_dma : { *(.console_dma) } . = ALIGN(8); end = .; }\n")
    binary = work / "test.elf"
    gcc = shutil.which("arm-none-eabi-gcc")
    includes=[root/'tools/tests/console_stubs',root/'Core/Inc']
    subprocess.run([gcc,'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',*["-I"+str(x) for x in includes],'-ffunction-sections','-fdata-sections',str(c),str(root/'Core/Src/console_dma.c'),'-nostartfiles','--specs=nosys.specs','-Wl,--gc-sections','-Wl,-T,'+str(linker),'-o',str(binary)],check=True)
    emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    emu.mem_map(0x08000000,0x100000);emu.mem_map(0x20000000,0x100000)
    with binary.open("rb") as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment["p_type"]=="PT_LOAD" and segment["p_filesz"]:emu.mem_write(segment["p_vaddr"],segment.data())
        symbols = {s.name:s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()};entry=elf["e_entry"]
    sentinel=0x080FF000;completed=[]
    def returned(machine,address,size,user_data):completed.append(True);machine.emu_stop()
    emu.hook_add(UC_HOOK_CODE,returned,begin=sentinel,end=sentinel)
    emu.reg_write(UC_ARM_REG_SP,0x200FF000);emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=30_000_000,count=30_000_000)
    def value(name):return int.from_bytes(emu.mem_read(symbols[name],4),"little")
    print(f"Console DMA runtime: {value('checks')} checks, {value('failures')} failures; first line={value('first_failure')}")
    if not completed or not value("checks") or value("failures"):raise SystemExit("Console DMA runtime failed")
