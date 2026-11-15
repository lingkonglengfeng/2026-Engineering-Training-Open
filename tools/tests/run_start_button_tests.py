"""Run production PD1 EXTI capture, task debounce and original OK dispatch."""
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
block = (root / "tools/tests/test_start_button.c").read_text(encoding="utf-8")
source = (root / "Core/Src/zdt_motor.c").read_text(encoding="utf-8")
def body_at(needle,text=source):
    start=text.index(needle);brace=text.index('{',start);depth=1;pos=brace+1
    while depth:
        if text[pos]=='{':depth+=1
        elif text[pos]=='}':depth-=1
        pos+=1
    return text[start:pos]
ok=body_at('if (ZDT_Match_CommandWord(command,"ok")) {')
prepare=body_at('if(ZDT_Match_CommandWord(command,"prepare")) {')
service=body_at('static void ZDT_StartButton_Service(void)\n{')
dispatch=body_at('static HAL_StatusTypeDef ZDT_ConfirmLaunchZone(uint8_t zone)\n{')+'\n'+'static uint8_t ZDT_Match_Command(const char *command,uint32_t received_tick) {\n'+prepare+'\n'+ok+'\nreturn 0U;\n}\n'+service
dispatch+=body_at('void EXTI1_IRQHandler(void)',(root/'Core/Src/stm32h7xx_it.c').read_text(encoding='utf-8'))
block=block.replace('/* INSERT_PRODUCTION_DISPATCH */',dispatch)
assert '/* INSERT_PRODUCTION_DISPATCH */' not in block
assert '#define LIGHT_CH1_PIN                (0U)' in (root/'Core/Src/light.c').read_text(encoding='utf-8')
config=(root/'Engineering_Training_Project.ioc').read_text(encoding='utf-8')
assert 'PD1.Signal=GPXTI1' in config and 'PA1.Signal=GPIO_Input' in config
assert 'PD1.GPIO_ModeDefaultEXTI=GPIO_MODE_IT_RISING_FALLING' in config
assert 'HAL_GPIO_ReadPin' not in service
with tempfile.TemporaryDirectory(prefix="arm-round1-tests-") as temp:
    work = Path(temp)
    c = work / "test.c"
    c.write_text(block + syscalls, encoding="utf-8")
    linker = work / "test.ld"
    linker.write_text("ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n")
    binary = work / "test.elf"
    gcc = shutil.which("arm-none-eabi-gcc")
    includes = [root / "Core/Inc", root / "Core/Src", root / "Drivers/STM32H7xx_HAL_Driver/Inc", root / "Drivers/CMSIS/Device/ST/STM32H7xx/Include", root / "Drivers/CMSIS/Include"]
    subprocess.run([gcc,"-std=c99","-mcpu=cortex-m3","-mthumb","-O1","-Wall","-Wextra","-Werror","-DSTM32H723xx","-DUSE_HAL_DRIVER",*["-I"+str(x) for x in includes],"-ffunction-sections","-fdata-sections",str(c), str(root/"Core/Src/match_control.c"),str(root/"Core/Src/gpio.c"),str(root/"Core/Src/bt_button.c"),"-nostartfiles","--specs=nosys.specs","-Wl,--gc-sections","-Wl,-T,"+str(linker),"-lm","-o",str(binary)],check=True)
    emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    emu.mem_map(0x08000000,0x100000);emu.mem_map(0x20000000,0x100000);emu.mem_map(0x58000000,0x100000)
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
    print(f"PD1 key/original OK/GPIO: {value('test_checks')} checks, {value('test_failures')} failures; first line={value('test_first_failure')}")
    if not completed or not value("test_checks") or value("test_failures"):raise SystemExit("PD1 key/original OK/GPIO failed")
