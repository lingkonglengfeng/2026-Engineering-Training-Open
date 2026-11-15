"""Execute production DM enable handshake against delayed CAN replies and STOP."""
from pathlib import Path
import shutil,subprocess,tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR
ROOT=Path(__file__).resolve().parents[2]
gcc=shutil.which('arm-none-eabi-gcc')
if not gcc: raise SystemExit('arm-none-eabi-gcc required')
source=(ROOT/'Core/Src/dm_motor.c').read_text(encoding='utf-8')
def function(signature):
    begin=source.index(signature);body=source.index('{',begin);end=body+1;depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[begin:end]
with tempfile.TemporaryDirectory(prefix='dm-zero-tests-') as folder:
    work=Path(folder)
    signatures=['static void DM_WaitYield(', 'static HAL_StatusTypeDef DM_SendSpecialCmd(', 'static HAL_StatusTypeDef DM_SendPosFrame(',
                'static void DM_ReadFeedbackSnapshot(', 'static uint8_t DM_EnableWaitFeedback(',
                'static uint8_t DM_EnableAbort(', 'static uint8_t DM_EnableCommit(',
                'static HAL_StatusTypeDef DM_EnableConfirmHold(', 'HAL_StatusTypeDef DM_Motor_Enable(']
    (work/'dm_zero_under_test.inc').write_text('\n\n'.join(map(function,signatures)),encoding='utf-8')
    linker=work/'tests.ld'
    linker.write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }')
    binary=work/'tests.elf'
    includes=['Core/Inc','Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32H7xx/Include',
              'Drivers/CMSIS/Include','Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2']
    subprocess.run([gcc,'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
       '-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER','-I'+str(work),
       *['-I'+str(ROOT/i) for i in includes],str(ROOT/'tools/tests/test_dm_enable.c'),
       '-nostartfiles','--specs=nosys.specs','-Wl,--gc-sections','-Wl,-T,'+str(linker),'-lm','-o',str(binary)],check=True)
    emu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    emu.mem_map(0x08000000,0x100000);emu.mem_map(0x20000000,0x100000)
    with binary.open('rb') as stream:
        elf=ELFFile(stream)
        for seg in elf.iter_segments():
            if seg['p_type']=='PT_LOAD' and seg['p_filesz']: emu.mem_write(seg['p_vaddr'],seg.data())
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()};entry=elf['e_entry']
    sentinel=0x080ff000;done=[]
    def returned(machine,address,size,data): done.append(True);machine.emu_stop()
    emu.hook_add(UC_HOOK_CODE,returned,begin=sentinel,end=sentinel)
    emu.reg_write(UC_ARM_REG_SP,0x200ff000);emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=30_000_000,count=10_000_000)
    def val(name):return int.from_bytes(emu.mem_read(symbols[name],4),'little')
    if not done or val('test_failures'):raise SystemExit(f"FAIL: checks={val('test_checks')} failures={val('test_failures')} first={val('test_first_failure')}")
    print(f"PASS: {val('test_checks')} DM enable/holding/timeout/STOP assertions; no hardware")
