"""Execute production visual yaw adapters and alignment health checks with mocked ports in Cortex-M."""
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
    adapter=(ROOT/'Core/Src/zdt_motor.c').read_text(encoding='utf-8')
    def extract_adapter(sig):
        start=adapter.index(sig);end=adapter.index('{',start)+1;depth=1
        while depth:
            depth+=(adapter[end]=='{')-(adapter[end]=='}');end+=1
        return adapter[start:end]
    adapters='\n'.join(extract_adapter(x) for x in (
        'static void ZDT_Align_ReadPixels(', 'static void ZDT_VyawObserve(void)', 'static uint8_t ZDT_VyawCommand(', 'static uint8_t ZDT_Align_SearchWithYaw(',
        'static void ZDT_Align_ResumeCombinedXY(', 'static float ZDT_Align_YawOmega(', 'static void ZDT_Align_TimeoutDetails(',
        'static void ZDT_Align_ProgressService(void)\n{'))
    dispatch=extract_adapter('static void ZDT_Align_Service(void)\n{')
    dispatch=dispatch[dispatch.index('  ChassisMotor_GetWheelRpm(rpm);\n  for(i=0U;'):]
    dispatch='''static void ZDT_Align_CoupledUnderTest(void) {
      uint32_t now=HAL_GetTick(),age=UINT32_MAX;
      chassis_telemetry_t t=telemetry;vision_pixel_t p={0};vision_ring_count_t scene={0};vision_plate_scene_t plate={0};
      float forward,right,error=0,omega;int16_t rpm[4];uint8_t i,stopped=1U;
    '''+dispatch
    # Exercise the actual alignment health path through the real yaw observer;
    # cut before XY dispatch, keeping its clock/sample/timeout logic unchanged.
    health=extract_adapter('static void ZDT_Align_Service(void)\n{')
    health=health[:health.index('  error=NavBlend_AngleError(')]
    health=health.replace('static void ZDT_Align_Service(void)','static void ZDT_Align_HealthUnderTest(void)')
    health+='\n  (void)age;(void)p;(void)scene;(void)plate;(void)forward;(void)right;(void)error;(void)omega;(void)rpm;(void)i;(void)stopped;\n  align_health_passes++;\n}\n'
    (work/'vyaw_adapter.inc').write_text(adapters+'\n'+health+'\n'+dispatch,encoding='utf-8')
    linker=work/'tests.ld'
    linker.write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }')
    binary=work/'tests.elf'
    includes=['Core/Inc','Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32H7xx/Include',
              'Drivers/CMSIS/Include','Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2']
    subprocess.run([gcc,'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
       '-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER','-I'+str(work),
       *['-I'+str(ROOT/i) for i in includes],str(ROOT/'Core/Src/vision_yaw.c'),str(ROOT/'Core/Src/vision_align.c'),str(ROOT/'Core/Src/nav_frame.c'),str(ROOT/'Core/Src/nav_blend.c'),str(ROOT/'tools/tests/test_vision_yaw.c'),
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
    print(f"PASS: {val('test_checks')} visual yaw calibration/control assertions; no hardware")
