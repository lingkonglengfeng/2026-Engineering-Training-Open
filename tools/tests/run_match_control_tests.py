"""Compile real Cortex-M C sources and execute regression checks in Unicorn.

Requires arm-none-eabi-gcc on PATH, unicorn==2.1.4 and pyelftools==0.32.
No board, flash operation or peripheral emulation is involved.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import re
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

ROOT = Path(__file__).resolve().parents[2]
gcc = shutil.which("arm-none-eabi-gcc")
if not gcc:
    raise SystemExit("arm-none-eabi-gcc is required")
with tempfile.TemporaryDirectory(prefix="match-control-tests-") as temp:
    work = Path(temp)
    linker = work / "tests.ld"
    linker.write_text("""ENTRY(main)
SECTIONS {
 . = 0x08000000;
 .text : { *(.text*) *(.rodata*) }
 .ARM.exidx : { *(.ARM.exidx*) }
 . = 0x20000000;
 .data : { *(.data*) }
 .bss : { *(.bss*) *(COMMON) }
 . = ALIGN(8);
 end = .;
}
""")
    binary = work / "tests.elf"
    nav_object = work / "nav_params.o"
    subprocess.run([
        gcc, "-c", "-std=c99", "-mcpu=cortex-m3", "-mthumb", "-O1",
        "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
        "-DSTM32H723xx", "-DUSE_HAL_DRIVER",
        "-I"+str(ROOT/"Core/Inc"),
        "-I"+str(ROOT/"Drivers/STM32H7xx_HAL_Driver/Inc"),
        "-I"+str(ROOT/"Drivers/CMSIS/Device/ST/STM32H7xx/Include"),
        "-I"+str(ROOT/"Drivers/CMSIS/Include"),
        str(ROOT/"Core/Src/nav_params.c"), "-o", str(nav_object),
    ], check=True)
    chassis_object = work / "chassis_motor.o"
    subprocess.run([
        gcc, "-c", "-std=c99", "-mcpu=cortex-m3", "-mthumb", "-O1",
        "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
        "-DSTM32H723xx", "-DUSE_HAL_DRIVER",
        "-I"+str(ROOT/"Core/Inc"),
        "-I"+str(ROOT/"Drivers/STM32H7xx_HAL_Driver/Inc"),
        "-I"+str(ROOT/"Drivers/CMSIS/Device/ST/STM32H7xx/Include"),
        "-I"+str(ROOT/"Drivers/CMSIS/Include"),
        str(ROOT/"Core/Src/chassis_motor.c"), "-o", str(chassis_object),
    ], check=True)
    grid_object = work / "grid_nav.o"
    chassis_test_object = work / "chassis_test.o"
    # Exercise the actual route-owner service, including time captured before
    # a synchronous action-start callback. Extract rather than copy its logic.
    source = (ROOT / "Core/Src/zdt_motor.c").read_text(encoding="utf-8")
    signature = "static void ZDT_Zone2_Service(void)\n{"
    begin = source.index(signature)
    brace = begin + len(signature) - 1
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    (work / "zone2_service_under_test.inc").write_text(source[begin:end], encoding="utf-8")
    zone_service_object = work / "zone_service_test.o"
    subprocess.run([
        gcc, "-c", "-std=c99", "-mcpu=cortex-m3", "-mthumb", "-O1",
        "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
        "-DSTM32H723xx", "-DUSE_HAL_DRIVER", "-I"+str(work),
        "-I"+str(ROOT/"Core/Inc"),
        "-I"+str(ROOT/"Drivers/STM32H7xx_HAL_Driver/Inc"),
        "-I"+str(ROOT/"Drivers/CMSIS/Device/ST/STM32H7xx/Include"),
        "-I"+str(ROOT/"Drivers/CMSIS/Include"),
        str(ROOT/"tools/tests/test_zone2_service.c"), "-o", str(zone_service_object),
    ], check=True)
    rx_type = re.search(r'typedef struct\s*\{\s*FDCAN_RxHeaderTypeDef header;.*?\} ZDT_CAN_RxFrameTypeDef;', source, re.S)
    if not rx_type: raise SystemExit('production CAN RX type missing')
    (work / 'zdt_query_rx_type.inc').write_text(rx_type.group(), encoding='utf-8')
    def extract_query_function(signature):
        begin=source.index(signature);brace=source.index('{',begin);end=brace+1;depth=1
        while depth:
            depth+=(source[end]=='{')-(source[end]=='}');end+=1
        return source[begin:end]
    (work / 'zdt_query_isr_under_test.inc').write_text('\n\n'.join([
        extract_query_function('static uint8_t ZDT_CAN_DataLengthBytes('),
        extract_query_function('void ZDT_Motor_RxFifo0MsgPending(')]),encoding='utf-8')
    query_isr_object = work / 'zdt_query_isr_test.o'
    (work / 'align_anchor_under_test.inc').write_text('\n\n'.join([
        extract_query_function('static float ZDT_Nav_GetHeading(void)\n{'),
        extract_query_function('static float ZDT_Zone2_RelYaw(float world)\n{'),
        extract_query_function('static uint8_t ZDT_Align_UsesYaw(uint8_t station)\n{'),
        extract_query_function('static uint8_t ZDT_Align_Flags(uint8_t mode,uint8_t station)\n{'),
        extract_query_function('static uint8_t ZDT_RingProfileCommand(const char *command)\n{'),
        extract_query_function('static uint8_t ZDT_Align_Station(void)'),
        extract_query_function('static uint8_t ZDT_Align_ConfigForStage(uint8_t station,vision_align_config_t *out)\n{'),
        extract_query_function('static uint8_t ZDT_Align_Temp2StationaryReady(uint32_t now,uint8_t stopped)\n{'),
        extract_query_function('static void ZDT_Align_OnMileage(uint8_t mask)\n{')]),encoding='utf-8')
    anchor_object = work / 'align_anchor_test.o'
    (work/'outer_corner_owner_under_test.inc').write_text('\n\n'.join(
        extract_query_function(signature) for signature in [
        'static uint8_t ZDT_Outer_Clear(float x,float y,float yaw)\n{',
        'static uint8_t ZDT_Outer_Skip(const char *reason,float value)\n{',
        'static uint8_t ZDT_Outer_TryStart(void)\n{',
        'static void ZDT_Outer_OnQuery(uint8_t mask)\n{',
        'static void ZDT_Outer_Service(void)\n{',
        'static uint8_t ZDT_Outer_Command(const char *command)\n{']),encoding='utf-8')
    outer_owner_object=work/'outer_corner_owner_test.o'
    subprocess.run([
        gcc,'-c','-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
        '-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER','-I'+str(work),
        '-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Drivers/STM32H7xx_HAL_Driver/Inc'),
        '-I'+str(ROOT/'Drivers/CMSIS/Device/ST/STM32H7xx/Include'),'-I'+str(ROOT/'Drivers/CMSIS/Include'),
        str(ROOT/'tools/tests/test_nav_corner_owner.c'),'-o',str(outer_owner_object)],check=True)
    (work / 'odometry_owner_under_test.inc').write_text('\n\n'.join([
        extract_query_function('static int32_t ZDT_Nav_StepPulses('),
        extract_query_function('static uint32_t ZDT_Script_SettleDelay('),
        extract_query_function('static void ZDT_Odo_Cancel(void)\n{'),
        extract_query_function('static uint8_t ZDT_Odo_InitBase('),
        extract_query_function('static uint8_t ZDT_Odo_Start(uint8_t owner,float mm,float scale,int8_t forward,int8_t right,\n    float heading,const int32_t *base,uint8_t mask)\n{'),
        extract_query_function('static uint8_t ZDT_Odo_OnMileage(uint8_t mask)\n{'),
        extract_query_function('static uint8_t ZDT_Odo_Service(uint8_t owner)\n{'),
        extract_query_function('static uint8_t ZDT_Mileage_AddressIndex('),
        extract_query_function('static int32_t ZDT_Mileage_DecodeSignedPosition('),
        extract_query_function('static void ZDT_Mileage_HandleFrame(const FDCAN_RxHeaderTypeDef *header,\n                                    const uint8_t *data, uint8_t data_length,uint32_t received_tick,uint32_t received_seq)\n{')]),encoding='utf-8')
    owner_object = work / 'odometry_owner_test.o'
    (work/'screen_owner_under_test.inc').write_text('\n\n'.join([
        extract_query_function('static uint8_t ZDT_Screen_CanApply(void)\n{'),
        extract_query_function('static void ZDT_Screen_Service(void)\n{')]),encoding='utf-8')
    (work / 'qr_application_under_test.inc').write_text('\n\n'.join([
        extract_query_function('uint8_t ZDT_Match_GetQrTask(vision_qr_task_t *task)\n{'),
        extract_query_function('static uint8_t ZDT_Qr_Start(uint8_t for_match)\n{'),
        extract_query_function('static void ZDT_Qr_Cancel(void)\n{'),
        extract_query_function('static void ZDT_Qr_Service(void)\n{'),
        extract_query_function('static uint8_t ZDT_Qr_Command(const char *command)\n{'),
        extract_query_function('static uint8_t ZDT_DepartureParallelStart(void)\n{'),
        extract_query_function('static void ZDT_Match_Complete(uint8_t success, const char *reason)\n{'),
        extract_query_function('static void ZDT_MaterialService(void)\n{')]),encoding='utf-8')
    qr_application_object=work/'qr_application_test.o'
    subprocess.run([
        gcc, '-c','-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
        '-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER','-I'+str(work),
        '-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Drivers/STM32H7xx_HAL_Driver/Inc'),
        '-I'+str(ROOT/'Drivers/CMSIS/Device/ST/STM32H7xx/Include'),'-I'+str(ROOT/'Drivers/CMSIS/Include'),
        str(ROOT/'tools/tests/test_qr_application.c'),'-o',str(qr_application_object)],check=True)

    (work / 'return_start_action_under_test.inc').write_text('\n'.join([
        re.search(r'^#define\s+ZDT_NAV_ALIGN_TOLERANCE_DEG\s+.*$',source,re.M).group(),
        re.search(r'^#define\s+ZDT_DEPARTURE_LANE_TOLERANCE_MM\s+.*$',source,re.M).group(),
        extract_query_function('static uint8_t ZDT_Zone2_TryDepartureQrStraight(const zone2_action_t *a,float yaw)\n{'),
        extract_query_function('static uint8_t ZDT_Zone2_StartAction(const zone2_action_t *a,uint32_t id)\n{')]),encoding='utf-8')
    return_object=work/'return_detour_test.o'
    subprocess.run([
        gcc, '-c','-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
        '-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER','-I'+str(work),
        '-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Drivers/STM32H7xx_HAL_Driver/Inc'),
        '-I'+str(ROOT/'Drivers/CMSIS/Device/ST/STM32H7xx/Include'),'-I'+str(ROOT/'Drivers/CMSIS/Include'),
        str(ROOT/'tools/tests/test_return_detour.c'),'-o',str(return_object)],check=True)
    match_check=re.search(r'nav_motion_feedback_t check = .*?;',source,re.S)
    if not match_check: raise SystemExit('production MATCH feedback selection missing')
    (work / 'odometry_match_check_under_test.inc').write_text(match_check.group(),encoding='utf-8')
    final_begin=source.index('        if(zdt_nav_odometry_enabled && ((zdt_nav_start_mask&zdt_nav_end_mask)')
    final_brace=source.index('{',final_begin);final_end=final_brace+1;final_depth=1
    while final_depth:
        final_depth+=(source[final_end]=='{')-(source[final_end]=='}');final_end+=1
    (work / 'odometry_final_guard_under_test.inc').write_text(source[final_begin:final_end],encoding='utf-8')
    drive_begin=source.index('    case ZDT_NAV_DRIVE:\n')
    drive_end=source.index('    case ZDT_NAV_BLEND:\n',drive_begin)
    (work / 'odometry_drive_case_under_test.inc').write_text(source[drive_begin:drive_end],encoding='utf-8')
    subprocess.run([
        gcc, '-c','-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
        '-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER','-I'+str(work),
        '-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Drivers/STM32H7xx_HAL_Driver/Inc'),
        '-I'+str(ROOT/'Drivers/CMSIS/Device/ST/STM32H7xx/Include'),'-I'+str(ROOT/'Drivers/CMSIS/Include'),
        str(ROOT/'tools/tests/test_odometry_owner.c'),'-o',str(owner_object)],check=True)
    subprocess.run([
        gcc, '-c','-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
        '-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER','-I'+str(work),
        '-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Drivers/STM32H7xx_HAL_Driver/Inc'),
        '-I'+str(ROOT/'Drivers/CMSIS/Device/ST/STM32H7xx/Include'),'-I'+str(ROOT/'Drivers/CMSIS/Include'),
        str(ROOT/'tools/tests/test_align_anchor.c'),'-o',str(anchor_object)],check=True)
    subprocess.run([
        gcc, '-c','-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror',
        '-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER','-I'+str(work),
        '-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Drivers/STM32H7xx_HAL_Driver/Inc'),
        '-I'+str(ROOT/'Drivers/CMSIS/Device/ST/STM32H7xx/Include'),'-I'+str(ROOT/'Drivers/CMSIS/Include'),
        str(ROOT/'tools/tests/test_zdt_query_isr.c'),'-o',str(query_isr_object)],check=True)
    subprocess.run([
        gcc, "-c", "-std=c99", "-mcpu=cortex-m3", "-mthumb", "-O1",
        "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
        "-DSTM32H723xx", "-DUSE_HAL_DRIVER",
        "-I"+str(ROOT/"Core/Inc"),
        "-I"+str(ROOT/"Drivers/STM32H7xx_HAL_Driver/Inc"),
        "-I"+str(ROOT/"Drivers/CMSIS/Device/ST/STM32H7xx/Include"),
        "-I"+str(ROOT/"Drivers/CMSIS/Include"),
        str(ROOT/"tools/tests/test_chassis_field.c"), "-o", str(chassis_test_object),
    ], check=True)
    subprocess.run([
        gcc, "-c", "-std=c99", "-mcpu=cortex-m3", "-mthumb", "-O1",
        "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
        "-DSTM32H723xx", "-DUSE_HAL_DRIVER",
        "-I"+str(ROOT/"Core/Inc"),
        "-I"+str(ROOT/"Drivers/STM32H7xx_HAL_Driver/Inc"),
        "-I"+str(ROOT/"Drivers/CMSIS/Device/ST/STM32H7xx/Include"),
        "-I"+str(ROOT/"Drivers/CMSIS/Include"),
        str(ROOT/"Core/Src/grid_nav.c"), "-o", str(grid_object),
    ], check=True)
    subprocess.run([
        gcc, *(["-DTEST_NAV_PARAMS_ONLY"] if "--params-only" in sys.argv else []), "-std=c99", "-mcpu=cortex-m3", "-mthumb", "-O1",
        "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
        "-I"+str(ROOT/"tools/tests/stubs"), "-I"+str(ROOT/"Core/Inc"), "-I"+str(work),
        str(ROOT/"Core/Src/match_control.c"), str(ROOT/"Core/Src/bt_button.c"),
        str(ROOT/"Core/Src/nav_frame.c"),
        str(ROOT/"Core/Src/nav_blend.c"),
        str(ROOT/"Core/Src/nav_corner.c"),
        str(ROOT/"Core/Src/chassis_field.c"),
        str(ROOT/"Core/Src/zone2_route.c"),
        str(ROOT/"Core/Src/zdt_query.c"),
        str(ROOT/"Core/Src/nav_feedback.c"),
        str(ROOT/"Core/Src/nav_short_move.c"),
        str(ROOT/"Core/Src/nav_odometry.c"),
        str(ROOT/"Core/Src/vision_align.c"), str(ROOT/"Core/Src/vision_yaw.c"),
        str(ROOT/"Core/Src/vision_align_diag.c"),
        str(ROOT/"Core/Src/vision_mode_gate.c"),
        str(ROOT/"Core/Src/vision_qr.c"),
        str(ROOT/"Core/Src/qr_scan.c"),
        str(ROOT/"Core/Src/screen_metrics.c"), str(ROOT/"Core/Src/screen_protocol.c"),
        str(ROOT/"tools/tests/test_screen_protocol.c"),str(ROOT/"tools/tests/test_screen_owner.c"),
        str(ROOT/"tools/tests/test_vision_align.c"),
        str(ROOT/"tools/tests/test_nav_odometry.c"),
        str(ROOT/"tools/tests/test_nav_corner.c"),
        str(ROOT/"tools/tests/test_qr_scan.c"),
        str(ROOT/"tools/tests/test_vision_align_diag.c"),
        str(ROOT/"tools/tests/test_zdt_query.c"),
        str(ROOT/"tools/tests/test_match_control.c"),
        str(nav_object), str(grid_object), str(chassis_object), str(chassis_test_object), str(zone_service_object), str(query_isr_object), str(anchor_object), str(owner_object), str(return_object), str(qr_application_object),str(outer_owner_object),
        "-nostartfiles", "--specs=nosys.specs", "-Wl,--gc-sections",
        "-Wl,-T,"+str(linker), "-lm", "-o", str(binary),
    ], check=True)
    emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    emu.mem_map(0x08000000, 0x100000)
    emu.mem_map(0x20000000, 0x100000)
    with binary.open("rb") as stream:
        elf = ELFFile(stream)
        for seg in elf.iter_segments():
            if seg["p_type"] == "PT_LOAD" and seg["p_filesz"]:
                emu.mem_write(seg["p_vaddr"], seg.data())
        symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
        entry = elf["e_entry"]
    sentinel = 0x080FF000
    completed = []
    def returned(machine, address, size, user_data):
        if address == sentinel:
            completed.append(True)
            machine.emu_stop()
    # Only the return sentinel needs a Python callback. Hooking every ARM
    # instruction consumed the wall-clock budget on otherwise passing runs.
    # Keep the 60s wall limit; 192 real-ramp odometry +9 corner cases need more
    # soft-float instructions than the original transport regression.
    emu.hook_add(UC_HOOK_CODE, returned, begin=sentinel, end=sentinel)
    emu.reg_write(UC_ARM_REG_SP, 0x200FF000)
    emu.reg_write(UC_ARM_REG_LR, sentinel | 1)
    emu.emu_start(entry | 1, sentinel+2, timeout=60_000_000, count=1_600_000_000)
    def value(name):
        return int.from_bytes(emu.mem_read(symbols[name], 4), "little")
    checks = value("test_checks")
    failures = value("test_failures")
    if not completed or failures:
        import struct
        if 'test_corner_metrics' in symbols:
            rows=struct.unpack('<72f',emu.mem_read(symbols['test_corner_metrics'],288))
            for i in range(9):print(f'corner case {i}: time/cross/peak/end/minspeed/progress/odom/stable={rows[i*8:i*8+8]}')
        for i in range(value('test_failure_detail_count')):
            address=int.from_bytes(emu.mem_read(symbols['test_failure_files']+i*4,4),'little')
            file=bytes(emu.mem_read(address,240)).split(b'\0',1)[0].decode(errors='replace')
            line=int.from_bytes(emu.mem_read(symbols['test_failure_lines']+i*4,4),'little')
            print(f'Failed check: {file}:{line}')
        metrics=struct.unpack("<36f",emu.mem_read(symbols["test_blend_metrics"],144))
        for i in range(6):
            print(f"blend case {i}: steps/path/cross/yawerr/encoder/peak_cross={metrics[6*i:6*i+6]}")
        raise SystemExit(f"FAIL: completed={bool(completed)} checks={checks} failures={failures} first C line={value('test_first_failure')}")
    if '--params-only' in sys.argv:
        print(f'PASS: {checks} parameter/Flash assertions; no driving simulation executed')
        raise SystemExit(0)
    import struct
    exit_old=struct.unpack('<3I',emu.mem_read(symbols['test_exit_old_ms'],12))
    if 'test_corner_metrics' in symbols:
        import json
        rows=struct.unpack('<72f',emu.mem_read(symbols['test_corner_metrics'],288))
        names=('time_s','final_cross_mm','peak_cross_mm','end_error_mm','min_corner_speed_mm_s','progress_mm','odometry_difference_mm','stable_ticks')
        metrics=[dict(zip(names,rows[i*8:i*8+8])) for i in range(9)]
        (ROOT/'tmp/nav_corner_validation.json').write_text(json.dumps(dict(hardware_run=False,cases=metrics),indent=2),encoding='utf-8')
        print('Outer arc plant: 9 directions including QR return, continuous corner, real field wheel ramps and80ms wheel queries; max cross=%.2fmm max end=%.2fmm'%(max(v['peak_cross_mm'] for v in metrics),max(v['end_error_mm'] for v in metrics)))
    exit_fast=struct.unpack('<3I',emu.mem_read(symbols['test_exit_fast_ms'],12))
    print(f"Exit plant simulation old/fast ms (0.55,0.80,1.05): {list(zip(exit_old,exit_fast))}")
    print(f"PASS: {checks} assertions; route/UART/odometry/Flash/geometry/ramps; visual pulse, freshness, stable frames, timeout/convergence; camera SET/ACK gate, station identity, rejection, heartbeat loss and sequence")
