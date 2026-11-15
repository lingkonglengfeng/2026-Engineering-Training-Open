"""Compile the real arm driver with real STM32 headers; execute mocked ports in Unicorn.

Requires arm-none-eabi-gcc, unicorn==2.1.4 and pyelftools==0.32.
Uses temporary outputs only; never changes project files or flashes a board.
Default suite checks the legacy expiry policy (ARM_ZDT_WAIT_ON_STALE=0).
Run --feedback-wait-only for the production V179 wait/recovery policy (=1).
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

ROOT = Path(__file__).resolve().parents[2]
gcc = shutil.which("arm-none-eabi-gcc")
if not gcc:
    raise SystemExit("arm-none-eabi-gcc is required")
with tempfile.TemporaryDirectory(prefix="arm-driver-tests-") as temp:
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
    for variant in (('camera replay',) if '--camera-replay-only' in sys.argv else ('camera hold',) if '--camera-hold-only' in sys.argv else ('native lift',) if '--native-lift-only' in sys.argv else ('speed profile',) if '--speed-profile-only' in sys.argv else ('servo only',) if '--servo-only' in sys.argv else ('feedback wait',) if '--feedback-wait-only' in sys.argv else ('default driver zero','DM internal zero','measured camera default','replacement reach direction','native reach retries','28 stall trial') if '--legacy-tail-only' in sys.argv else ('28 stall trial',) if '--trial-only' in sys.argv else ('calibrated axes','uncalibrated reach','default driver zero','DM internal zero','measured camera default','replacement reach direction','native reach retries','28 stall trial')):
        uncalibrated = variant == 'uncalibrated reach'
        binary = work / "tests.elf"
        command = [
            gcc, "-std=c99", "-mcpu=cortex-m3", "-mthumb", "-O1",
            "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
            "-DSTM32H723xx", "-DUSE_HAL_DRIVER",
            "-I"+str(ROOT/"Core/Inc"),
            "-I"+str(ROOT/"Drivers/STM32H7xx_HAL_Driver/Inc"),
            "-I"+str(ROOT/"Drivers/CMSIS/Device/ST/STM32H7xx/Include"),
            "-I"+str(ROOT/"Drivers/CMSIS/Include"),
            str(ROOT/"Core/Src/arm_driver.c"), str(ROOT/"Core/Src/arm_calibration.c"),
            str(ROOT/"Core/Src/arm_servo.c"), str(ROOT/"Core/Src/screen_metrics.c"),
            str(ROOT/"tools/tests/test_arm_driver.c"),
            "-nostartfiles", "--specs=nosys.specs", "-Wl,--gc-sections",
            "-Wl,-T,"+str(linker), "-lm", "-o", str(binary),
        ]
        command.insert(1, '-DARM_ZDT_WAIT_ON_STALE='+('1' if variant in ('camera replay','camera hold','feedback wait','servo only','speed profile','native lift') else '0'))
        if variant=='camera replay':command.insert(1,'-DTEST_CAMERA_REPLAY_ONLY')
        if variant=='camera hold':command.insert(1,'-DTEST_CAMERA_HOLD_ONLY')
        if variant=='feedback wait':command.insert(1,'-DTEST_FEEDBACK_WAIT')
        if variant=='native lift':command.insert(1,'-DTEST_NATIVE_LIFT')
        if variant=='servo only':command.insert(1,'-DTEST_SERVO_ONLY')
        if variant=='speed profile':command.insert(1,'-DTEST_SPEED_PROFILE_ONLY')
        if variant == '28 stall trial':
            command.insert(1, '-DARM_28_STALL_IGNORE_TRIAL=1')
            command.insert(1, '-DTEST_28_STALL_TRIAL')
            command.insert(1, '-DTEST_REACH_RETRY')
        else:
            command.insert(1, '-DARM_28_STALL_IGNORE_TRIAL=0')
        if variant == 'measured camera default':
            command.insert(1, '-DTEST_MEASURED_CAMERA')
        if variant == 'replacement reach direction':
            command.insert(1, '-DTEST_REPLACEMENT_REACH')
        if variant == 'native reach retries':
            command.insert(1, '-DTEST_REACH_RETRY')
        if variant == 'DM internal zero':
            command.insert(1, '-DTEST_DM_INTERNAL_ZERO')
        else:
            command.insert(1, '-DARM_DM_SOFTWARE_ZERO_ENABLED=1')
        if uncalibrated:
            command.insert(1, "-DTEST_UNCALIBRATED")
        if variant == 'default driver zero':
            command.insert(1, '-DTEST_DEFAULT_ZERO')
        subprocess.run(command, check=True)
        emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        emu.mem_map(0x08000000, 0x100000)
        emu.mem_map(0x20000000, 0x100000)
        emu.mem_map(0x40000000, 0x10000) # TIM4 registers for real CCR-write macros
        with binary.open("rb") as stream:
            elf = ELFFile(stream)
            for segment in elf.iter_segments():
                if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                    emu.mem_write(segment["p_vaddr"], segment.data())
            symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
            entry = elf["e_entry"]
        sentinel = 0x080FF000
        completed = []

        def returned(machine, address, size, user_data):
            if address == sentinel:
                completed.append(True)
                machine.emu_stop()

        emu.hook_add(UC_HOOK_CODE, returned)
        emu.reg_write(UC_ARM_REG_SP, 0x200FF000)
        emu.reg_write(UC_ARM_REG_LR, sentinel | 1)
        # The full facade variant now includes the real CAMERA residual
        # controller and its bounded fault/cancel cases, in addition to
        # native recovery. Keep a finite emulator budget for the larger suite.
        emu.emu_start(entry | 1, sentinel+2, timeout=60_000_000, count=80_000_000)

        def value(name):
            return int.from_bytes(emu.mem_read(symbols[name], 4), "little")

        checks, failures = value("test_checks"), value("test_failures")
        label = variant
        print(f"{label}: {checks} checks, {failures} failures")
        if failures:
            if variant=='28 stall trial':
                print('Trial camera/calibration state:',value('test_trial_camera_state'),value('test_trial_cal_state'))
                print('Trial camera reason:',bytes(emu.mem_read(symbols['test_trial_camera_reason'],96)).split(b'\0')[0].decode())
            raw = emu.mem_read(symbols["test_failure_lines"], min(failures, 64)*4)
            print("Failed test lines:", [int.from_bytes(raw[i:i+4], "little") for i in range(0, len(raw), 4)])
        if not completed or failures or not checks:
            raise SystemExit("arm driver tests failed")
