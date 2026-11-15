"""Cortex-M production obstacle math/map and extracted motion-owner checks."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

ROOT = Path(__file__).resolve().parents[2]
GCC = shutil.which('arm-none-eabi-gcc')
if not GCC:
    raise SystemExit('arm-none-eabi-gcc required')


def run(work, source, name):
    linker = work / (name + '.ld')
    binary = work / (name + '.elf')
    linker.write_text('ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n')
    command = [GCC, '-std=c99', '-mcpu=cortex-m3', '-mthumb', '-O1', '-Wall', '-Wextra', '-Werror',
        '-Wno-misleading-indentation', '-ffunction-sections', '-fdata-sections',
        '-DSTM32H723xx', '-DUSE_HAL_DRIVER', '-I'+str(work)]
    for directory in ('Core/Inc', 'Drivers/STM32H7xx_HAL_Driver/Inc',
                      'Drivers/CMSIS/Device/ST/STM32H7xx/Include', 'Drivers/CMSIS/Include'):
        command.append('-I'+str(ROOT / directory))
    command += [str(source), str(ROOT/'Core/Src/vision_obstacle.c'), str(ROOT/'Core/Src/grid_nav.c'),
                str(ROOT/'Core/Src/zone2_route.c'), str(ROOT/'Core/Src/nav_feedback.c'),
                str(ROOT/'Core/Src/nav_frame.c'), '-nostartfiles', '--specs=nosys.specs',
                '-Wl,--gc-sections', '-Wl,-T,'+str(linker), '-lm', '-o', str(binary)]
    subprocess.run(command, check=True)
    machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    machine.mem_map(0x08000000, 0x200000);machine.mem_map(0x20000000, 0x200000)
    with binary.open('rb') as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                machine.mem_write(segment['p_vaddr'], segment.data())
        symbols = {s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        entry = elf['e_entry']
    sentinel = 0x081ff000;done = []
    def finish(cpu, address, size, user):
        done.append(True);cpu.emu_stop()
    machine.hook_add(UC_HOOK_CODE, finish, begin=sentinel, end=sentinel)
    machine.reg_write(UC_ARM_REG_SP, 0x201ff000);machine.reg_write(UC_ARM_REG_LR, sentinel | 1)
    machine.emu_start(entry | 1, sentinel+2, timeout=30_000_000, count=200_000_000)
    def value(symbol):
        return int.from_bytes(machine.mem_read(symbols[symbol], 4), 'little')
    print('%s: %d checks, %d failures; first line=%d' % (name, value('checks'), value('failures'), value('first_failure')))
    if value('failures') and 'debug_phase' in symbols:
        print('debug:',{key:value(key) for key in ('debug_phase','debug_target','debug_recenter','debug_started','debug_goal_x','debug_goal_y','debug_map_a','debug_map_b','debug_map_c')})
    if not done or not value('checks') or value('failures'):
        raise SystemExit(name + ' failed')


with tempfile.TemporaryDirectory(prefix='vision-obstacle-tests-') as directory:
    work = Path(directory)
    run(work, ROOT/'tools/tests/test_vision_obstacle.c', 'obstacle-math-map')
    owner = ROOT/'tools/tests/test_vision_obstacle_owner.c'
    if owner.exists():
        source = (ROOT/'Core/Src/zdt_motor.c').read_text(encoding='utf-8')
        begin = source.index('static float ZDT_Vobs_WorldYaw(void)\n{')
        end = source.index('/* ---------- Serial-stepped competition flow', begin)
        (work/'vobs_owner_under_test.inc').write_text(source[begin:end], encoding='utf-8')
        def function(signature):
            begin = source.index(signature + '\n{')
            brace = source.index('{', begin);end = brace + 1;depth = 1
            while depth:
                depth += (source[end] == '{') - (source[end] == '}');end += 1
            return source[begin:end]
        callbacks = [function('static void ' + name + '(void)')
                     for name in ('ZDT_Zone2_Done', 'ZDT_Nav_NotifyArrived')]
        (work/'vobs_arrival_under_test.inc').write_text('\n'.join(callbacks), encoding='utf-8')
        routes = [function('static uint8_t ZDT_Zone2_TryDepartureQrStraight(const zone2_action_t *a,float yaw)'),
                  function('static uint8_t ZDT_Zone2_StartAction(const zone2_action_t *a,uint32_t id)'),
                  function('static void ZDT_Nav_Start(void)').replace('ZDT_Nav_Start(void)', 'ZDT_Nav_StartBody(void)', 1)]
        (work/'vobs_routes_under_test.inc').write_text('\n'.join(routes), encoding='utf-8')
        reverse_begin=source.index('        if(zdt_nav_recenter_route && (!zdt_vobs_nav_active')
        reverse_end=source.index('        /*',reverse_begin)
        (work/'vobs_reverse_under_test.inc').write_text(source[reverse_begin:reverse_end],encoding='utf-8')
        cap_begin=source.index('    if(zdt_nav_recenter_route && !zdt_vobs_nav_active && distance_rev')
        cap_end=source.index(';',cap_begin)+1
        (work/'vobs_recenter_cap_under_test.inc').write_text(source[cap_begin:cap_end],encoding='utf-8')
        final_begin=source.index('        if(zdt_nav_odometry_enabled && ((zdt_nav_start_mask&zdt_nav_end_mask)')
        final_brace=source.index('{',final_begin);final_end=final_brace+1;depth=1
        while depth:
            depth+=(source[final_end]=='{')-(source[final_end]=='}');final_end+=1
        (work/'vobs_final_check_under_test.inc').write_text(source[final_begin:final_end],encoding='utf-8')
        match_begin=source.index('nav_motion_feedback_t check = ')
        match_end=source.index(';',match_begin)+1
        (work/'vobs_match_check_under_test.inc').write_text(source[match_begin:match_end],encoding='utf-8')
        run(work, owner, 'obstacle-motion-owner')
