"""Cortex-M regression for camera DMA transport, task parsing and raw monitor."""
from pathlib import Path
import importlib.util, shutil, subprocess, sys, tempfile
root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'tmp/vision_test_deps'),str(root/'tmp/maixcam_cv_deps')]
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR
gcc=shutil.which('arm-none-eabi-gcc')
if not gcc: raise SystemExit('arm-none-eabi-gcc required')
with tempfile.TemporaryDirectory(prefix='vision-tests-') as temp:
    work=Path(temp);linker=work/'tests.ld';binary=work/'tests.elf'
    def load(name,path):
        spec=importlib.util.spec_from_file_location(name,root/path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
    material=load('test_camera_material','vision/maixcam_material/main.py')
    plate=load('test_camera_plate','vision/maixcam_material/plate_tangent.py')
    ring=load('test_camera_ring','vision/maixcam_ring/middle_ring.py')
    service=load('test_camera_service','vision/maixcam_service/service.py')
    qr_service=load('test_camera_qr_service','vision/maixcam_qr/service.py')
    class FakeSerial:
        def __init__(self):self.output=bytearray()
        def write(self,data):self.output.extend(data);return len(data)
    class FakeBackend:
        def configure(self,config):pass
    def service_ack(mode,station,target,sequence,fail=False):
        serial=FakeSerial();backend=FakeBackend();clock=lambda:0.0
        server=service.VisionService(serial,backend,clock)
        if fail:
            server.config=(3,2,2,20,0)
            backend.configure=lambda c:(_ for _ in ()).throw(RuntimeError('test')) if c[0]==8 else None
        server.request(service.encode_set(mode,station,target,0 if mode==0 else 20,sequence),0.0)
        server.writer.pump(0.0)
        return bytes(serial.output)
    vectors={
        'material':material.encode_coordinate_frame(3,(60,-40),2028,2),
        'material_next':material.encode_coordinate_frame(3,(31,-21),13),
        'material_bad':material.encode_coordinate_frame(3,(float('nan'),0),14),
        'ring':ring.encode_rloc((90,-60),5,True,True,.9,14,3),
        'plate':plate.encode_plate_frame((30,-40),12,2),
        'origin':material.encode_coordinate_frame(1,(0,0),0),
        'markers':material.encode_coordinate_frame(3,(257.3,-1649.1),13),
        'meta_head':material.encode_coordinate_frame(3,None,13),
        'limits':material.encode_coordinate_frame(1,(-3276.8,-3276.7),15),
        'black':material.encode_coordinate_frame(5,(0,0),0),
        'lightblue':material.encode_coordinate_frame(6,(0,0),1),
        # Optional receiver extension remains tested independently; restored
        # legacy CAM1 never emits it or receives the corresponding SET flag.
        'ring_green':service.make_frame(service.RING_COUNT,3,2,False,bytes((1,3,0,1))),
        'set_material':service.encode_set(2,1,3,20,0),
        'ack_material':service_ack(2,1,3,0),
        'ack_ring':service_ack(3,2,2,1),
        'ack_plate_failure':service_ack(8,1,0,2,True),
        'ack_stop':service_ack(0,0,0,3),
        'ack_retry_ring':service_ack(3,2,2,4),
        'hb_stop':service.make_frame(0x82,0,0,True,bytes((1,0,0,0))),
        'hb_material':service.make_frame(0x82,0,0,True,bytes((1,2,1,0))),
        'hb_ring':service.make_frame(0x82,0,0,True,bytes((1,3,2,0))),
        'hb_unknown':service.make_frame(0x82,0,0,True,bytes((2,3,2,0))),
    }
    qr_packet=qr_service.encode_task('156+123+516+231',5)
    if len(qr_packet)!=32: raise SystemExit('CAM2 task must be four complete frames')
    for i in range(4):vectors[f'qr_group{i+1}']=qr_packet[i*8:(i+1)*8]
    (work/'vision_protocol8_vectors.h').write_text(''.join(
        'static const uint8_t vector_'+name+'[8]={'+','.join('0x%02X'%b for b in data)+'};\n'
        for name,data in vectors.items()))
    linker.write_text('''ENTRY(main)
SECTIONS {
 . = 0x08000000; .text : { *(.text*) *(.rodata*) }
 .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; }
 . = 0x20000000; .data : { *(.data*) }
 .bss : { *(.bss*) *(COMMON) }
 .vision_dma : { *(.vision_dma*) }
 . = ALIGN(8); end = .;
}
''')
    includes=['Core/Inc','Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32H7xx/Include','Drivers/CMSIS/Include','Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2']
    subprocess.run([gcc,'-std=c99','-mcpu=cortex-m3','-mthumb','-O1','-Wall','-Wextra','-Werror','-ffunction-sections','-fdata-sections','-DSTM32H723xx','-DUSE_HAL_DRIVER',
        *['-I'+str(root/p) for p in includes],'-I'+str(work),str(root/'Core/Src/vision.c'),
        str(root/'Core/Src/vision_mode_gate.c'),str(root/'Core/Src/vision_align.c'),str(root/'Core/Src/vision_qr.c'),str(root/'tools/tests/test_vision.c'),
        '-nostartfiles','--specs=nosys.specs','-Wl,--gc-sections','-Wl,-T,'+str(linker),'-lm','-o',str(binary)],check=True)
    emu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    emu.mem_map(0x08000000,0x100000);emu.mem_map(0x20000000,0x100000)
    with binary.open('rb') as stream:
        elf=ELFFile(stream)
        for seg in elf.iter_segments():
            if seg['p_type']=='PT_LOAD' and seg['p_filesz']: emu.mem_write(seg['p_vaddr'],seg.data())
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()};entry=elf['e_entry']
    done=[];sentinel=0x080ff000
    def hook(machine,address,size,user):
        if address==sentinel:done.append(True);machine.emu_stop()
    emu.hook_add(UC_HOOK_CODE,hook,begin=sentinel,end=sentinel);emu.reg_write(UC_ARM_REG_SP,0x200ff000);emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=30_000_000,count=30_000_000)
    def value(name):return int.from_bytes(emu.mem_read(symbols[name],4),'little')
    if not done or value('test_failures'):
        raise SystemExit(f"FAIL: completed={bool(done)} checks={value('test_checks')} failures={value('test_failures')} first line={value('test_first_failure')}")
    print(f"PASS: {value('test_checks')} assertions; Python-to-C pixels/ACK/heartbeat, SET async/retry/failure/timeout, CRC/split/markers/freshness/context/ports, legacy/DMA/raw/IMU")
