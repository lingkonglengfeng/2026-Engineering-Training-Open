"""Read-only diagnostic FSM and reply demultiplexing, using production C."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

root = Path(__file__).resolve().parents[2]
source = (root / "Core/Src/dm_motor.c").read_text(encoding="utf-8")
block = source[source.index("static const uint8_t DM_DiagRids"):source.index("static HAL_StatusTypeDef DM_SendPosFrame")]
def function(signature):
    start=source.index(signature);end=source.index("{",start)+1;depth=1
    while depth:
        depth+=(source[end]=="{")-(source[end]=="}");end+=1
    return source[start:end]
rx = function("static float DM_UintToFloat(")+function("void DM_RxFifo0Handler(")
syscalls = (root / "tools/tests/test_arm_driver.c").read_text(encoding="utf-8")
syscalls = syscalls[syscalls.index("int _close("):syscalls.index("volatile unsigned int test_checks")]
prefix = r'''
#include "dm_motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
volatile uint32_t checks,failures,first_failure;
#define CHECK(x) do{checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
static uint32_t tick=1000,reads,unknowns,invalids;
static HAL_StatusTypeDef send_result=HAL_OK;
static dm_motor_t motor;
static uint8_t rx_data[64],rx_pending;
static FDCAN_RxHeaderTypeDef rx_header;
#define DM_FRAME_MAX_BYTES 64U
#define DM_FB_LENGTH 8U
#define DM_FB_STATE_ENABLED 1U
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *can,uint32_t fifo,FDCAN_RxHeaderTypeDef *header,uint8_t *data){
 CHECK(can==motor.cfg.can && fifo==FDCAN_RX_FIFO0);
 if(!rx_pending)return HAL_ERROR;
 *header=rx_header;memcpy(data,rx_data,8);rx_pending=0;return HAL_OK;
}
static void DM_Print(const char *line){CHECK(line!=NULL);if(strstr(line,"UNKNOWN"))unknowns++;if(strstr(line,"INVALID"))invalids++;}
static HAL_StatusTypeDef DM_SendFrame(dm_motor_t *m,uint16_t id,const uint8_t *data,uint8_t len){
 CHECK(m==&motor && id==0x7FFU && len==4U);
 CHECK(data[0]==1U && data[1]==0U && data[2]==0x33U); /* no FC/FD/FE/55/AA */
 reads++;return send_result;
}
'''
suffix = r'''
static uint32_t value(uint8_t rid){
 float f=0;
 if(rid==10){return 2;} if(rid==13){return 1;} if(rid==14){return 7013;}
 if(rid==20){f=10;} if(rid==21){f=18.5f;} if(rid==22){f=30;} if(rid==23){f=10;}
 if(rid==56){f=0.125f;} if(rid==80){f=0;} if(rid==81){f=1.57f;}
 {uint32_t raw;memcpy(&raw,&f,4);return raw;}
}
static void reset(void){
 rx_pending=0;
 memset(&motor,0,sizeof(motor));motor.cfg.can=(FDCAN_HandleTypeDef *)0x20000000;
 motor.cfg.esc_id=1;motor.cfg.mst_id=17;motor.active_mode=DM_POS_MODE;
 motor.state=DM_STATE_ENABLED;motor.enable_sent=1;motor.fb.enabled=1;motor.fb.fb_count=7;
 motor.fb.pos_rad=0.5f;motor.fb.last_tick=tick;motor.last_rx_tick=tick;
 motor.target_pos_rad=1.25f;motor.target_vel_rads=0.3f;motor.pos_offset_rad=0.25f;
 reads=unknowns=invalids=0;send_result=HAL_OK;
}
static void run(uint8_t respond){
 unsigned int i;
 for(i=0;i<250 && motor.diag_active;i++){
  tick+=10;DM_Motor_DiagPoll(&motor,tick);
  if(respond && motor.diag_expected && !motor.diag_reply_ready){
   uint8_t d[8]={1,0,0x33,motor.diag_rid,0,0,0,0};uint32_t raw=value(d[3]);memcpy(d+4,&raw,4);
   CHECK(DM_DiagReply(&motor,d,tick)==1);
  }
 }
 CHECK(!motor.diag_active && reads==10);
 CHECK(motor.fb.fb_count==7 && motor.fb.pos_rad==0.5f && motor.fb.enabled && motor.state==DM_STATE_ENABLED);
 CHECK(motor.target_pos_rad==1.25f && motor.target_vel_rads==0.3f && motor.pos_offset_rad==0.25f);
 CHECK(motor.enable_sent==1 && motor.fault_latched==0 && motor.resend_hold==0);
}
int main(void){
 uint8_t d[8]={1,0,0x33,80,0,0,0,0};uint32_t raw=0x3F800000,before_rx;
 reset();CHECK(DM_Motor_DiagStart(&motor)==HAL_OK);CHECK(DM_Motor_DiagStart(&motor)==HAL_BUSY);
 run(1);CHECK(unknowns==0);
 reset();CHECK(DM_Motor_DiagStart(&motor)==HAL_OK);run(0);CHECK(unknowns==10);
 reset();send_result=HAL_BUSY;CHECK(DM_Motor_DiagStart(&motor)==HAL_OK);run(0);CHECK(unknowns==10);
 reset();before_rx=motor.last_rx_tick;CHECK(DM_Motor_DiagStart(&motor)==HAL_OK);DM_Motor_DiagPoll(&motor,tick);
 memcpy(d+4,&raw,4);CHECK(DM_DiagReply(&motor,d,tick)==1 && !motor.diag_reply_ready); /* wrong RID */
 d[3]=10;CHECK(DM_DiagReply(&motor,d,tick-1)==1 && !motor.diag_reply_ready); /* stale */
 d[0]=2;CHECK(DM_DiagReply(&motor,d,tick)==0 && !motor.diag_reply_ready);
 d[0]=1;CHECK(DM_DiagReply(&motor,d,tick)==1 && motor.diag_reply_ready);
 CHECK(motor.fb.fb_count==7 && motor.fb.pos_rad==0.5f && motor.last_rx_tick==before_rx);
 DM_Motor_DiagCancel(&motor);CHECK(!motor.diag_active && !motor.diag_expected);
 CHECK(DM_DiagReply(&motor,d,tick)==1 && !motor.diag_reply_ready); /* late reply: never FB */
 reset();motor.fb.vel_rads=0.2f;CHECK(DM_Motor_DiagStart(&motor)==HAL_BUSY && reads==0);
 reset();CHECK(DM_Motor_DiagStart(&motor)==HAL_OK);motor.fb.vel_rads=0.2f;
 DM_Motor_DiagPoll(&motor,tick);CHECK(!motor.diag_active && reads==0);
 reset();CHECK(DM_Motor_DiagStart(NULL)==HAL_ERROR);
 CHECK(DM_Motor_DiagStart(&motor)==HAL_OK);motor.diag_index=8;DM_Motor_DiagPoll(&motor,tick);
 d[3]=80;raw=0x7FC00000;memcpy(d+4,&raw,4);CHECK(DM_DiagReply(&motor,d,tick)==1);
 DM_Motor_DiagPoll(&motor,tick);CHECK(invalids==1 && motor.fb.fb_count==7);DM_Motor_DiagCancel(&motor);
 reset();tick=UINT32_MAX-20;motor.fb.last_tick=tick;
 CHECK(DM_Motor_DiagStart(&motor)==HAL_OK);run(1);CHECK(unknowns==0);
 /* Exercise the production IRQ dispatch: parameter packets never overwrite
  * position/enable or fake a new motion feedback timestamp. */
 reset();CHECK(DM_Motor_DiagStart(&motor)==HAL_OK);motor.diag_index=8;DM_Motor_DiagPoll(&motor,tick);
 memset(&rx_header,0,sizeof(rx_header));rx_header.IdType=FDCAN_STANDARD_ID;rx_header.Identifier=17;
 rx_header.RxFrameType=FDCAN_DATA_FRAME;rx_header.FDFormat=FDCAN_CLASSIC_CAN;rx_header.DataLength=FDCAN_DLC_BYTES_8;
 d[3]=80;raw=0x3F800000;memcpy(d+4,&raw,4);memcpy(rx_data,d,8);rx_pending=1;
 before_rx=motor.last_rx_tick;DM_RxFifo0Handler(&motor);
 CHECK(motor.diag_reply_ready && motor.fb.fb_count==7 && motor.fb.pos_rad==0.5f && motor.fb.enabled && motor.last_rx_tick==before_rx);
 DM_Motor_DiagCancel(&motor);rx_pending=1;DM_RxFifo0Handler(&motor);CHECK(motor.fb.fb_count==7);
 {const uint8_t fb[8]={0x11,0x7F,0xFF,0x7F,0xFF,0xFF,30,30};memcpy(rx_data,fb,8);rx_pending=1;tick++;DM_RxFifo0Handler(&motor);}
 CHECK(motor.fb.fb_count==8 && motor.fb.enabled && fabsf(motor.fb.pos_rad)<0.001f && motor.last_rx_tick==tick);
 /* Reported motor PMAX18.5 must decode p_m0.4734838 correctly. */
 {uint16_t p=(uint16_t)round((0.4734838+18.5)*65535.0/37.0);
  rx_data[1]=(uint8_t)(p>>8);rx_data[2]=(uint8_t)p;rx_pending=1;DM_RxFifo0Handler(&motor);
  CHECK(DM_P_MAX==18.5f && fabsf(motor.fb.pos_rad-0.4734838f)<0.001f && motor.fb.enabled);}
 return failures;
}
'''
with tempfile.TemporaryDirectory(prefix="dm-diag-tests-") as temp:
    work=Path(temp)
    c=work/"test.c"
    c.write_text(prefix+syscalls+block+rx+suffix,encoding="utf-8")
    linker=work/"test.ld"
    linker.write_text("ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n")
    binary=work/"test.elf"
    gcc=shutil.which("arm-none-eabi-gcc")
    if not gcc: raise SystemExit("arm-none-eabi-gcc required")
    includes=[root/"Core/Inc",root/"Drivers/STM32H7xx_HAL_Driver/Inc",root/"Drivers/CMSIS/Device/ST/STM32H7xx/Include",root/"Drivers/CMSIS/Include"]
    subprocess.run([gcc,"-std=c99","-mcpu=cortex-m3","-mthumb","-O1","-Wall","-Wextra","-Werror",
        "-DSTM32H723xx","-DUSE_HAL_DRIVER",*["-I"+str(x) for x in includes],"-ffunction-sections","-fdata-sections",
        str(c),"-nostartfiles","--specs=nosys.specs","-Wl,--gc-sections","-Wl,-T,"+str(linker),"-lm","-o",str(binary)],check=True)
    emu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    emu.mem_map(0x08000000,0x100000);emu.mem_map(0x20000000,0x100000)
    with binary.open("rb") as stream:
        elf=ELFFile(stream)
        for segment in elf.iter_segments():
            if segment["p_type"]=="PT_LOAD" and segment["p_filesz"]:emu.mem_write(segment["p_vaddr"],segment.data())
        symbols={s.name:s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()};entry=elf["e_entry"]
    sentinel=0x080FF000;completed=[]
    def returned(machine,address,size,user_data):completed.append(True);machine.emu_stop()
    emu.hook_add(UC_HOOK_CODE,returned,begin=sentinel,end=sentinel)
    emu.reg_write(UC_ARM_REG_SP,0x200FF000);emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=10_000_000,count=5_000_000)
    def result(name):return int.from_bytes(emu.mem_read(symbols[name],4),"little")
    print(f"DM diag: {result('checks')} checks, {result('failures')} failures; first line={result('first_failure')}")
    if not completed or not result("checks") or result("failures"):raise SystemExit("DM diag test failed")
