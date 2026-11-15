"""Exercise the actual Emm FD sender with captured CAN packets, no hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

root = Path(__file__).resolve().parents[2]
source = (root / "Core/Src/zdt_motor.c").read_text(encoding="utf-8")
def extract(signature):
    start=source.index(signature);brace=source.index('{',start);end=brace+1;depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]
function='\n'.join(extract(x) for x in (
    'static HAL_StatusTypeDef ZDT_MovePositionModeAddr(',
    'HAL_StatusTypeDef ZDT_Motor_MovePositionPulsesAddr(',
    'HAL_StatusTypeDef ZDT_Motor_MovePositionPulsesAbsAddr('))
prefix = r'''
#include <stdint.h>
#include <string.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2 } HAL_StatusTypeDef;
#define ZDT_CAN_RUN_MAX_RPM 3000L
#define ZDT_CAN_CHECKSUM 0x6BU
volatile uint32_t checks,failures;
#define CHECK(x) do { checks++;if(!(x))failures++; }while(0)
static uint32_t calls;
static uint8_t packets[2][8],lengths[2],ids[2],indexes[2];
static HAL_StatusTypeDef first_result=HAL_OK,second_result=HAL_OK;
static HAL_StatusTypeDef ZDT_CAN_SendPacketToAddress(uint8_t address,uint8_t index,
    const uint8_t *data,uint8_t length,const char *label,uint8_t logging) {
    CHECK(calls<2 && label!=0 && logging==0);
    if(calls>=2)return HAL_ERROR;
    ids[calls]=address;indexes[calls]=index;lengths[calls]=length;
    memcpy(packets[calls],data,length);calls++;
    return calls==1?first_result:second_result;
}
'''
suffix = r'''
int main(void) {
    const uint8_t p0[8]={0xFD,0,0,25,100,0,0,0x0A};
    const uint8_t p1[5]={0xFD,0x6B,2,0,0x6B};
    CHECK(ZDT_Motor_MovePositionPulsesAddr(6,2667,25,100,0)==HAL_OK);
    CHECK(calls==2 && ids[0]==6 && ids[1]==6 && indexes[0]==0 && indexes[1]==1);
    CHECK(lengths[0]==8 && lengths[1]==5);
    CHECK(memcmp(packets[0],p0,8)==0 && memcmp(packets[1],p1,5)==0);
    calls=0;CHECK(ZDT_Motor_MovePositionPulsesAddr(6,5333,50,150,0)==HAL_OK);
    CHECK(calls==2 && packets[0][2]==0 && packets[0][3]==50 && packets[0][4]==150);
    CHECK(packets[0][5]==0 && packets[0][6]==0 && packets[0][7]==0x14 && packets[1][1]==0xD5);
    CHECK(packets[1][2]==2 && packets[1][3]==0 && packets[1][4]==0x6B);
    calls=0;CHECK(ZDT_Motor_MovePositionPulsesAddr(6,5333,80,150,0)==HAL_OK);
    CHECK(calls==2 && packets[0][2]==0 && packets[0][3]==80 && packets[0][4]==150);
    CHECK(packets[0][7]==0x14 && packets[1][1]==0xD5 && packets[1][2]==2);
    calls=0;CHECK(ZDT_Motor_MovePositionPulsesAddr(6,-3200,25,100,0)==HAL_OK);
    CHECK(packets[0][1]==1 && packets[0][7]==0x0C && packets[1][1]==0x80 && packets[1][2]==2);
    calls=0;CHECK(ZDT_Motor_MovePositionPulsesAbsAddr(6,5333,80,150,0)==HAL_OK);
    CHECK(calls==2 && packets[0][1]==0 && packets[0][7]==0x14 && packets[1][1]==0xD5 && packets[1][2]==1);
    calls=0;CHECK(ZDT_Motor_MovePositionPulsesAbsAddr(6,3321,80,150,0)==HAL_OK);
    CHECK(packets[1][2]==1 && packets[0][7]==0x0C && packets[1][1]==0xF9);
    calls=0;CHECK(ZDT_Motor_MovePositionPulsesAbsAddr(6,0,80,150,0)==HAL_OK && calls==2 && packets[1][2]==1);
    calls=0;CHECK(ZDT_Motor_MovePositionPulsesAddr(6,2667,0,100,0)==HAL_ERROR && calls==0);
    CHECK(ZDT_Motor_MovePositionPulsesAddr(6,2667,3001,100,0)==HAL_ERROR && calls==0);
    first_result=HAL_BUSY;CHECK(ZDT_Motor_MovePositionPulsesAddr(6,2667,25,100,0)==HAL_BUSY && calls==1);
    calls=0;first_result=HAL_OK;second_result=HAL_ERROR;
    CHECK(ZDT_Motor_MovePositionPulsesAddr(6,2667,25,100,0)==HAL_ERROR && calls==2);
    return failures;
}
'''
with tempfile.TemporaryDirectory(prefix="zdt-native-tests-") as temp:
    work = Path(temp)
    c = work / "test.c"
    c.write_text(prefix + function + suffix, encoding="utf-8")
    linker = work / "test.ld"
    linker.write_text("ENTRY(main)\nSECTIONS { . = 0x08000000; .text : { *(.text*) *(.rodata*) } .ARM.exidx : { *(.ARM.exidx*) } . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) } . = ALIGN(8); end = .; }\n")
    binary = work / "test.elf"
    gcc = shutil.which("arm-none-eabi-gcc")
    if not gcc:
        raise SystemExit("arm-none-eabi-gcc required")
    subprocess.run([gcc,"-std=c99","-mcpu=cortex-m3","-mthumb","-O1","-Wall","-Wextra","-Werror",
        "-ffunction-sections","-fdata-sections",str(c),"-nostartfiles","--specs=nosys.specs",
        "-Wl,--gc-sections","-Wl,-T,"+str(linker),"-o",str(binary)],check=True)
    emu = Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    emu.mem_map(0x08000000,0x100000)
    emu.mem_map(0x20000000,0x100000)
    with binary.open("rb") as stream:
        elf=ELFFile(stream)
        for segment in elf.iter_segments():
            if segment["p_type"]=="PT_LOAD" and segment["p_filesz"]:
                emu.mem_write(segment["p_vaddr"],segment.data())
        symbols={s.name:s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
        entry=elf["e_entry"]
    sentinel=0x080FF000
    completed=[]
    def returned(machine,address,size,user_data):
        completed.append(True)
        machine.emu_stop()
    emu.hook_add(UC_HOOK_CODE,returned,begin=sentinel,end=sentinel)
    emu.reg_write(UC_ARM_REG_SP,0x200FF000)
    emu.reg_write(UC_ARM_REG_LR,sentinel|1)
    emu.emu_start(entry|1,sentinel+2,timeout=5_000_000,count=1_000_000)
    def value(name):
        return int.from_bytes(emu.mem_read(symbols[name],4),"little")
    print(f"native FD packets: {value('checks')} checks, {value('failures')} failures")
    if not completed or not value("checks") or value("failures"):
        raise SystemExit("native FD packet test failed")
