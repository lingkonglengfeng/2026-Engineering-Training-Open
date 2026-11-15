#ifndef PID_STORAGE_H
#define PID_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_parameters_t;

/* Returns 1 when the command belongs to this module, otherwise 0. */
uint8_t PID_Storage_ProcessCommand(const char *command);
uint8_t PID_Storage_Load(pid_parameters_t *parameters);
/* 上电装载：把 W25 里保存的转向 PID 装回 RAM（无有效记录则保持编译默认）。
 * 只能由 ZDT_Motor_Begin() 在 W25 可用后调用一次；返回 1=使用了 W25 记录。
 * V236 起生效：&save 过的参数断电重启后自动恢复，不再退回 0.045/0.0035。 */
uint8_t PID_Storage_ApplyBoot(void);
/* 当前生效 PID 的来源："DEFAULT" / "W25Q64" / "MANUAL"（供 &pid/&tstat 显示）。 */
const char *PID_Storage_Source(void);

#ifdef __cplusplus
}
#endif

#endif /* PID_STORAGE_H */
