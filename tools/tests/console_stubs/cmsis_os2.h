#include <stdint.h>
typedef void *osThreadId_t;
typedef enum {osKernelInactive,osKernelRunning} osKernelState_t;
#define osFlagsWaitAny 0U
osKernelState_t osKernelGetState(void);
uint32_t osThreadFlagsSet(osThreadId_t,uint32_t);
uint32_t osThreadFlagsWait(uint32_t,uint32_t,uint32_t);
osThreadId_t osThreadGetId(void);
