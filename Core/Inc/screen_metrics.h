#ifndef SCREEN_METRICS_H
#define SCREEN_METRICS_H
#include <stddef.h>
#include <stdint.h>
typedef enum {SCREEN_WAIT_QR,SCREEN_SCAN_OK,SCREEN_RUNNING,SCREEN_DONE,SCREEN_ERROR} screen_state_t;
typedef struct {char task[16];uint32_t pick,place;screen_state_t state;} screen_snapshot_t;
/* Code/state have one writer (ZDT owner); counters/reset have one writer
 * (ARM owner). HMI copies a snapshot under an IRQ critical section. No ISR
 * calls these setters; all are bounded and never send a UART command. */
void ScreenMetrics_ClearTask(void);
uint8_t ScreenMetrics_SetTask(const char *raw);
void ScreenMetrics_SetState(screen_state_t state);
void ScreenMetrics_Pick(void);
void ScreenMetrics_Place(void);
void ScreenMetrics_ResetCounts(void);
void ScreenMetrics_Snapshot(screen_snapshot_t *out);
size_t ScreenMetrics_Command(const screen_snapshot_t *s,uint8_t field,uint8_t *out,size_t capacity);
#endif
