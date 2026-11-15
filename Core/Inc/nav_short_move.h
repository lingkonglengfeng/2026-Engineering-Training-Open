#ifndef NAV_SHORT_MOVE_H
#define NAV_SHORT_MOVE_H
#include <stdint.h>
#define NAV_SHORT_MOVE_MAX_MM 150.0f
#define NAV_SHORT_MOVE_MAX_SCALE 0.10f
/* From-rest, low-speed corrective step. Return speed and time BEFORE issuing
 * stop; the subsequent software braking ramp consumes the remaining distance.
 * Hardware slip/driver response still requires field validation. */
uint8_t NavShortMove_Plan(float mm,float requested,float max_rpm,float mm_per_rev,
                          float accel,float decel,uint32_t period_ms,
                          float *scale,uint32_t *hold_ms);
#endif
