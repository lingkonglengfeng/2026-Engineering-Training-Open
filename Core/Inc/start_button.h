#ifndef START_BUTTON_H
#define START_BUTTON_H
#include <stdint.h>
#define START_BUTTON_DEBOUNCE_MS 30U
#define START_BUTTON_LONG_PRESS_MS 1000U
/* ZDT task owns this state. A stable release arms exactly one closure. */
typedef struct {
    uint32_t changed_tick;
    uint8_t candidate_released,stable_released,armed;
    uint32_t pressed_at;
    uint8_t gesture_pending;
} start_button_t;
static inline void StartButton_Init(start_button_t *b,uint8_t released,uint32_t now)
{
    b->candidate_released=b->stable_released=(uint8_t)(released!=0U);
    b->changed_tick=now;b->armed=0U;b->pressed_at=now;b->gesture_pending=0U;
}
/* Apply the hardware edge timestamp, even when task dispatch was delayed. */
static inline void StartButton_OnEdge(start_button_t *b,uint8_t released,uint32_t changed_tick)
{
    b->candidate_released=(uint8_t)(released!=0U);b->changed_tick=changed_tick;
}
static inline uint8_t StartButton_Update(start_button_t *b,uint8_t released,
                                         uint32_t now,uint32_t *pressed_tick)
{
    released=(uint8_t)(released!=0U);
    if(released!=b->candidate_released) {
        b->candidate_released=released;b->changed_tick=now;return 0U;
    }
    if(now-b->changed_tick<START_BUTTON_DEBOUNCE_MS)return 0U;
    if(released) {
        b->stable_released=1U;b->armed=1U;return 0U;
    }
    if(!b->stable_released)return 0U;
    b->stable_released=0U;
    if(!b->armed)return 0U;
    b->armed=0U;*pressed_tick=b->changed_tick;return 1U;
}
/* Return1/2 only on stable release; preserve the original press edge for READY freshness. */
static inline uint8_t StartButton_GestureUpdate(start_button_t *b,uint32_t now,uint32_t *pressed_tick,uint32_t *duration)
{
    uint8_t was_released=b->stable_released;uint32_t edge;
    if(StartButton_Update(b,b->candidate_released,now,&edge)) {
        b->pressed_at=edge;b->gesture_pending=1U;return 0U;
    }
    if(!was_released && b->stable_released && b->gesture_pending) {
        b->gesture_pending=0U;*pressed_tick=b->pressed_at;*duration=b->changed_tick-b->pressed_at;
        return *duration>=START_BUTTON_LONG_PRESS_MS?2U:1U;
    }
    return 0U;
}
#endif
