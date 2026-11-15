#ifndef NAV_FRAME_H
#define NAV_FRAME_H
#include <stdint.h>

/* Map: x right, y up. Heading: CCW positive from the initial forward vector.
 * Body: forward positive, RIGHT positive (matches chassis_motor). */
void NavFrame_MapToBody(float initial_x, float initial_y, float heading_deg,
                        float map_dx, float map_dy, float *forward, float *right);
void NavFrame_BodyToMap(float initial_x,float initial_y,float heading_deg,
                        float forward,float right,float *map_x,float *map_y);
int8_t NavFrame_StrafeSide(float initial_x, float initial_y, float heading_deg,
                           float map_dx, float map_dy, float tolerance_deg);
uint8_t NavFrame_ReverseAligned(float initial_x,float initial_y,float heading_deg,
                               float map_dx,float map_dy,float tolerance_deg);
/* Wheel projection order: FL, FR, RL, RR (ZDT mileage query order). */
void NavFrame_WheelValues(float forward, float right, float values[4]);
/* Expected command direction for timed translation, including deliberate BACK. */
void NavFrame_VelocityWheelValues(int8_t forward_sign,int8_t strafe_sign,float values[4]);
#endif
