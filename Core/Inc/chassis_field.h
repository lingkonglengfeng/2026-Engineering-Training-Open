#ifndef CHASSIS_FIELD_H
#define CHASSIS_FIELD_H
#include <stdint.h>
typedef struct {
    float x, y, omega, wheel[4];
} chassis_field_profile_t;
/* Fractional field velocities; no motor signs. One shared interpolation keeps
 * field translation and rotation coherent through wheel direction reversals.
 * rpm_max is the velocity reference; rpm_limit is the actual wheel ceiling. */
void ChassisField_Step(chassis_field_profile_t *p,float tx,float ty,float to,
                       float fx,float fy,float yaw,float dt,float rpm_max,
                       float accel,float decel,float turn_decel,float rpm_limit);
#endif
