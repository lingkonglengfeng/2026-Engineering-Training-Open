#ifndef NAV_FEEDBACK_H
#define NAV_FEEDBACK_H
#include <stdint.h>

typedef enum {
    NAV_MOTION_OK, NAV_MOTION_RESIDUAL, NAV_MOTION_NO_MOVEMENT,
    NAV_MOTION_REVERSED, NAV_MOTION_INVALID
} nav_motion_feedback_t;

/* RESIDUAL is telemetry for manual tuning, not a stop or a correction request. */
float NavFeedback_SettleCounts(float counts_per_rev, float mm_per_rev);
/* Signed progress along the requested wheel pattern. Median rejects one
 * encoder outlier; opposite motion remains negative, never use abs(). */
float NavFeedback_ProjectWheels(const double wheel_rev[4],const float expected[4],
                                uint8_t mask,float *spread);
nav_motion_feedback_t NavFeedback_Check(float planned_mm, float measured_mm);
/* Only after live odometry stopped/settled and all four end replies arrived.
 * Finite/range/direction checks remain; final distance residual is accepted. */
nav_motion_feedback_t NavFeedback_CheckOdometry(float planned_mm,float measured_mm);
const char *NavFeedback_Reason(nav_motion_feedback_t result);
#endif
