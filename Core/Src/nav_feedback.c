#include "nav_feedback.h"
#include "nav_odometry.h"
#include <math.h>

#define NAV_FEEDBACK_ARRIVAL_MM     (20.0f)
#define NAV_FEEDBACK_MIN_PROGRESS  (0.25f)
#define NAV_FEEDBACK_MAX_DISTANCE  (10000.0f)

float NavFeedback_SettleCounts(float counts_per_rev, float mm_per_rev)
{
    return fmaxf(1.0f, counts_per_rev * 0.5f / mm_per_rev);
}

float NavFeedback_ProjectWheels(const double wheel_rev[4],const float expected[4],
                                uint8_t mask,float *spread)
{
    double sorted[4];uint8_t count=0U,i,j;
    if(!wheel_rev || !expected || !spread)return NAN;
    *spread=0.0f;
    for(i=0U;i<4U;i++) {
        if(!(mask&(1U<<i)))continue;
        if(!isfinite(wheel_rev[i]) || !isfinite(expected[i]))return NAN;
        if(fabsf(expected[i])<=0.1f)continue;
        sorted[count++]=wheel_rev[i]/(double)expected[i];
    }
    if(!count)return 0.0f;
    for(i=1U;i<count;i++) {
        double key=sorted[i];j=i;
        while(j && sorted[j-1U]>key){sorted[j]=sorted[j-1U];j--;}
        sorted[j]=key;
    }
    *spread=(float)(sorted[count-1U]-sorted[0]);
    return (float)((count&1U)?sorted[count/2U]:
        (sorted[count/2U-1U]+sorted[count/2U])*0.5);
}

nav_motion_feedback_t NavFeedback_Check(float planned_mm, float measured_mm)
{
    float residual;
    if (!(planned_mm > 0.0f) || planned_mm > NAV_FEEDBACK_MAX_DISTANCE ||
        measured_mm != measured_mm || fabsf(measured_mm) > NAV_FEEDBACK_MAX_DISTANCE) {
        return NAV_MOTION_INVALID;
    }
    if (measured_mm < -5.0f) { return NAV_MOTION_REVERSED; }
    if (measured_mm < planned_mm * NAV_FEEDBACK_MIN_PROGRESS) {
        return NAV_MOTION_NO_MOVEMENT;
    }
    residual = fabsf(planned_mm - measured_mm);
    if (residual <= NAV_FEEDBACK_ARRIVAL_MM) { return NAV_MOTION_OK; }
    /* Manual serial flow: operator checks docking at WAIT_OK. Distance error
     * stays visible, but never schedules slow pulses or trips a precision gate. */
    return NAV_MOTION_RESIDUAL;
}

nav_motion_feedback_t NavFeedback_CheckOdometry(float planned,float measured)
{
    if(!isfinite(planned) || !isfinite(measured) || planned<=0.0f ||
       planned>NAV_FEEDBACK_MAX_DISTANCE || fabsf(measured)>NAV_FEEDBACK_MAX_DISTANCE)return NAV_MOTION_INVALID;
    if(measured< -5.0f)return NAV_MOTION_REVERSED;
    /* Live odometry already owns progress, braking and fresh-wheel checks.
     * Final distance error never rejects an otherwise valid stopped segment. */
    return NAV_MOTION_OK;
}
const char *NavFeedback_Reason(nav_motion_feedback_t result)
{
    switch (result) {
        case NAV_MOTION_NO_MOVEMENT: return "wheel movement too small";
        case NAV_MOTION_REVERSED: return "wheel feedback indicates reverse motion";
        case NAV_MOTION_RESIDUAL: return "distance residual; tune dcomp";
        case NAV_MOTION_INVALID: return "invalid motion feedback";
        default: return "motion feedback accepted";
    }
}
