#include "vision_align_diag.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef char record_size_bounded[(sizeof(vision_diag_record_t)<=48U)?1:-1];
void VisionDiag_Init(vision_align_diag_t *d)
{ memset(d,0,sizeof(*d));d->enabled=1U; }
void VisionDiag_Begin(vision_align_diag_t *d,uint8_t station,
                      const vision_align_config_t *config,uint32_t now)
{
    uint8_t enabled=d->enabled;
    if(!enabled)return;
    /* Old rows are unreachable after count=0; avoid clearing 22KB in a tick. */
    memset(&d->last,0,sizeof(*d)-offsetof(vision_align_diag_t,last));
    d->enabled=enabled;d->config=*config;d->station=station;
    d->started=d->last_tick=now;d->last_phase=V_DIAG_ACK;
    d->active=d->has_data=1U;
}
static int16_t scaled(float value,float factor)
{
    if(!isfinite(value))return 0;
    value*=factor;
    if(value>32767.0f)return 32767;
    if(value< -32768.0f)return -32768;
    return (int16_t)lroundf(value);
}
static void store(vision_align_diag_t *d,const vision_diag_record_t *row)
{
    if(d->count<V_ALIGN_DIAG_CAPACITY)d->rows[d->count++]=*row;
    else {
        /* Keep the start of the run and its latest sample, including fault
         * transitions. Replacing the previous tail drops one explicit row. */
        d->overflow++;d->rows[V_ALIGN_DIAG_CAPACITY-1U]=*row;
    }
}
void VisionDiag_Observe(vision_align_diag_t *d,const vision_diag_input_t *s)
{
    vision_diag_record_t row;const vision_pixel_t *p=s->pixel;
    uint32_t dt;uint8_t flags,change,new_frame;
    if(!d->active || s->phase>=V_DIAG_PHASE_COUNT)return;
    dt=s->now-d->last_tick;d->phase_ms[d->last_phase]+=dt;d->last_tick=s->now;
    if(dt>d->tick_max)d->tick_max=dt;
    new_frame=(uint8_t)(p && p->observation_id && p->observation_id!=d->last_observation);
    flags=(uint8_t)((p && p->valid?V_DIAG_VALID:0U)|
        (p && p->observation_id && s->age<d->config.observation_ms?V_DIAG_FRESH:0U)|
        (s->context?V_DIAG_CONTEXT:0U)|(s->stopped?V_DIAG_STOPPED:0U));
    if(new_frame) {
        uint32_t received=s->now-s->age;
        d->last_observation=p->observation_id;d->new_frames++;
        if(!(flags&V_DIAG_VALID))d->invalid_frames++;
        if(s->age!=UINT32_MAX) {
            if(d->have_receive) {
                uint32_t gap=received-d->last_receive;
                if(gap>d->gap_max)d->gap_max=gap;
                d->gaps[gap<=50U?0:(gap<=100U?1:(gap<=200U?2:(gap<=500U?3:4)))]++;
            }
            d->last_receive=received;d->have_receive=1U;
        }
    } else if(p && p->observation_id)d->same_observation_polls++;
    change=(uint8_t)(!d->have_sample || s->phase!=d->last.phase ||
        s->state!=d->last.state || s->stable!=d->last.stable ||
        s->scene_count!=d->last.scene_count || s->scene_valid!=d->last.scene_valid ||
        flags!=(d->last.flags&(V_DIAG_VALID|V_DIAG_FRESH|V_DIAG_CONTEXT|V_DIAG_STOPPED)));
    memset(&row,0,sizeof(row));
    row.forward=scaled(s->forward,10000.0f);row.right=scaled(s->right,10000.0f);
    row.omega=scaled(s->omega,10000.0f);
    if(row.forward!=d->last.forward || row.right!=d->last.right || row.omega!=d->last.omega)
        flags|=V_DIAG_COMMAND;
    if(s->phase==V_DIAG_PULSE && d->last_phase!=V_DIAG_PULSE)d->pulses++;
    d->last_phase=s->phase;
    /* No float formatting in motion. Record every new observation even in a
     * pulse; trace's observation counter never changes control state. */
    if(!new_frame && !change && (!(flags&V_DIAG_COMMAND) ||
        (s->now-d->started)-d->last.ms<20U))return;
    row.ms=s->now-d->started;row.observation=p?p->observation_id:0U;row.age=s->age;
    row.dx=p?p->dx_px:d->last.dx;row.dy=p?p->dy_px:d->last.dy;
    row.yaw=scaled(remainderf(s->yaw,360.0f),100.0f);row.gyro=scaled(s->gyro,100.0f);
    memcpy(row.rpm,s->rpm,sizeof(row.rpm));
    row.phase=s->phase;row.flags=(uint8_t)(flags|(new_frame?V_DIAG_NEW:0U)|
        (change?V_DIAG_TRANSITION:0U));
    row.type=p?p->type:0U;row.id=p?p->id:0U;row.state=s->state;row.stable=s->stable;
    row.scene_count=s->scene_count;row.scene_valid=s->scene_valid;
    d->last=row;d->have_sample=1U;store(d,&row);
}
void VisionDiag_Wheels(vision_align_diag_t *d,const int32_t position[4],
                       uint8_t mask,uint8_t end)
{
    if(!d->active)return;
    if(end){memcpy(d->wheel_end,position,sizeof(d->wheel_end));d->end_mask=mask;}
    else {memcpy(d->wheel_start,position,sizeof(d->wheel_start));d->start_mask=mask;}
}
void VisionDiag_Finish(vision_align_diag_t *d,uint32_t now,uint8_t success,
                       const char *reason)
{
    if(!d->active)return;
    d->phase_ms[d->last_phase]+=now-d->last_tick;
    d->elapsed=now-d->started;d->success=success;d->reason=reason;
    d->active=0U;
}
uint8_t VisionDiag_Line(const vision_align_diag_t *d,uint16_t index,
                        char *line,size_t capacity)
{
    const vision_align_config_t *c=&d->config;int written;
    if(!line || !capacity || !d->has_data || d->active)return 0U;
    if(index==0U)written=snprintf(line,capacity,
        "[VDIAG] BEGIN v=2 station=%u success=%u rows=%u capacity=%u dropped=%lu total_ms=%lu reason=%s\r\n",
        d->station,d->success,d->count,V_ALIGN_DIAG_CAPACITY,(unsigned long)d->overflow,
        (unsigned long)d->elapsed,d->reason?d->reason:"none");
    else if(index==1U)written=snprintf(line,capacity,
        "[VDIAG] config cam=%u type=%02X id=%u axes=%u K=%.5f,%.5f,%.5f,%.5f tol=%.2f min=%.4f max=%.4f\r\n",
        c->camera,c->type,c->id,c->axes,(double)c->forward_x,(double)c->forward_y,
        (double)c->right_x,(double)c->right_y,(double)c->tolerance_px,(double)c->min_speed,(double)c->max_speed);
    else if(index==2U)written=snprintf(line,capacity,
        "[VDIAG] config pulse=%u settle=%u stable_ms=%u stable_frames=%u loss=%u timeout=%lu drive_limit=%lu age_limit=%u mode=%u search=%u PID=%.3f,%.3f,%.3f\r\n",
        c->pulse_ms,c->settle_ms,c->stable_ms,c->stable_frames,c->loss_ms,
        (unsigned long)c->timeout_ms,(unsigned long)c->max_drive_ms,c->observation_ms,c->mode,c->search_axis,
        (double)c->pid_p,(double)c->pid_i,(double)c->pid_d);
    else if(index==3U)written=snprintf(line,capacity,
        "[VDIAG] ms ACK=%lu BASE=%lu PULSE=%lu BRAKE=%lu SETTLE=%lu WAIT=%lu END=%lu pulses=%lu\r\n",
        (unsigned long)d->phase_ms[0],(unsigned long)d->phase_ms[1],(unsigned long)d->phase_ms[2],
        (unsigned long)d->phase_ms[3],(unsigned long)d->phase_ms[4],(unsigned long)d->phase_ms[5],
        (unsigned long)d->phase_ms[6],(unsigned long)d->pulses);
    else if(index==4U)written=snprintf(line,capacity,
        "[VDIAG] frames=%lu invalid_new=%lu same_obs_polls=%lu rx_gap_max=%lu tick_max=%lu can_enqueue_fail=%lu gap_bins=50:%lu,100:%lu,200:%lu,500:%lu,over:%lu\r\n",
        (unsigned long)d->new_frames,(unsigned long)d->invalid_frames,(unsigned long)d->same_observation_polls,
        (unsigned long)d->gap_max,(unsigned long)d->tick_max,(unsigned long)d->can_failures,
        (unsigned long)d->gaps[0],(unsigned long)d->gaps[1],(unsigned long)d->gaps[2],
        (unsigned long)d->gaps[3],(unsigned long)d->gaps[4]);
    else if(index==5U || index==6U) {
        const int32_t *w=index==5U?d->wheel_start:d->wheel_end;
        written=snprintf(line,capacity,"[VDIAG] wheel_%s mask=%u counts=%ld,%ld,%ld,%ld\r\n",
            index==5U?"start":"end",index==5U?d->start_mask:d->end_mask,
            (long)w[0],(long)w[1],(long)w[2],(long)w[3]);
    } else if(index==7U)written=snprintf(line,capacity,
        "VDR,t_ms,obs_id,rx_age_ms,dx_px,dy_px,F_1e4,R_1e4,O_1e4,yaw_cdeg,gyro_cdeg_s,FL_cmd_rpm,FR_cmd_rpm,RL_cmd_rpm,RR_cmd_rpm,phase,flags,type,id,state,stable,ring_count,scene_valid\r\n");
    else if(index<(uint16_t)(8U+d->count)) {
        const vision_diag_record_t *r=&d->rows[index-8U];
        written=snprintf(line,capacity,
            "VDR,%lu,%lu,%lu,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
            (unsigned long)r->ms,(unsigned long)r->observation,(unsigned long)r->age,
            (double)r->dx,(double)r->dy,r->forward,r->right,r->omega,r->yaw,r->gyro,
            r->rpm[0],r->rpm[1],r->rpm[2],r->rpm[3],r->phase,r->flags,r->type,r->id,r->state,r->stable,
            r->scene_count,r->scene_valid);
    } else if(index==(uint16_t)(8U+d->count))written=snprintf(line,capacity,
        "[VDIAG] END station=%u rows=%u dropped=%lu last_dx=%.2f last_dy=%.2f\r\n",
        d->station,d->count,(unsigned long)d->overflow,(double)d->last.dx,(double)d->last.dy);
    else return 0U;
    return (uint8_t)(written>0 && (size_t)written<capacity);
}
