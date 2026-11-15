#include "vision_align.h"
#include "station_map.h"
#include <math.h>
#include <string.h>

void VisionAlign_DefaultConfig(vision_align_config_t *c)
{
    memset(c,0,sizeof(*c));
    c->axes=3U;c->tolerance_px=5.0f;c->max_speed=0.035f;c->min_speed=0.015f;
    c->pulse_ms=100U;c->settle_ms=250U;c->stable_ms=200U;c->stable_frames=5U;
    c->loss_ms=3000U;c->timeout_ms=30000U;c->max_drive_ms=2500U;
    c->pid_p=1.0f;c->integral_limit_px=10.0f;c->derivative_filter_ms=100U;c->search_limit_ms=3000U;
    c->observation_ms=VISION_PIXEL_TIMEOUT_MS;c->fade_start_ms=120U;c->fade_zero_ms=160U;
    c->search_speed=0.015f;
}
uint8_t VisionAlign_StationConfig(vision_align_config_t *c,uint8_t station)
{
    if(!c || station<1U || station>3U)return 0U;
    VisionAlign_DefaultConfig(c);c->camera=1U;c->station=station;
    c->type=station==1U?VISION_TYPE_PLATE_PX:VISION_TYPE_RING_PX;
    c->id=station==1U?0U:2U;c->mode=V_ALIGN_MODE_SMOOTH;c->search_axis=V_ALIGN_SEARCH_IMAGE_RIGHT;
    c->zero_ring_left=(uint8_t)(station==2U);
    c->tolerance_px=station==1U?V_ALIGN_PLATE_TOLERANCE_PX:V_ALIGN_RING_TOLERANCE_PX;
    if(station==1U)c->stable_frames=3U;
    /* V257: increase only XY correction cap to110mm/s.
     * Search, minimum speed, pixel gains and yaw retain their settings. */
    c->max_speed=0.10324832f; /* 110mm/s at the250RPM baseline. */
    c->min_speed=0.030f;
    c->search_speed=station==1U?0.030f:0.040f; /* V148 PLATE slow image-left search. */
    c->forward_x=c->right_y=0.0014f;
    c->observation_ms=400U;c->fade_start_ms=260U;c->fade_zero_ms=360U;
    c->search_limit_ms=station==1U?4000U:8000U;
    c->loss_ms=20000U;c->timeout_ms=90000U;c->max_drive_ms=V_ALIGN_STATION_DRIVE_LIMIT_MS;
    return 1U;
}
uint8_t VisionAlign_ConfigValid(const vision_align_config_t *c)
{
    float determinant=c->forward_x*c->right_y-c->forward_y*c->right_x;
    if(c->camera<1U || c->camera>2U ||
       (c->type!=VISION_TYPE_MAT_PX && c->type!=VISION_TYPE_RING_PX && c->type!=VISION_TYPE_PLATE_PX) ||
       c->id>6U || (c->type==VISION_TYPE_PLATE_PX && c->id!=0U) ||
       c->axes<1U || c->axes>3U || !isfinite(c->forward_x) || !isfinite(c->forward_y) ||
       !isfinite(c->right_x) || !isfinite(c->right_y) ||
       fabsf(c->forward_x)>0.02f || fabsf(c->forward_y)>0.02f ||
       fabsf(c->right_x)>0.02f || fabsf(c->right_y)>0.02f ||
       !(c->tolerance_px>=0.5f && c->tolerance_px<=30.0f) ||
       !(c->max_speed>=0.01f && c->max_speed<=0.20f) ||
       !(c->min_speed>0.0f && c->min_speed<=c->max_speed) ||
       c->pulse_ms<20U || c->pulse_ms>300U || c->settle_ms<100U ||
       c->stable_ms<100U || c->stable_frames<3U || c->loss_ms<500U ||
       c->timeout_ms<c->loss_ms || c->max_drive_ms<c->pulse_ms)return 0U;
    if(c->mode>V_ALIGN_MODE_SMOOTH || c->search_axis>V_ALIGN_SEARCH_IMAGE_RIGHT || c->plate_search_right>1U ||
       !isfinite(c->pid_p) || c->pid_p<=0.0f || c->pid_p>4.0f ||
       !isfinite(c->pid_i) || c->pid_i<0.0f || c->pid_i>1.0f ||
       !isfinite(c->pid_d) || c->pid_d<0.0f || c->pid_d>0.5f ||
       !isfinite(c->integral_limit_px) || c->integral_limit_px<0.0f || c->integral_limit_px>50.0f ||
       c->derivative_filter_ms<10U || c->derivative_filter_ms>500U ||
       c->search_limit_ms<100U || c->search_limit_ms>10000U)return 0U;
    if(c->observation_ms<VISION_PIXEL_TIMEOUT_MS || c->observation_ms>500U ||
       c->fade_start_ms>=c->fade_zero_ms || c->fade_zero_ms>c->observation_ms ||
       !isfinite(c->search_speed) || c->search_speed<=0.0f || c->search_speed>c->max_speed)return 0U;
    if(c->search_axis && ((c->type!=VISION_TYPE_RING_PX && c->type!=VISION_TYPE_PLATE_PX) ||
       c->station<1U || c->station>3U))return 0U;
    if((c->search_axis==V_ALIGN_SEARCH_IMAGE_RIGHT || (c->search_axis && c->type==VISION_TYPE_PLATE_PX)) &&
       fabsf(c->forward_x)+fabsf(c->right_x)<1e-6f)return 0U;
    if(c->zero_ring_left>1U || (c->zero_ring_left && (c->type!=VISION_TYPE_RING_PX || c->station!=2U)))return 0U;
    if(c->axes==3U && fabsf(determinant)<1e-10f)return 0U;
    if((c->axes&1U) && fabsf(c->forward_x)+fabsf(c->right_x)<1e-6f)return 0U;
    if((c->axes&2U) && fabsf(c->forward_y)+fabsf(c->right_y)<1e-6f)return 0U;
    return 1U;
}
uint8_t VisionAlign_Start(vision_align_t *a,const vision_align_config_t *c,
                         uint32_t now,uint32_t previous)
{
    if(!VisionAlign_ConfigValid(c))return 0U;
    memset(a,0,sizeof(*a));a->config=*c;a->state=V_ALIGN_SETTLE;
    a->started=now;a->phase_tick=now;a->last_valid=now;a->last_observation=previous;
    a->target_id=c->id;a->target_locked=(uint8_t)(c->id!=0U || c->type==VISION_TYPE_PLATE_PX);
    return 1U;
}
void VisionAlign_Cancel(vision_align_t *a)
{ a->state=V_ALIGN_IDLE;a->forward=0.0f;a->right=0.0f; }
static void fail(vision_align_t *a,const char *reason)
{ a->state=V_ALIGN_FAILED;a->reason=reason;a->forward=0.0f;a->right=0.0f; }
static uint8_t usable(const vision_align_t *a,const vision_pixel_t *p,uint32_t age)
{
    return (uint8_t)(p!=NULL && p->valid && p->observation_id!=0U && age<a->config.observation_ms &&
        p->type==a->config.type && p->id<=6U &&
        (p->type==VISION_TYPE_PLATE_PX?p->id==0U:p->id!=0U) &&
        (!a->target_locked || p->id==a->target_id) && isfinite(p->dx_px) && isfinite(p->dy_px));
}
static void reset_pid(vision_align_t *a)
{ a->pid_ready=0U;a->integral_x=a->integral_y=a->derivative_x=a->derivative_y=0.0f; }
static void account(vision_align_t *a,uint32_t now)
{
    if(a->state==V_ALIGN_TRACK || a->state==V_ALIGN_SEARCH) {
        uint32_t dt=now-a->control_tick;
        if(a->driving)a->drive_ms+=dt;
        if(a->searching)a->search_ms+=dt;
        a->control_tick=now;
    }
}
static void wait_frame(vision_align_t *a,uint32_t now)
{
    if(a->state==V_ALIGN_PULSE)a->drive_ms+=now-a->phase_tick;
    a->forward=a->right=0.0f;a->driving=a->searching=0U;a->stable_count=0U;
    a->minimum_frame_tick=now;a->state=V_ALIGN_WAIT;reset_pid(a);
}
static uint8_t stable_gap_wait(const vision_align_t *a,uint32_t now,
                              const vision_pixel_t *p,uint32_t age,uint8_t stopped)
{
    return (uint8_t)(a->state==V_ALIGN_WAIT && stopped && a->stable_count && p &&
        p->observation_id==a->last_observation && p->type==a->config.type && p->id==a->target_id &&
        age>=a->config.observation_ms && now-a->stable_receive_tick<=V_ALIGN_STABLE_FRAME_GAP_MS);
}
static float fade(const vision_align_config_t *c,uint32_t age)
{
    if(age>=c->fade_zero_ms)return 0.0f;
    return age<=c->fade_start_ms?1.0f:
        (c->fade_zero_ms-age)/(float)(c->fade_zero_ms-c->fade_start_ms);
}
static float bound(float v,float limit){return fmaxf(-limit,fminf(limit,v));}
static uint8_t y_in_band(const vision_align_config_t *c,float y)
{
    /* Wire image Y is down-positive. PLATE accepts the upper half-plane;
     * below the existing aim line it must reach/cross zero, not the21px band. */
    return c->type==VISION_TYPE_PLATE_PX?y<=0.0f:fabsf(y)<=c->tolerance_px;
}
static uint8_t in_band(const vision_align_config_t *c,float x,float y)
{return fabsf(x)<=c->tolerance_px && y_in_band(c,y);}
static void smooth_command(vision_align_t *a,float x,float y,uint32_t received,uint32_t age)
{
    const vision_align_config_t *c=&a->config;float ix=a->integral_x,iy=a->integral_y,px,py,maximum;
    if(c->type==VISION_TYPE_PLATE_PX && y<0.0f)y=0.0f;
    if(a->pid_ready) {
        uint32_t ms=received-a->pid_tick;
        if(ms && ms<c->observation_ms) {
            float dt=ms*0.001f,alpha=ms/(float)(c->derivative_filter_ms+ms);
            a->derivative_x+=alpha*((x-a->previous_x)/dt-a->derivative_x);
            a->derivative_y+=alpha*((y-a->previous_y)/dt-a->derivative_y);
            ix=bound(ix+c->pid_i*x*dt,c->integral_limit_px);
            iy=bound(iy+c->pid_i*y*dt,c->integral_limit_px);
        } else if(ms>=c->observation_ms){reset_pid(a);ix=iy=0.0f;}
    }
    a->pid_tick=received;a->previous_x=x;a->previous_y=y;a->pid_ready=1U;
    if(fabsf(x)<=c->tolerance_px){x=0.0f;ix=a->integral_x=a->derivative_x=0.0f;}
    if(y_in_band(c,y)){y=0.0f;iy=a->integral_y=a->derivative_y=0.0f;}
    px=c->pid_p*x+ix+c->pid_d*a->derivative_x;
    py=c->pid_p*y+iy+c->pid_d*a->derivative_y;
    a->forward=c->forward_x*px+c->forward_y*py;
    a->right=c->right_x*px+c->right_y*py;
    maximum=fabsf(a->forward)+fabsf(a->right);
    /* Conditional integration: do not accumulate against clipping or fading
     * old measurements. I/D only update on distinct camera observations. */
    if(maximum<=c->max_speed && age<=c->fade_start_ms) {
        a->integral_x=ix;a->integral_y=iy;
    } else {
        px=c->pid_p*x+a->integral_x+c->pid_d*a->derivative_x;
        py=c->pid_p*y+a->integral_y+c->pid_d*a->derivative_y;
        a->forward=c->forward_x*px+c->forward_y*py;
        a->right=c->right_x*px+c->right_y*py;
        maximum=fabsf(a->forward)+fabsf(a->right);
    }
    if(maximum>c->max_speed){a->forward*=c->max_speed/maximum;a->right*=c->max_speed/maximum;}
    /* PLATE only: fresh out-of-dead-zone commands must overcome static
     * friction. Keep the calibrated vector; fading still applies afterwards. */
    maximum=fabsf(a->forward)+fabsf(a->right);
    if(c->type==VISION_TYPE_PLATE_PX && maximum>1e-8f && maximum<c->min_speed) {
        a->forward*=c->min_speed/maximum;a->right*=c->min_speed/maximum;
    }
}
static void search_command(vision_align_t *a,uint32_t now,uint32_t age,uint8_t body_left,float *f,float *r)
{
    float vx=0.0f,vy=1.0f,norm,weight=fade(&a->config,age),speed=a->config.search_speed;
    if(a->state!=V_ALIGN_SEARCH){wait_frame(a,now);a->state=V_ALIGN_SEARCH;a->control_tick=now;}
    if(a->search_ms>=a->config.search_limit_ms) {
        fail(a,a->config.type==VISION_TYPE_PLATE_PX?"plate search budget reached":"third ring search budget reached");return;
    }
    if(a->drive_ms>=a->config.max_drive_ms){fail(a,"visual correction motion budget reached");return;}
    /* Follow the selected nominal approach in CAMERA coordinates. Keep the
     * signed calibration; do not use a transient obstacle-detour velocity. */
    if(a->config.type==VISION_TYPE_PLATE_PX){
        float sign=a->config.plate_search_right?1.0f:-1.0f;
        vx=sign*a->config.forward_x;vy=sign*a->config.right_x;
    }
    else if(body_left){vx=0.0f;vy=-1.0f;speed=fminf(speed*V_ALIGN_EMPTY_LEFT_SPEED_MULTIPLIER,a->config.max_speed);}
    else if(a->config.search_axis==V_ALIGN_SEARCH_IMAGE_RIGHT){vx=a->config.forward_x;vy=a->config.right_x;}
    norm=fabsf(vx)+fabsf(vy);
    if(norm<1e-6f){fail(a,"visual search direction not calibrated");return;}
    a->forward=vx/norm*speed;a->right=vy/norm*speed;
    *f=a->forward*weight;*r=a->right*weight;a->driving=a->searching=(uint8_t)(weight>0.0f);
}
static uint8_t finish_search(vision_align_t *a,uint32_t now,const vision_pixel_t *p,uint32_t age,uint8_t valid)
{
    float x=(a->config.axes&1U)?p->dx_px:0.0f,y=(a->config.axes&2U)?p->dy_px:0.0f;
    wait_frame(a,now);
    if(!valid){a->last_observation=p->observation_id;return 1U;}
    if(in_band(&a->config,x,y)) {
        a->error_x=x;a->error_y=y;a->last_observation=p->observation_id;
        a->state=V_ALIGN_SETTLE;a->settled=0U;a->phase_tick=now;return 1U;
    }
    /* Use this qualified target immediately. Existing chassis vector ramps
     * blend the search velocity into PID; do not throw away one camera frame. */
    a->minimum_frame_tick=now-age;return 0U;
}
void VisionAlign_StepTargets(vision_align_t *a,uint32_t now,const vision_pixel_t *p,
                           uint32_t age,const vision_ring_count_t *scene,uint32_t scene_age,
                           const vision_plate_scene_t *plate,uint32_t plate_age,uint8_t stopped,float *f,float *r)
{
    uint8_t valid;
    *f=0.0f;*r=0.0f;
    if(a->state==V_ALIGN_IDLE || a->state==V_ALIGN_DONE || a->state==V_ALIGN_FAILED)return;
    if(now-a->started>=a->config.timeout_ms){fail(a,"visual alignment timeout");return;}
    account(a,now);
    valid=usable(a,p,age);
    if(a->stable_count && (!stopped || now-a->stable_receive_tick>V_ALIGN_STABLE_FRAME_GAP_MS))
        a->stable_count=0U;
    if((a->state==V_ALIGN_TRACK || a->state==V_ALIGN_SEARCH) && a->drive_ms>=a->config.max_drive_ms) {
        fail(a,"visual correction motion budget reached");return;
    }
    if(a->state==V_ALIGN_SETTLE) {
        if(!stopped){a->settled=0U;a->phase_tick=now;return;}
        if(!a->settled){a->settled=1U;a->phase_tick=now;}
        if(now-a->phase_tick<a->config.settle_ms)return;
        a->minimum_frame_tick=now;a->last_valid=now;a->state=V_ALIGN_WAIT;
    }
    if(a->config.search_axis && a->config.type==VISION_TYPE_PLATE_PX) {
        if(valid) {
            a->plate_seen=1U;a->plate_missing_frames=0U;
            if(a->state==V_ALIGN_SEARCH && finish_search(a,now,p,age,1U))return;
        } else if(plate && plate->valid && plate->status==VISION_PLATE_SEARCH && plate->id==0U &&
            plate->observation_id && plate->station==a->config.station && plate_age<a->config.observation_ms &&
            p && plate->seq==p->seq && p->observation_id && !p->valid && p->type==VISION_TYPE_PLATE_PX && p->id==0U &&
            p->dx_px==0.0f && p->dy_px==0.0f && age<a->config.observation_ms &&
            (a->state==V_ALIGN_SEARCH || (int32_t)((now-age)-a->minimum_frame_tick)>=0)) {
            if(p->observation_id!=a->last_scene) {
                a->last_scene=p->observation_id;a->last_valid=now;
                if(a->plate_missing_frames<V_ALIGN_PLATE_SEARCH_FRAMES)a->plate_missing_frames++;
            }
            if(a->plate_missing_frames>=V_ALIGN_PLATE_SEARCH_FRAMES)search_command(a,now,age,0U,f,r);
            else if(a->state!=V_ALIGN_WAIT)wait_frame(a,now); /* Stop tracking during missing-frame confirmation. */
            return;
        } else {
            a->plate_missing_frames=0U;
            if(a->state==V_ALIGN_SEARCH)wait_frame(a,now);
        }
    }
    if(a->config.search_axis && a->config.type==VISION_TYPE_RING_PX) {
        uint8_t empty_left;
        uint8_t scene_current=(uint8_t)(scene && scene->valid && scene->observation_id &&
            scene_age<a->config.observation_ms && scene->station==a->config.station &&
            scene->id==2U);
        uint8_t pixel_current=(uint8_t)(p && p->observation_id && age<a->config.observation_ms &&
            p->type==VISION_TYPE_RING_PX && p->id==2U && scene && p->seq==scene->seq);
        uint8_t coherent=(uint8_t)(scene_current && pixel_current);
        empty_left=(uint8_t)(coherent && scene->count==0U && a->config.zero_ring_left && a->config.station==2U);

        /* TEMP's second ring is allowed to publish a trustworthy count before
         * its target-2 pixel has been acquired.  Keep searching to the camera
         * image-right on count==2; count==3 stops the search and falls through
         * to the normal target-pixel wait/correction path. */
        if(a->config.station==3U && scene_current && scene->count==2U) {
            if(scene->observation_id!=a->last_scene) {
                a->last_scene=scene->observation_id;a->last_valid=now;
            }
            search_command(a,now,scene_age,0U,f,r);return;
        }
        if(!coherent || (scene->count<2U && !empty_left)) {
            /* A missing next frame is not a failed measurement. Hold zero
             * speed during the stopped verification gap; never move on it. */
            if(scene && scene->count==3U && scene->station==a->config.station &&
                scene->id==2U && p && p->seq==scene->seq && scene_age>=a->config.observation_ms &&
                stable_gap_wait(a,now,p,age,stopped))return;
            wait_frame(a,now);
            if(now-a->last_valid>=a->config.loss_ms)fail(a,"ring scene missing/stale or fewer than two rings");
            return;
        }
        if(scene->count==2U || empty_left) {
            if(scene->observation_id!=a->last_scene){a->last_scene=scene->observation_id;a->last_valid=now;}
            search_command(a,now,scene_age,empty_left,f,r);return;
        }
        if(a->state==V_ALIGN_SEARCH) {
            if(finish_search(a,now,p,age,valid))return;
        }
    }
    if(a->state==V_ALIGN_TRACK) {
        if(!valid) {
            wait_frame(a,now);
            if(now-a->last_valid>=a->config.loss_ms)fail(a,"visual target missing/stale or wrong ID");
            return;
        }
        if(p->observation_id!=a->last_observation) {
            float x=(a->config.axes&1U)?p->dx_px:0.0f,y=(a->config.axes&2U)?p->dy_px:0.0f;
            a->last_observation=p->observation_id;a->last_valid=now;a->error_x=x;a->error_y=y;
            if(in_band(&a->config,x,y)) {
                a->forward=a->right=0.0f;a->driving=0U;reset_pid(a);
                a->phase_tick=now;a->settled=0U;a->state=V_ALIGN_SETTLE;return;
            }
            smooth_command(a,x,y,now-age,age);
        }
        *f=a->forward*fade(&a->config,age);*r=a->right*fade(&a->config,age);
        a->driving=(uint8_t)(*f!=0.0f || *r!=0.0f);return;
    }
    if(a->state==V_ALIGN_PULSE) {
        if(valid && now-a->phase_tick<a->config.pulse_ms) {
            *f=a->forward;*r=a->right;return;
        }
        a->drive_ms+=now-a->phase_tick;
        a->forward=0.0f;a->right=0.0f;a->stable_count=0U;
        a->phase_tick=now;a->settled=0U;a->state=V_ALIGN_SETTLE;
        /* An invalid frame ends the pulse immediately; the next frame after
         * stopping starts a new observation cycle, never resumes this pulse. */
        return;
    }
    if(!valid){
        if(!stable_gap_wait(a,now,p,age,stopped))a->stable_count=0U;
    }
    else if(p->observation_id!=a->last_observation &&
            (int32_t)((now-age)-a->minimum_frame_tick)>=0) {
        float x=(a->config.axes&1U)?p->dx_px:0.0f;
        float y=(a->config.axes&2U)?p->dy_px:0.0f;
        float maximum,scale;
        a->last_observation=p->observation_id;a->last_valid=now;
        if(!a->target_locked){a->target_id=p->id;a->target_locked=1U;}
        a->error_x=x;a->error_y=y;
        if(in_band(&a->config,x,y)) {
            if(!stopped){a->stable_count=0U;return;}
            if(!a->stable_count)a->stable_tick=now;
            a->stable_receive_tick=now-age;
            if(a->stable_count<255U)a->stable_count++;
            if(a->stable_count>=a->config.stable_frames && now-a->stable_tick>=a->config.stable_ms)
                a->state=V_ALIGN_DONE;
            return;
        }
        a->stable_count=0U;
        if(a->drive_ms>=a->config.max_drive_ms){fail(a,"visual correction motion budget reached");return;}
        if(a->config.mode==V_ALIGN_MODE_SMOOTH) {
            smooth_command(a,x,y,now-age,age);a->state=V_ALIGN_TRACK;a->control_tick=now;
            *f=a->forward*fade(&a->config,age);*r=a->right*fade(&a->config,age);
            a->driving=(uint8_t)(*f!=0.0f || *r!=0.0f);return;
        }
        if(fabsf(x)<=a->config.tolerance_px)x=0.0f;
        if(y_in_band(&a->config,y))y=0.0f;
        a->forward=a->config.forward_x*x+a->config.forward_y*y;
        a->right=a->config.right_x*x+a->config.right_y*y;
        /* L1 bound is the translational mecanum wheel peak; preserve direction. */
        maximum=fabsf(a->forward)+fabsf(a->right);
        if(maximum<1e-8f){fail(a,"visual mapping produced zero command");return;}
        scale=maximum>a->config.max_speed?a->config.max_speed/maximum:
              (maximum<a->config.min_speed?a->config.min_speed/maximum:1.0f);
        a->forward*=scale;a->right*=scale;a->phase_tick=now;a->state=V_ALIGN_PULSE;
        *f=a->forward;*r=a->right;return;
    }
    if(now-a->last_valid>=a->config.loss_ms)fail(a,"visual target missing/stale or wrong ID");
}
void VisionAlign_Step(vision_align_t *a,uint32_t now,const vision_pixel_t *p,
                      uint32_t age,uint8_t stopped,float *f,float *r)
{ VisionAlign_StepScene(a,now,p,age,NULL,UINT32_MAX,stopped,f,r); }
void VisionAlign_StepScene(vision_align_t *a,uint32_t now,const vision_pixel_t *p,
                           uint32_t age,const vision_ring_count_t *scene,uint32_t scene_age,
                           uint8_t stopped,float *f,float *r)
{VisionAlign_StepTargets(a,now,p,age,scene,scene_age,NULL,UINT32_MAX,stopped,f,r);}
uint8_t VisionAlign_StationPoseMm(uint8_t station,uint16_t *x,uint16_t *y)
{
    if(!x || !y || station<1U || station>3U)return 0U;
    return StationMap_TargetMm(station,x,y);
}
