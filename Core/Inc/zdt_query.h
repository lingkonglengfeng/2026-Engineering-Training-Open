#ifndef ZDT_QUERY_H
#define ZDT_QUERY_H
#include <stdint.h>
#define ZDT_QUERY_POSITION 1U
#define ZDT_QUERY_PULSES 2U
typedef struct zdt_query zdt_query_t;
typedef struct {
    uint8_t (*send)(uint8_t address,uint8_t command);
    void (*report)(const zdt_query_t *query,const char *reason);
} zdt_query_ports_t;
struct zdt_query {
    zdt_query_ports_t ports;
    int64_t position_counts,input_pulses;
    uint32_t started,baseline_seq,position_tick,pulse_tick;
    uint8_t running,address,mask;
};
uint8_t ZDTQuery_Start(zdt_query_t *query,const zdt_query_ports_t *ports,uint8_t address,
                       uint32_t now,uint32_t baseline_seq);
/* Owner task only; received tick/sequence come from the one existing CAN ISR.
 * Readback sign+magnitude32 is kept in int64, not wrapped into int32. */
void ZDTQuery_Frame(zdt_query_t *query,uint8_t address,const uint8_t *data,uint8_t length,
                    uint32_t received_tick,uint32_t received_seq);
void ZDTQuery_Service(zdt_query_t *query,uint32_t now);
double ZDTQuery_AngleDeg(const zdt_query_t *query);
double ZDTQuery_CircleDeg(const zdt_query_t *query);
#endif
