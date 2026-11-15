#ifndef SCREEN_PROTOCOL_H
#define SCREEN_PROTOCOL_H
#include <stdint.h>
#define SCREEN_MAX_OBSTACLES 19U
#define SCREEN_LINE_CAPACITY 384U
#define SCREEN_RX_TIMEOUT_MS 1000U
typedef struct {uint16_t x,y;} screen_point_mm_t;
typedef struct {uint8_t count;screen_point_mm_t points[SCREEN_MAX_OBSTACLES];} screen_batch_t;
typedef struct {
    char line[SCREEN_LINE_CAPACITY];uint16_t length;
    uint8_t dropping,binary[16],binary_length,ff_count,page;
    uint32_t last_byte;
} screen_parser_t;
typedef enum {SCREEN_EVENT_NONE,SCREEN_EVENT_BATCH,SCREEN_EVENT_PAGE,SCREEN_EVENT_REJECT} screen_event_t;
uint8_t Screen_ParseBatch(const char *text,screen_batch_t *batch);
void Screen_ParserInit(screen_parser_t *p);
void Screen_ParserDrop(screen_parser_t *p);
screen_event_t Screen_ParserFeed(screen_parser_t *p,uint8_t byte,uint32_t now,screen_batch_t *batch);
#endif
