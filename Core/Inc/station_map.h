#ifndef STATION_MAP_H
#define STATION_MAP_H
#include <stdint.h>
/* Surveyed car-centre approach and visual anchor coordinates, mm, Y up.
 * User-provided adjustment range: each axis within nominal +/-300mm, and the
 * resulting car-centre/anchor points must stay on the 2400mm field. */
enum { STATION_MAP_QR,STATION_MAP_RAW,STATION_MAP_ROUGH,STATION_MAP_TEMP,STATION_MAP_COUNT };
#define STATION_MAP_LIMIT_MM 300
/* V278: ROUGH car-centre references move20mm toward map bottom. */
#define STATION_MAP_ROUGH_APPROACH_Y_MM 380U
#define STATION_MAP_ROUGH_ANCHOR_Y_MM STATION_MAP_ROUGH_APPROACH_Y_MM
#define STATION_MAP_ROUGH_NAMED_Y_MM 330U
uint16_t StationMap_X(uint8_t station,uint16_t nominal);
uint16_t StationMap_Y(uint8_t station,uint16_t nominal);
/* Canonical car-centre target, also used for successful visual rebase. */
uint8_t StationMap_TargetMm(uint8_t station,uint16_t *x,uint16_t *y);
void StationMap_PrintStatus(void); /* Read-only map/manual-obstacle snapshot. */
/* Commands run only through the idle navigation owner. No motion is sent. */
uint8_t StationMap_ProcessCommand(const char *command);
/* 1=loaded, 0=no usable record/read failure, 2=SPI storage busy. */
uint8_t StationMap_LoadFromFlash(void);
#endif
