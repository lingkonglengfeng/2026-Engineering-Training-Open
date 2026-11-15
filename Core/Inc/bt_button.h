/**
  ******************************************************************************
  * @file    bt_button.h
  * @brief   Bluetooth command parser for UART4.
  *
  * Commands:
  *   &EN, &M1, &M2, &U, &L, &D, &R, &STOP
  *   &R90, &L90 ... relative heading turn in degrees
  *
  * For turn commands, right is positive and left is negative.
  ******************************************************************************
  */

#ifndef __BT_BUTTON_H__
#define __BT_BUTTON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  BT_BUTTON_NONE = 0,
  BT_BUTTON_ENABLE,
  BT_BUTTON_M1,
  BT_BUTTON_M2,
  BT_BUTTON_FORWARD,
  BT_BUTTON_LEFT,
  BT_BUTTON_BACKWARD,
  BT_BUTTON_RIGHT,
  BT_BUTTON_STOP,
  BT_BUTTON_ROTATE_LEFT,
  BT_BUTTON_ROTATE_RIGHT,
  BT_BUTTON_CUSTOM
} bt_button_id_t;

typedef struct
{
  bt_button_id_t id;
  int16_t angle_deg;
  /* 63 characters support three precise pose angles without truncation. */
  char text[64];
  uint32_t sequence;
  uint32_t tick_ms;
} bt_button_event_t;

typedef struct
{
  uint32_t valid_count;
  uint32_t invalid_count;
  uint32_t dropped_count;
} bt_button_stats_t;

void BT_Button_Init(void);
void BT_Button_Poll(void);
uint8_t BT_Button_ProcessByte(uint8_t byte);
uint8_t BT_Button_ProcessCommand(const char *command);
uint8_t BT_Button_Pop(bt_button_event_t *event);
const bt_button_stats_t *BT_Button_GetStats(void);
const char *BT_Button_GetName(bt_button_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BT_BUTTON_H__ */
