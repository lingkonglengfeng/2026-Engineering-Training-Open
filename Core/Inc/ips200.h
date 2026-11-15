#ifndef IPS200_H
#define IPS200_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

typedef enum
{
    IPS200_ROTATION_PORTRAIT = 0U,
    IPS200_ROTATION_PORTRAIT_180,
    IPS200_ROTATION_LANDSCAPE,
    IPS200_ROTATION_LANDSCAPE_180
} ips200_rotation_t;

#define IPS200_COLOR_BLACK       0x0000U
#define IPS200_COLOR_BLUE        0x001FU
#define IPS200_COLOR_RED         0xF800U
#define IPS200_COLOR_GREEN       0x07E0U
#define IPS200_COLOR_CYAN        0x07FFU
#define IPS200_COLOR_MAGENTA     0xF81FU
#define IPS200_COLOR_YELLOW      0xFFE0U
#define IPS200_COLOR_WHITE       0xFFFFU
#define IPS200_COLOR_GRAY        0x8410U

HAL_StatusTypeDef ips200_init(ips200_rotation_t rotation);
HAL_StatusTypeDef ips200_set_rotation(ips200_rotation_t rotation);
HAL_StatusTypeDef ips200_fill_screen(uint16_t color);
HAL_StatusTypeDef ips200_fill_rect(uint16_t x_start, uint16_t y_start,
                                   uint16_t x_end, uint16_t y_end,
                                   uint16_t color);
HAL_StatusTypeDef ips200_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
HAL_StatusTypeDef ips200_draw_bitmap_rgb565(uint16_t x, uint16_t y,
                                             uint16_t width, uint16_t height,
                                             const uint16_t *pixels);
HAL_StatusTypeDef ips200_draw_mono_bitmap(uint16_t x, uint16_t y,
                                          uint16_t width, uint16_t height,
                                          const uint8_t *bits,
                                          uint16_t foreground,
                                          uint16_t background);
HAL_StatusTypeDef ips200_draw_mono_bitmap_scaled(uint16_t x, uint16_t y,
                                                 uint16_t dest_width,
                                                 uint16_t dest_height,
                                                 const uint8_t *bits,
                                                 uint16_t source_width,
                                                 uint16_t source_height,
                                                 uint16_t foreground,
                                                 uint16_t background);
HAL_StatusTypeDef ips200_draw_text(uint16_t x, uint16_t y, const char *text,
                                   uint16_t foreground, uint16_t background);
HAL_StatusTypeDef ips200_draw_test_pattern(void);

uint16_t ips200_get_width(void);
uint16_t ips200_get_height(void);

#endif /* IPS200_H */
