#include "ips200.h"

#include "cmsis_os2.h"
#include "main.h"
#include "spi.h"

#define IPS200_SPI_TIMEOUT_MS       100U
#define IPS200_COLOR_BUFFER_PIXELS  64U

static uint16_t ips200_width = 240U;
static uint16_t ips200_height = 320U;

/* Watch diagnostics: stage 1/2 reset delays, 3 sleep-out, 4 delay,
 * 5 register sequence, 6 white fill, 7 initialized. First error is sticky. */
volatile uint32_t g_lcd_stage;
volatile uint32_t g_lcd_spi_calls;
volatile uint32_t g_lcd_spi_failures;
volatile uint32_t g_lcd_first_status;
volatile uint32_t g_lcd_first_hal_error;
volatile uint32_t g_lcd_first_spi_sr;
volatile uint32_t g_lcd_first_command;
static uint8_t ips200_last_command;

static HAL_StatusTypeDef ips200_transmit(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;
    g_lcd_spi_calls++;
    status = HAL_SPI_Transmit(&hspi2, (uint8_t *)data, length,
                              IPS200_SPI_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        if (g_lcd_spi_failures == 0U)
        {
            g_lcd_first_status = (uint32_t)status;
            g_lcd_first_hal_error = HAL_SPI_GetError(&hspi2);
            g_lcd_first_spi_sr = hspi2.Instance->SR;
            g_lcd_first_command = ips200_last_command;
        }
        g_lcd_spi_failures++;
    }
    return status;
}

typedef struct
{
    char character;
    uint8_t columns[5];
} ips200_glyph_t;

static const ips200_glyph_t ips200_font_5x7[] = {
    {' ', {0x00U, 0x00U, 0x00U, 0x00U, 0x00U}},
    {'-', {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}},
    {':', {0x00U, 0x36U, 0x36U, 0x00U, 0x00U}},
    {'+', {0x00U, 0x08U, 0x7FU, 0x08U, 0x00U}},
    {'.', {0x00U, 0x00U, 0x00U, 0x00U, 0x40U}},
    {'?', {0x02U, 0x01U, 0x51U, 0x09U, 0x06U}},
    {'0', {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}},
    {'1', {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}},
    {'2', {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}},
    {'3', {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}},
    {'4', {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}},
    {'5', {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}},
    {'6', {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}},
    {'7', {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}},
    {'8', {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'9', {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}},
    {'A', {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}},
    {'B', {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'C', {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}},
    {'D', {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}},
    {'E', {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}},
    {'F', {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}},
    {'G', {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}},
    {'H', {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}},
    {'I', {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}},
    {'J', {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}},
    {'K', {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}},
    {'L', {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}},
    {'M', {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}},
    {'N', {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}},
    {'O', {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}},
    {'P', {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}},
    {'Q', {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}},
    {'R', {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}},
    {'S', {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}},
    {'T', {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}},
    {'U', {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}},
    {'V', {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}},
    {'W', {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}},
    {'X', {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}},
    {'Y', {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}},
    {'Z', {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}}
};

static void ips200_delay_ms(uint32_t delay_ms)
{
    if (osKernelGetState() == osKernelRunning)
    {
        (void)osDelay(delay_ms);
    }
    else
    {
        HAL_Delay(delay_ms);
    }
}

static void ips200_select(void)
{
    HAL_GPIO_WritePin(IPS_CS_GPIO_Port, IPS_CS_Pin, GPIO_PIN_RESET);
}

static void ips200_deselect(void)
{
    HAL_GPIO_WritePin(IPS_CS_GPIO_Port, IPS_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef ips200_write_command(uint8_t command)
{
    ips200_last_command = command;
    HAL_GPIO_WritePin(IPS_DC_GPIO_Port, IPS_DC_Pin, GPIO_PIN_RESET);
    return ips200_transmit(&command, 1U);
}

static HAL_StatusTypeDef ips200_write_data(const uint8_t *data, uint16_t length)
{
    HAL_GPIO_WritePin(IPS_DC_GPIO_Port, IPS_DC_Pin, GPIO_PIN_SET);
    return ips200_transmit(data, length);
}

static HAL_StatusTypeDef ips200_write_command_data(uint8_t command,
                                                    const uint8_t *data,
                                                    uint16_t length)
{
    HAL_StatusTypeDef status;

    status = ips200_write_command(command);
    if ((status == HAL_OK) && (length != 0U))
    {
        status = ips200_write_data(data, length);
    }

    return status;
}

static uint8_t ips200_get_madctl(ips200_rotation_t rotation)
{
    switch (rotation)
    {
        case IPS200_ROTATION_PORTRAIT:
            return 0x00U;

        case IPS200_ROTATION_PORTRAIT_180:
            return 0xC0U;

        case IPS200_ROTATION_LANDSCAPE:
            return 0x70U;

        case IPS200_ROTATION_LANDSCAPE_180:
            return 0xA0U;

        default:
            return 0x00U;
    }
}

static void ips200_update_geometry(ips200_rotation_t rotation)
{
    if ((rotation == IPS200_ROTATION_LANDSCAPE) ||
        (rotation == IPS200_ROTATION_LANDSCAPE_180))
    {
        ips200_width = 320U;
        ips200_height = 240U;
    }
    else
    {
        ips200_width = 240U;
        ips200_height = 320U;
    }
}

static HAL_StatusTypeDef ips200_set_window(uint16_t x_start, uint16_t y_start,
                                           uint16_t x_end, uint16_t y_end)
{
    HAL_StatusTypeDef status;
    uint8_t column_data[4];
    uint8_t row_data[4];

    column_data[0] = (uint8_t)(x_start >> 8U);
    column_data[1] = (uint8_t)x_start;
    column_data[2] = (uint8_t)(x_end >> 8U);
    column_data[3] = (uint8_t)x_end;

    row_data[0] = (uint8_t)(y_start >> 8U);
    row_data[1] = (uint8_t)y_start;
    row_data[2] = (uint8_t)(y_end >> 8U);
    row_data[3] = (uint8_t)y_end;

    status = ips200_write_command_data(0x2AU, column_data, sizeof(column_data));
    if (status != HAL_OK)
    {
        return status;
    }

    status = ips200_write_command_data(0x2BU, row_data, sizeof(row_data));
    if (status != HAL_OK)
    {
        return status;
    }

    return ips200_write_command(0x2CU);
}

static HAL_StatusTypeDef ips200_write_color(uint16_t color, uint32_t pixel_count)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t color_buffer[IPS200_COLOR_BUFFER_PIXELS * 2U];
    uint32_t index;
    uint32_t chunk_size;

    for (index = 0U; index < IPS200_COLOR_BUFFER_PIXELS; index++)
    {
        color_buffer[index * 2U] = (uint8_t)(color >> 8U);
        color_buffer[index * 2U + 1U] = (uint8_t)color;
    }

    while (pixel_count != 0U)
    {
        chunk_size = pixel_count;
        if (chunk_size > IPS200_COLOR_BUFFER_PIXELS)
        {
            chunk_size = IPS200_COLOR_BUFFER_PIXELS;
        }

        status = ips200_write_data(color_buffer, (uint16_t)(chunk_size * 2U));
        if (status != HAL_OK)
        {
            break;
        }

        pixel_count -= chunk_size;
    }

    return status;
}

static const uint8_t *ips200_get_glyph(char character)
{
    uint32_t index;

    if ((character >= 'a') && (character <= 'z'))
    {
        character = (char)(character - ('a' - 'A'));
    }

    for (index = 0U; index < (sizeof(ips200_font_5x7) / sizeof(ips200_font_5x7[0])); index++)
    {
        if (ips200_font_5x7[index].character == character)
        {
            return ips200_font_5x7[index].columns;
        }
    }

    return ips200_font_5x7[3].columns;
}

static HAL_StatusTypeDef ips200_draw_char(uint16_t x, uint16_t y, char character,
                                          uint16_t foreground, uint16_t background)
{
    HAL_StatusTypeDef status;
    const uint8_t *glyph;
    uint8_t display_buffer[6U * 8U * 2U];
    uint16_t color;
    uint32_t column;
    uint32_t row;
    uint32_t offset = 0U;

    if (((uint32_t)x + 5U >= ips200_width) || ((uint32_t)y + 7U >= ips200_height))
    {
        return HAL_ERROR;
    }

    glyph = ips200_get_glyph(character);
    for (row = 0U; row < 8U; row++)
    {
        for (column = 0U; column < 6U; column++)
        {
            if ((column < 5U) && (row < 7U) && ((glyph[column] & (1U << row)) != 0U))
            {
                color = foreground;
            }
            else
            {
                color = background;
            }

            display_buffer[offset++] = (uint8_t)(color >> 8U);
            display_buffer[offset++] = (uint8_t)color;
        }
    }

    ips200_select();
    status = ips200_set_window(x, y, (uint16_t)(x + 5U), (uint16_t)(y + 7U));
    if (status == HAL_OK)
    {
        status = ips200_write_data(display_buffer, sizeof(display_buffer));
    }
    ips200_deselect();

    return status;
}

static HAL_StatusTypeDef ips200_send_init_sequence(ips200_rotation_t rotation)
{
    HAL_StatusTypeDef status;
    uint8_t madctl;
    static const uint8_t b2_data[] = {0x0CU, 0x0CU, 0x00U, 0x33U, 0x33U};
    static const uint8_t pixel_format_data[] = {0x05U};
    static const uint8_t b7_data[] = {0x35U};
    static const uint8_t bb_data[] = {0x29U};
    static const uint8_t c2_data[] = {0x01U};
    static const uint8_t c3_data[] = {0x19U};
    static const uint8_t c4_data[] = {0x20U};
    static const uint8_t c5_data[] = {0x1AU};
    static const uint8_t c6_data[] = {0x1FU};
    static const uint8_t d0_data[] = {0xA4U, 0xA1U};
    static const uint8_t e0_data[] = {
        0xD0U, 0x08U, 0x0EU, 0x09U, 0x09U, 0x05U, 0x31U,
        0x33U, 0x48U, 0x17U, 0x14U, 0x15U, 0x31U, 0x34U
    };
    static const uint8_t e1_data[] = {
        0xD0U, 0x08U, 0x0EU, 0x09U, 0x09U, 0x15U, 0x31U,
        0x33U, 0x48U, 0x17U, 0x14U, 0x15U, 0x31U, 0x34U
    };

    madctl = ips200_get_madctl(rotation);

    status = ips200_write_command_data(0x36U, &madctl, 1U);
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0x3AU, pixel_format_data, sizeof(pixel_format_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xB2U, b2_data, sizeof(b2_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xB7U, b7_data, sizeof(b7_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xBBU, bb_data, sizeof(bb_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xC2U, c2_data, sizeof(c2_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xC3U, c3_data, sizeof(c3_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xC4U, c4_data, sizeof(c4_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xC5U, c5_data, sizeof(c5_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xC6U, c6_data, sizeof(c6_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xD0U, d0_data, sizeof(d0_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xE0U, e0_data, sizeof(e0_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command_data(0xE1U, e1_data, sizeof(e1_data));
    if (status != HAL_OK) return status;
    status = ips200_write_command(0x21U);
    if (status != HAL_OK) return status;

    return ips200_write_command(0x29U);
}

HAL_StatusTypeDef ips200_init(ips200_rotation_t rotation)
{
    HAL_StatusTypeDef status;

    if (rotation > IPS200_ROTATION_LANDSCAPE_180)
    {
        return HAL_ERROR;
    }

    ips200_update_geometry(rotation);
    ips200_deselect();
    HAL_GPIO_WritePin(IPS_DC_GPIO_Port, IPS_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IPS_RST_GPIO_Port, IPS_RST_Pin, GPIO_PIN_RESET);
    g_lcd_stage = 1U;
    ips200_delay_ms(5U);
    HAL_GPIO_WritePin(IPS_RST_GPIO_Port, IPS_RST_Pin, GPIO_PIN_SET);
    g_lcd_stage = 2U;
    ips200_delay_ms(120U);

    g_lcd_stage = 3U;
    ips200_select();
    status = ips200_write_command(0x11U);
    ips200_deselect();
    if (status != HAL_OK)
    {
        return status;
    }

    g_lcd_stage = 4U;
    ips200_delay_ms(120U);

    g_lcd_stage = 5U;
    ips200_select();
    status = ips200_send_init_sequence(rotation);
    ips200_deselect();
    if (status != HAL_OK)
    {
        return status;
    }

    g_lcd_stage = 6U;
    status = ips200_fill_screen(IPS200_COLOR_WHITE);
    if (status == HAL_OK)
    {
        g_lcd_stage = 7U;
    }
    return status;
}

HAL_StatusTypeDef ips200_set_rotation(ips200_rotation_t rotation)
{
    HAL_StatusTypeDef status;
    uint8_t madctl;

    if (rotation > IPS200_ROTATION_LANDSCAPE_180)
    {
        return HAL_ERROR;
    }

    madctl = ips200_get_madctl(rotation);
    ips200_select();
    status = ips200_write_command_data(0x36U, &madctl, 1U);
    ips200_deselect();
    if (status == HAL_OK)
    {
        ips200_update_geometry(rotation);
    }

    return status;
}

HAL_StatusTypeDef ips200_fill_screen(uint16_t color)
{
    return ips200_fill_rect(0U, 0U, (uint16_t)(ips200_width - 1U),
                            (uint16_t)(ips200_height - 1U), color);
}

HAL_StatusTypeDef ips200_fill_rect(uint16_t x_start, uint16_t y_start,
                                   uint16_t x_end, uint16_t y_end,
                                   uint16_t color)
{
    HAL_StatusTypeDef status;
    uint32_t pixel_count;

    if ((x_start > x_end) || (y_start > y_end) ||
        (x_end >= ips200_width) || (y_end >= ips200_height))
    {
        return HAL_ERROR;
    }

    pixel_count = ((uint32_t)x_end - x_start + 1U) *
                  ((uint32_t)y_end - y_start + 1U);

    ips200_select();
    status = ips200_set_window(x_start, y_start, x_end, y_end);
    if (status == HAL_OK)
    {
        status = ips200_write_color(color, pixel_count);
    }
    ips200_deselect();

    return status;
}

HAL_StatusTypeDef ips200_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    return ips200_fill_rect(x, y, x, y, color);
}

HAL_StatusTypeDef ips200_draw_bitmap_rgb565(uint16_t x, uint16_t y,
                                             uint16_t width, uint16_t height,
                                             const uint16_t *pixels)
{
    HAL_StatusTypeDef status;
    uint8_t transmit_buffer[IPS200_COLOR_BUFFER_PIXELS * 2U];
    uint32_t pixel_count;
    uint32_t index;
    uint32_t chunk_size;

    if ((pixels == NULL) || (width == 0U) || (height == 0U) ||
        (x >= ips200_width) || (y >= ips200_height) ||
        ((uint32_t)x + width > ips200_width) ||
        ((uint32_t)y + height > ips200_height))
    {
        return HAL_ERROR;
    }

    pixel_count = (uint32_t)width * height;
    ips200_select();
    status = ips200_set_window(x, y, (uint16_t)(x + width - 1U),
                               (uint16_t)(y + height - 1U));

    while ((status == HAL_OK) && (pixel_count != 0U))
    {
        chunk_size = pixel_count;
        if (chunk_size > IPS200_COLOR_BUFFER_PIXELS)
        {
            chunk_size = IPS200_COLOR_BUFFER_PIXELS;
        }

        for (index = 0U; index < chunk_size; index++)
        {
            transmit_buffer[index * 2U] = (uint8_t)(pixels[index] >> 8U);
            transmit_buffer[index * 2U + 1U] = (uint8_t)pixels[index];
        }

        status = ips200_write_data(transmit_buffer, (uint16_t)(chunk_size * 2U));
        pixels += chunk_size;
        pixel_count -= chunk_size;
    }

    ips200_deselect();
    return status;
}

HAL_StatusTypeDef ips200_draw_text(uint16_t x, uint16_t y, const char *text,
                                   uint16_t foreground, uint16_t background)
{
    HAL_StatusTypeDef status;
    uint16_t start_x = x;

    if (text == NULL)
    {
        return HAL_ERROR;
    }

    while (*text != '\0')
    {
        if (*text == '\n')
        {
            x = start_x;
            y = (uint16_t)(y + 8U);
        }
        else
        {
            if ((uint32_t)x + 5U >= ips200_width)
            {
                x = start_x;
                y = (uint16_t)(y + 8U);
            }

            if ((uint32_t)y + 7U >= ips200_height)
            {
                return HAL_ERROR;
            }

            status = ips200_draw_char(x, y, *text, foreground, background);
            if (status != HAL_OK)
            {
                return status;
            }

            x = (uint16_t)(x + 6U);
        }

        text++;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ips200_draw_mono_bitmap(uint16_t x, uint16_t y,
                                          uint16_t width, uint16_t height,
                                          const uint8_t *bits,
                                          uint16_t foreground,
                                          uint16_t background)
{
    HAL_StatusTypeDef status;
    uint8_t transmit_buffer[IPS200_COLOR_BUFFER_PIXELS * 2U];
    uint32_t pixel_count;
    uint16_t bytes_per_row;
    uint32_t start_pixel;
    uint32_t chunk_size;
    uint32_t index;

    if ((bits == NULL) || (width == 0U) || (height == 0U) ||
        (x >= ips200_width) || (y >= ips200_height) ||
        ((uint32_t)x + width > ips200_width) ||
        ((uint32_t)y + height > ips200_height))
    {
        return HAL_ERROR;
    }

    pixel_count = (uint32_t)width * height;
    /* 每一行位图按整字节补齐；必须跳过行尾填充位，
     * 否则每行像素会逐渐偏移，导致图片斜向拉伸。 */
    bytes_per_row = (width + 7U) / 8U;
    start_pixel = 0U;

    ips200_select();
    status = ips200_set_window(x, y, (uint16_t)(x + width - 1U),
                               (uint16_t)(y + height - 1U));

    while ((status == HAL_OK) && (pixel_count != 0U))
    {
        chunk_size = pixel_count;
        if (chunk_size > IPS200_COLOR_BUFFER_PIXELS)
        {
            chunk_size = IPS200_COLOR_BUFFER_PIXELS;
        }

        for (index = 0U; index < chunk_size; index++)
        {
            uint32_t pixel_position = start_pixel + index;
            uint32_t row = pixel_position / width;
            uint32_t column = pixel_position % width;
            uint32_t byte_offset = ((uint32_t)bytes_per_row * row) +
                                   (column / 8U);
            uint16_t color = ((bits[byte_offset] &
                               (0x80U >> (column % 8U))) != 0U) ?
                             foreground : background;

            transmit_buffer[index * 2U] = (uint8_t)(color >> 8U);
            transmit_buffer[index * 2U + 1U] = (uint8_t)color;
        }

        status = ips200_write_data(transmit_buffer,
                                   (uint16_t)(chunk_size * 2U));
        start_pixel += chunk_size;
        pixel_count -= chunk_size;
    }

    ips200_deselect();
    return status;
}

HAL_StatusTypeDef ips200_draw_mono_bitmap_scaled(uint16_t x, uint16_t y,
                                                 uint16_t dest_width,
                                                 uint16_t dest_height,
                                                 const uint8_t *bits,
                                                 uint16_t source_width,
                                                 uint16_t source_height,
                                                 uint16_t foreground,
                                                 uint16_t background)
{
    HAL_StatusTypeDef status;
    uint8_t transmit_buffer[IPS200_COLOR_BUFFER_PIXELS * 2U];
    uint32_t pixel_count;
    uint16_t source_bytes_per_row;
    uint32_t start_pixel;
    uint32_t chunk_size;
    uint32_t index;

    if ((bits == NULL) || (dest_width == 0U) || (dest_height == 0U) ||
        (source_width == 0U) || (source_height == 0U) ||
        (x >= ips200_width) || (y >= ips200_height) ||
        ((uint32_t)x + dest_width > ips200_width) ||
        ((uint32_t)y + dest_height > ips200_height))
    {
        return HAL_ERROR;
    }

    pixel_count = (uint32_t)dest_width * dest_height;
    source_bytes_per_row = (source_width + 7U) / 8U;
    start_pixel = 0U;

    ips200_select();
    status = ips200_set_window(x, y,
                               (uint16_t)(x + dest_width - 1U),
                               (uint16_t)(y + dest_height - 1U));

    while ((status == HAL_OK) && (pixel_count != 0U))
    {
        chunk_size = pixel_count;
        if (chunk_size > IPS200_COLOR_BUFFER_PIXELS)
        {
            chunk_size = IPS200_COLOR_BUFFER_PIXELS;
        }

        for (index = 0U; index < chunk_size; index++)
        {
            uint32_t pixel_position = start_pixel + index;
            uint32_t dest_row = pixel_position / dest_width;
            uint32_t dest_column = pixel_position % dest_width;

            /* 最近邻插值：将目标像素映射回原图最近位置。 */
            uint32_t source_row =
                (dest_row * (uint32_t)source_height) / dest_height;
            uint32_t source_column =
                (dest_column * (uint32_t)source_width) / dest_width;
            uint32_t byte_offset =
                ((uint32_t)source_bytes_per_row * source_row) +
                (source_column / 8U);
            uint8_t bit_mask = (uint8_t)(0x80U >> (source_column % 8U));
            uint16_t color = ((bits[byte_offset] & bit_mask) != 0U) ?
                             foreground : background;

            transmit_buffer[index * 2U] = (uint8_t)(color >> 8U);
            transmit_buffer[index * 2U + 1U] = (uint8_t)color;
        }

        status = ips200_write_data(transmit_buffer,
                                   (uint16_t)(chunk_size * 2U));
        start_pixel += chunk_size;
        pixel_count -= chunk_size;
    }

    ips200_deselect();
    return status;
}

HAL_StatusTypeDef ips200_draw_test_pattern(void)
{
    HAL_StatusTypeDef status;
    uint16_t band_width;
    uint16_t bar_top;
    uint16_t bar_bottom;

    status = ips200_fill_screen(IPS200_COLOR_BLACK);
    if (status != HAL_OK) return status;

    status = ips200_draw_text(12U, 12U, "IPS200 OK", IPS200_COLOR_YELLOW, IPS200_COLOR_BLACK);
    if (status != HAL_OK) return status;
    status = ips200_draw_text(12U, 28U, "FREERTOS", IPS200_COLOR_CYAN, IPS200_COLOR_BLACK);
    if (status != HAL_OK) return status;

    band_width = (uint16_t)((ips200_width - 40U) / 4U);
    bar_top = 56U;
    bar_bottom = (uint16_t)(ips200_height - 20U);

    status = ips200_fill_rect(10U, bar_top, (uint16_t)(10U + band_width - 1U),
                              bar_bottom, IPS200_COLOR_RED);
    if (status != HAL_OK) return status;
    status = ips200_fill_rect((uint16_t)(20U + band_width), bar_top,
                              (uint16_t)(20U + 2U * band_width - 1U),
                              bar_bottom, IPS200_COLOR_GREEN);
    if (status != HAL_OK) return status;
    status = ips200_fill_rect((uint16_t)(30U + 2U * band_width), bar_top,
                              (uint16_t)(30U + 3U * band_width - 1U),
                              bar_bottom, IPS200_COLOR_BLUE);
    if (status != HAL_OK) return status;

    return ips200_fill_rect((uint16_t)(40U + 3U * band_width), bar_top,
                            (uint16_t)(ips200_width - 11U), bar_bottom,
                            IPS200_COLOR_WHITE);
}

uint16_t ips200_get_width(void)
{
    return ips200_width;
}

uint16_t ips200_get_height(void)
{
    return ips200_height;
}
