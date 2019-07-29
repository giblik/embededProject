#ifndef ILI9341_LL_H__
#define ILI9341_LL_H__

#include <stdint.h>

#define TFT_WIDTH           240
#define TFT_HEIGHT          320
typedef enum
{
    TFT_COLOR_BLACK       = 0x0000,
    TFT_COLOR_NAVY        = 0x000F,
    TFT_COLOR_DARKGREEN   = 0x03E0,
    TFT_COLOR_DARKCYAN    = 0x03EF,
    TFT_COLOR_MAROON      = 0x7800,
    TFT_COLOR_PURPLE      = 0x780F,
    TFT_COLOR_OLIVE       = 0x7BE0,
    TFT_COLOR_LIGHTGREY   = 0xC618,
    TFT_COLOR_DARKGREY    = 0x7BEF,
    TFT_COLOR_BLUE        = 0x001F,
    TFT_COLOR_GREEN       = 0x07E0,
    TFT_COLOR_CYAN        = 0x07FF,
    TFT_COLOR_RED         = 0xF800,
    TFT_COLOR_MAGENTA     = 0xF81F,
    TFT_COLOR_YELLOW      = 0xFFE0,
    TFT_COLOR_WHITE       = 0xFFFF,
    TFT_COLOR_ORANGE      = 0xFD20,
    TFT_COLOR_GREENYELLOW = 0xAFE5,
    TFT_COLOR_PINK        = 0xF81F
} color_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
} point_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
    color_t color;
} point_info_t;

typedef struct
{
    uint32_t status;
} tft_display_status_t;

typedef struct
{
    uint8_t ic_version;
    uint8_t ic_model_name[2];
} tft_display_id_t;

void tft_ll_nop(void);
void tft_ll_soft_reset(void);
void tft_ll_background_color_set(color_t color);
void tft_ll_display_init(void);
tft_display_id_t tft_ll_read_id(void);
tft_display_status_t tft_ll_read_display_status(void);
void tft_ll_draw_pixel(point_info_t point);
void tft_ll_fill_area(point_t a, point_t b, color_t color);
void tft_ll_char_write(point_t *p_point, color_t color, uint8_t *p_buff, uint8_t size);

#endif /* ILI9341_LL_H__ */
