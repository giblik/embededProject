#ifndef ILI9341_LL_H__
#define ILI9341_LL_H__

#include <stdint.h>

typedef struct
{
    uint8_t lcd_manuf_id;
    uint8_t lcd_driver_version;
    uint8_t lcd_driver_id;
} tft_display_id_info_t;

typedef struct
{
    uint32_t status;
} tft_display_status_t;

typedef struct
{
    uint8_t booster         :1;
    uint8_t idle_mode       :1;
    uint8_t partial_mode    :1;
    uint8_t sleep_mode      :1;
    uint8_t display_mode    :1;
    uint8_t display_onoff   :1;
    uint8_t rfu             :2;
} tft_display_power_mode_t;

void tft_ll_nop(void);
void tft_ll_soft_reset(void);
void tft_ll_display_init(void);
tft_display_id_info_t tft_ll_read_display_id(void);
tft_display_status_t tft_ll_read_display_status(void);
tft_display_power_mode_t tft_ll_read_display_power_mode(void);

#endif /* ILI9341_LL_H__ */
