
//#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "tft_gfx.h"
#include "serial.h"
#include <string.h>
#include "avr_gpio.h"
#include "logger.h"

#include <stdlib.h>

#define LED_PIN       5
#define LED_DDR       &(DDRB)
#define LED_PORT      &(PORTB)

int main (void)
{
    serial_init();
    sei();

    logget_set_observer((logger_observer_t)serial_send_block);

    char buff[250];
    sprintf(buff, "ok.\r\n");
    serial_send_block((uint8_t*)buff, strlen(buff));

    tft_display_init();

    // avr_gpio_pins_dir_set(LED_DDR, (1 << LED_PIN), AVR_GPIO_PIN_DIR_OUTPUT);
    // tft_display_id_info_t tft_info = tft_ll_read_display_id();
    // sprintf(buff, "lcd_manuf_id: %u\n\rlcd_driver_version: %u\n\rlcd_driver_id: %u\n\r",
    //     tft_info.lcd_manuf_id, tft_info.lcd_driver_version, tft_info.lcd_driver_id);
    // serial_send_block((uint8_t*)buff, strlen(buff));

    // tft_display_status_t status = tft_ll_read_display_status();
    // sprintf(buff, "status: %0lu\n\r", status.status);
    // serial_send_block((uint8_t*)buff, strlen(buff));

    // tft_display_id_t id = tft_ll_read_id();
    // sprintf(buff, "IC_version %X, IC_model_name: %X, IC_model_name: %X\n\r", id.ic_version, id.ic_model_name[0], id.ic_model_name[1]);
    // serial_send_block((uint8_t*)buff, strlen(buff));

    point_t a, b;
    // color_t rand_color;

    a.x = 10;
    a.y = 20;
    b.x = 100;
    b.y = 200;

    tft_draw_rect(a, b, TFT_COLOR_RED);

    log(LEVEL_INFO, "hello: %d", 1);
    for(;;)
    {
        _delay_ms(1000);
        for(char i = 32; i < 127; ++i)
        {
            tft_draw_char(i);
        }

        // a.x = rand() % 240;
        // b.x = rand() % 240 + a.x;
        // a.y = rand() % 320;
        // b.y = rand() % 320 + a.y;
        // rand_color = rand() % 0xFFFF + 0xFF00;
        // tft_ll_fill_area(a, b, rand_color);

        // point_info_t rand_p;
        // rand_p.x = rand() % 240;
        // rand_p.y = rand() % 320;
        // rand_p.color   = rand() % 0xFFFF + 0xFF00;
        // tft_ll_draw_pixel(rand_p);
    }
    return 0;
}
