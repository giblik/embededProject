
//#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "ili9341_ll.h"
#include "serial.h"
#include <string.h>

int main (void)
{
    serial_init();
    sei();

    char buff[250] = "qwerty";
    serial_send_block((uint8_t*)buff, strlen(buff));
    tft_ll_display_init();
    _delay_ms(200);

    tft_display_id_info_t tft_info =tft_ll_read_display_id();
    sprintf(buff, "lcd_manuf_id: %u\n\rlcd_driver_version: %u\n\rlcd_driver_id: %u\n\r",
        tft_info.lcd_manuf_id, tft_info.lcd_driver_version, tft_info.lcd_driver_id);
    serial_send_block((uint8_t*)buff, strlen(buff));
    for(;;)
    {
        _delay_ms(1000);
    }
    return 0;
}
