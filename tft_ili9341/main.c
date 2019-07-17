
//#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "ili9341_ll.h"

int main (void)
{
    tft_ll_display_init();

    for(;;)
    {
        _delay_ms(1000);
    }
    return 0;
}
