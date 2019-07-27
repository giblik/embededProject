#ifndef DISPLAY_CONFIG_H__
#define DISPLAY_CONFIG_H__

#ifdef ARDUINO_BOARD
#include <avr/io.h>
#include "avr_gpio.h"

#define PIN0          0
#define PIN1          1
#define PIN2          2
#define PIN3          3
#define PIN4          4
#define PIN5          5
#define PIN6          6
#define PIN7          7

#define RST_PIN       4
#define CS_PIN        3
#define RS_PIN        2
#define WR_PIN        1
#define RD_PIN        0

#define D0_PIN_DDR    &(DDRB)
#define D1_PIN_DDR    &(DDRB)
#define D2_PIN_DDR    &(DDRD)
#define D3_PIN_DDR    &(DDRD)
#define D4_PIN_DDR    &(DDRD)
#define D5_PIN_DDR    &(DDRD)
#define D6_PIN_DDR    &(DDRD)
#define D7_PIN_DDR    &(DDRD)

#define RST_PIN_DDR   &(DDRC)
#define RS_PIN_DDR    &(DDRC)
#define CS_PIN_DDR    &(DDRC)
#define WR_PIN_DDR    &(DDRC)
#define RD_PIN_DDR    &(DDRC)
#define CMD_DDR       &(DDRC)

#define D0_PIN_PORT   &(PORTB)
#define D1_PIN_PORT   &(PORTB)
#define D3_PIN_PORT   &(PORTD)
#define D2_PIN_PORT   &(PORTD)
#define D4_PIN_PORT   &(PORTD)
#define D5_PIN_PORT   &(PORTD)
#define D6_PIN_PORT   &(PORTD)
#define D7_PIN_PORT   &(PORTD)

#define CS_PIN_PORT   &(PORTC)
#define RST_PIN_PORT  &(PORTC)
#define RS_PIN_PORT   &(PORTC)
#define WR_PIN_PORT   &(PORTC)
#define RD_PIN_PORT   &(PORTC)
#define CMD_PORT      &(PORTC)
#endif

#endif /* DISPLAY_CONFIG_H__ */
