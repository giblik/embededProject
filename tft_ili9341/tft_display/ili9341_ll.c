#include <stdarg.h>
#include "ili9341_ll.h"
#include "display_config.h"

#if defined(ARDUINO_BOARD)
    #include "avr_gpio.h"
    #include <util/delay.h>

    #define CS_HI()       avr_gpio_pin_set(CS_PIN_PORT, CS_PIN)
    #define CS_LO()       avr_gpio_pin_clear(CS_PIN_PORT, CS_PIN)
    #define WR_HI()       avr_gpio_pin_set(WR_PIN_PORT, WR_PIN)
    #define WR_LO()       avr_gpio_pin_clear(WR_PIN_PORT, WR_PIN)
    #define RD_HI()       avr_gpio_pin_set(RD_PIN_PORT, RD_PIN)
    #define RD_LO()       avr_gpio_pin_clear(RD_PIN_PORT, RD_PIN)
    #define RS_DATA()     avr_gpio_pin_set(RS_PIN_PORT, RS_PIN)
    #define RS_CMD()      avr_gpio_pin_clear(RS_PIN_PORT, RS_PIN)
    #define RST_HI()      avr_gpio_pin_set(RST_PIN_PORT, RST_PIN)
    #define RST_LO()      avr_gpio_pin_clear(RST_PIN_PORT, RST_PIN)

    #define pause()                 //_delay_us(10)

    #define read_port() ((PIND & 0b11111100) | (PINB & 0b00000011))
    #define write_port(data)                                    \
    do                                                          \
    {                                                           \
        PORTD = (PORTD & 0b00000011) | ((data) & 0b11111100);   \
        PORTB = (PORTB & 0b11111100) | ((data) & 0b00000011);   \
    } while(0)

    #define pins_direction(dir)                                                  \
    do                                                                           \
    {                                                                            \
        avr_gpio_pins_dir_set(&DDRB, (1<<PIN0)|(1<<PIN1), dir);                  \
        avr_gpio_pins_dir_set(&DDRD,                                             \
            (1<<PIN2)|(1<<PIN3)|(1<<PIN4)|(1<<PIN5)|(1<<PIN6)|(1<<PIN7), dir);   \
    } while(0)
    #define pins_as_input()         pins_direction(AVR_GPIO_PIN_DIR_INPUT)
    #define pins_as_output()        pins_direction(AVR_GPIO_PIN_DIR_OUTPUT)
    #define gpio_init()                                                         \
    do                                                                          \
    {                                                                           \
        pins_as_output();                                                       \
        avr_gpio_pins_dir_set(CMD_DDR,                                          \
        (1<<RST_PIN)|(1<<CS_PIN)|(1<<RS_PIN)|(1<<WR_PIN)|(1<<RD_PIN),           \
        AVR_GPIO_PIN_DIR_OUTPUT);                                               \
                                                                                \
        avr_gpio_pin_set(RST_PIN_PORT, RST_PIN);                                \
        avr_gpio_pin_set(RD_PIN_PORT, RD_PIN);                                  \
        avr_gpio_pin_set(WR_PIN_PORT, WR_PIN);                                  \
    } while(0)
#elif defined(STM32_BOARD)
    #error "NOT SUPORTED"
#elif defined(NRF52_BOARD)
    #error "NOT SUPORTED"
#else
    #error "Not selected board."
#endif

#define wr_strobe()     \
do                      \
{                       \
    WR_LO();            \
    WR_HI();            \
    pause();            \
} while(0)

#define rd_strobe()     \
do                      \
{                       \
    RD_LO();            \
    pause();            \
    RD_HI();            \
} while(0)

typedef enum
{
    ILI9341_NOP                = 0x00,
    ILI9341_SOFTRESET          = 0x01,
    ILI9341_DISPLAYSSTATUS     = 0x09,
    ILI9341_SLEEPIN            = 0x10,
    ILI9341_SLEEPOUT           = 0x11,
    ILI9341_NORMALDISP         = 0x13,
    ILI9341_INVERTOFF          = 0x20,
    ILI9341_INVERTON           = 0x21,
    ILI9341_GAMMASET           = 0x26,
    ILI9341_DISPLAYOFF         = 0x28,
    ILI9341_DISPLAYON          = 0x29,
    ILI9341_COLADDRSET         = 0x2A,
    ILI9341_PAGEADDRSET        = 0x2B,
    ILI9341_MEMORYWRITE        = 0x2C,
    ILI9341_MEMORYREAD         = 0x2E,
    ILI9341_PIXELFORMAT        = 0x3A,
    ILI9341_FRAMECONTROL       = 0xB1,
    ILI9341_DISPLAYFUNC        = 0xB6,
    ILI9341_ENTRYMODE          = 0xB7,
    ILI9341_POWERCONTROL1      = 0xC0,
    ILI9341_POWERCONTROL2      = 0xC1,
    ILI9341_VCOMCONTROL1       = 0xC5,
    ILI9341_VCOMCONTROL2       = 0xC7,
    ILI9341_MEMCONTROL         = 0x36,
    ILI9341_MADCTL             = 0x36,
    ILI9341_READ_ID4           = 0xD3,

    ILI9341_MADCTL_MY          = 0x80,
    ILI9341_MADCTL_MX          = 0x40,
    ILI9341_MADCTL_MV          = 0x20,
    ILI9341_MADCTL_ML          = 0x10,
    ILI9341_MADCTL_RGB         = 0x00,
    ILI9341_MADCTL_BGR         = 0x08,
    ILI9341_MADCTL_MH          = 0x04
} ili9341_cmd_t;

static color_t m_backgroung_color;

static inline void write_mem8(uint8_t data)
{
    write_port(data);
    wr_strobe();
}

static inline void write_mem16(uint16_t data)
{
    write_port(data >> 8);
    wr_strobe();
    write_port(data & 0xFF);
    wr_strobe();
}

static void write_seq(uint8_t cmd, uint8_t num, ...)
{
    // char buff[25];
    CS_LO();
    /* command address */
    RS_CMD();
    write_port(cmd);
    wr_strobe();
    /* command data */
    RS_DATA();
    va_list va;
    va_start(va, num);
    while(num--)
    {
        uint8_t data = (uint8_t)va_arg(va, unsigned);
        write_port(data);
        wr_strobe();
    }
    va_end(va);

    CS_LO();
}

void read_seq(uint8_t cmd, void *p_payload, uint8_t size)
{
    CS_LO();
    /* command address */
    RS_CMD();
    write_port(cmd);
    wr_strobe();
    /* command data */
    RS_DATA();
    pins_as_input();
    rd_strobe();                               // dummy data
    for(uint8_t i = 0 ; i < size; ++i)
    {
        rd_strobe();
        ((uint8_t*)(p_payload))[i] = read_port();
    }
    pins_as_output();

    CS_HI();
}

static void set_orientation(uint8_t orient)
{
      switch (orient)
      {
        case 0: write_seq(0x36, 1, ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
                break;
        case 1: write_seq(0x36, 1, ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
                break;
        case 2: write_seq(0x36, 1, ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
                break;
        case 3: write_seq(0x36, 1, 0xE8);
                break;
      }
}

void tft_ll_display_init(void)
{
    gpio_init();

    write_seq(ILI9341_SOFTRESET, 0);
    _delay_ms(50);
    write_seq(0, 0x28, 0);

    write_seq(ILI9341_POWERCONTROL1, 1, 0x23);
    write_seq(ILI9341_POWERCONTROL2, 1, 0x10);
    write_seq(ILI9341_VCOMCONTROL1, 2, 0x2B, 0x2B);
    write_seq(ILI9341_VCOMCONTROL2, 1, 0xC0);
    set_orientation(0);
    write_seq(ILI9341_PIXELFORMAT, 1, 0x55);
    write_seq(ILI9341_FRAMECONTROL, 2, 0x00, 0x1B);

    write_seq(ILI9341_ENTRYMODE, 1, 0x07);

    write_seq(ILI9341_SLEEPOUT, 0);
    _delay_ms(150);
    write_seq(ILI9341_DISPLAYON, 0);
    _delay_ms(500);

    point_t x = { .x = 0, .y = 0 };
    point_t y = { .x = 239, .y = 319 };
    tft_ll_fill_area(x, y, m_backgroung_color);
}

void tft_ll_nop(void)
{
    write_seq(ILI9341_NOP, 0);
}


static void set_col(uint16_t start, uint16_t end)
{
    write_seq(ILI9341_COLADDRSET,
                sizeof(start) + sizeof(end),
                (start >> 8), (start & 0xFF),
                (end >> 8), (end & 0xFF));
}
static void set_row(uint16_t start, uint16_t end)
{
    write_seq(ILI9341_PAGEADDRSET,
                sizeof(start) + sizeof(end),
                (start >>8), (start & 0xFF),
                (end >>8), (end & 0xFF));
}
static void set_xy(uint16_t x, uint16_t y)
{
    set_col(x, x);
    set_row(y, y);
}

void tft_ll_background_color_set(color_t color)
{
    m_backgroung_color = color;
}

void tft_ll_soft_reset(void)
{
    write_seq(ILI9341_SOFTRESET, 0);
}

tft_display_id_t tft_ll_read_id(void)
{
    tft_display_id_t id;
    read_seq(ILI9341_READ_ID4, &id, sizeof(id));
    return id;
}

tft_display_status_t tft_ll_read_display_status(void)
{
    tft_display_status_t display_status;

    read_seq(ILI9341_DISPLAYSSTATUS, &display_status, sizeof(display_status));
    return display_status;
}

void tft_ll_draw_pixel(point_info_t point)
{
    set_xy(point.x, point.y);
    write_seq(ILI9341_MEMORYWRITE, sizeof(point.color),
        (point.color >> 8), (point.color & 0x00FF));
}

#include <stdio.h>
#include "string.h"
#include "serial.h"
void tft_ll_fill_area(point_t a, point_t b, color_t color)
{
    set_col(a.x, b.x);
    set_row(a.y, b.y);

    uint16_t x = (uint16_t)((b.x - a.x)+1);
    uint16_t y = (uint16_t)((b.y - a.y)+1);
    uint32_t xy =  (uint32_t)x * y;

    write_seq(ILI9341_MEMORYWRITE, 0);

    CS_LO();
    RS_DATA();
    char buff[50];
    sprintf(buff, "a.x:%u a.y:%u b.x:%u b.y:%u xy=%lu\n\r",
        a.x, a.y, b.x, b.y, xy);
    serial_send_block((uint8_t*)buff, strlen(buff));
    for(uint32_t i=0; i < xy; i++)
    {
        write_mem16(color);
    }

    CS_HI();
}

void tft_ll_char_write(point_t *p_point, color_t color, uint8_t *p_buff, uint8_t size)
{
    set_col(p_point->x, p_point->x + size-1);
    set_row(p_point->y, p_point->y + 16);

    write_seq(ILI9341_MEMORYWRITE, 0);
    CS_LO();
    RS_DATA();

    for(uint8_t a = 0; a < 2; a++)
    {
        for(uint8_t r = 0; r < 8; r++)
        {
            for(uint8_t  i = a; i < size*2; i+=2)
            {
                if (p_buff[i] & (1 << r))
                {
                    write_mem16(color);
                }
                else
                {
                    write_mem16(m_backgroung_color);
                }
            }
        }
    }

    CS_HI();
}

