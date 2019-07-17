#include "ili9341_ll.h"
#include "display_config.h"

#if defined(ARDUINO_BOARD)
    #include "avr_gpio.h"
    #include <util/delay.h>
    #define reset_up()              avr_gpio_pin_set(RST_PIN_PORT, RST_PIN)
    #define reset_down()            avr_gpio_pin_clear(RST_PIN_PORT, RST_PIN)
    #define cs_set()                avr_gpio_pin_set(CS_PIN_PORT, CS_PIN)
    #define cs_reset()              avr_gpio_pin_clear(CS_PIN_PORT, CS_PIN)
    #define data_set()              avr_gpio_pin_set(RS_PIN_PORT, RS_PIN)
    #define command_set()           avr_gpio_pin_clear(RS_PIN_PORT, RS_PIN)
    #define wr_set()                avr_gpio_pin_set(WR_PIN_PORT, WR_PIN)
    #define wr_reset()              avr_gpio_pin_clear(WR_PIN_PORT, WR_PIN)
    #define rd_set()                avr_gpio_pin_set(RD_PIN_PORT, RD_PIN)
    #define rd_reset()              avr_gpio_pin_clear(RD_PIN_PORT, RD_PIN)
    #define pause()                 _delay_ms(10)
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
        avr_gpio_pin_dir_set(&DDRD, PIN_0|PIN_1, dir);                           \
        avr_gpio_pin_dir_set(&DDRB, PIN_2|PIN_3|PIN_4|PIN_5|PIN_6|PIN_7, dir);   \
    } while(0)
    #define pins_as_input()         pins_direction(AVR_GPIO_PIN_DIR_INPUT)
    #define pins_as_output()        pins_direction(AVR_GPIO_PIN_DIR_OUTPUT)
    #define gpio_init()                                                         \
    do                                                                          \
    {                                                                           \
        pins_as_output();                                                       \
        avr_gpio_pin_dir_set(RST_PIN_DDR, RST_PIN, AVR_GPIO_PIN_DIR_OUTPUT);    \
        avr_gpio_pin_dir_set(CS_PIN_DDR, CS_PIN, AVR_GPIO_PIN_DIR_OUTPUT);      \
        avr_gpio_pin_dir_set(RS_PIN_DDR, RS_PIN, AVR_GPIO_PIN_DIR_OUTPUT);      \
        avr_gpio_pin_dir_set(WR_PIN_DDR, WR_PIN, AVR_GPIO_PIN_DIR_OUTPUT);      \
        avr_gpio_pin_dir_set(RD_PIN_DDR, RD_PIN, AVR_GPIO_PIN_DIR_OUTPUT);      \
    } while(0)
#elif defined(STM32_BOARD)
    #error "NOT SUPORTED"
#elif defined(NRF52_BOARD)
    #error "NOT SUPORTED"
#else
    #error "Not selected board."
#endif

typedef enum
{
    CMD_NOP         = 0x00,
    CMD_SWRESET     = 0x01,
    CMD_RDDIDIF     = 0x04,
    CMD_RDDST       = 0x09,
    CMD_RDDPM       = 0x0A,

    CMD_RAMWR       = 0x2C,
} ll_cmd_t;

static inline void ll_cmd_write(ll_cmd_t cmd)
{
    command_set();
    rd_set();
    cs_reset();
    wr_reset();
    write_port(cmd);
    pause();
    wr_set();
    cs_set();
}

static inline uint8_t ll_mem_read()
{
    pins_as_input();
    data_set();
    wr_set();
    cs_reset();
    wr_reset();
    pause();
    uint8_t data = read_port();
    rd_set();
    cs_set();
    pins_as_output();
    return data;
}

static inline void ll_mem_write(uint8_t data)
{
    data_set();
    rd_set();
    cs_reset();
    write_port(data);
    wr_reset();
    pause();
    wr_set();
    cs_set();
}

void tft_ll_nop(void)
{
    ll_cmd_write(CMD_NOP);
}

void tft_ll_soft_reset(void)
{
    ll_cmd_write(CMD_SWRESET);
}

tft_display_id_info_t tft_ll_read_display_id(void)
{
    tft_display_id_info_t display_info;

    ll_cmd_write(CMD_RDDIDIF);
    (void)ll_mem_read();                                                /*< dummy data */
    display_info.lcd_manuf_id       = ll_mem_read();
    display_info.lcd_driver_version = ll_mem_read();
    display_info.lcd_driver_id      = ll_mem_read();

    return display_info;
}

tft_display_status_t tft_ll_read_display_status(void)
{
    tft_display_status_t display_status;

    ll_cmd_write(CMD_RDDST);
    (void)ll_mem_read();                                                /*< dummy data */
    display_status.status  = (uint32_t)ll_mem_read() << 24;
    display_status.status |= (uint32_t)ll_mem_read() << 16;
    display_status.status |= (uint32_t)ll_mem_read() << 8;
    display_status.status |= ll_mem_read();

    return display_status;
}

tft_display_power_mode_t tft_ll_read_display_power_mode(void)
{
    tft_display_power_mode_t *p_power_mode;

    ll_cmd_write(CMD_RDDPM);
    (void)ll_mem_read();                                                /*< dummy data */
    uint8_t data = ll_mem_read();
    p_power_mode = (tft_display_power_mode_t *)(&data);

    return *p_power_mode;
}

void tft_ll_display_init(void)
{
    gpio_init();
}
