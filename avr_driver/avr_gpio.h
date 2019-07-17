#ifndef AVR_GPIO_H__
#define AVR_GPIO_H__

#define __STATIC_INLINE 	                static inline

#define PIN_0            (1 << 0)
#define PIN_1            (1 << 1)
#define PIN_2            (1 << 2)
#define PIN_3            (1 << 3)
#define PIN_4            (1 << 4)
#define PIN_5            (1 << 5)
#define PIN_6            (1 << 6)
#define PIN_7            (1 << 7)

typedef enum
{
    AVR_GPIO_PIN_DIR_INPUT,
    AVR_GPIO_PIN_DIR_OUTPUT
} avr_gpio_pin_dir_t;


__STATIC_INLINE void avr_gpio_pin_dir_set(volatile uint8_t *p_ddr_reg, uint8_t pin_mask, avr_gpio_pin_dir_t dir);
__STATIC_INLINE void avr_gpio_pin_set(volatile uint8_t *p_port_reg, uint8_t pin);
__STATIC_INLINE void avr_gpio_pin_clear(volatile uint8_t *p_port_reg, uint8_t pin);


__STATIC_INLINE void avr_gpio_pin_dir_set(volatile uint8_t *p_ddr_reg, uint8_t pin_mask, avr_gpio_pin_dir_t dir)
{
    switch(dir)
    {
        case AVR_GPIO_PIN_DIR_INPUT:
        {
            *p_ddr_reg &= ~(pin_mask);
            break;
        }
        case AVR_GPIO_PIN_DIR_OUTPUT:
        {
            *p_ddr_reg |=  (pin_mask);
            break;
        }
        default:
            break;
    }
}

__STATIC_INLINE void avr_gpio_pin_set(volatile uint8_t *p_port_reg, uint8_t pin)
{
    *p_port_reg |= (1 << pin);
}

__STATIC_INLINE void avr_gpio_pin_clear(volatile uint8_t *p_port_reg, uint8_t pin)
{
    *p_port_reg &= ~(1 << pin);
}

#endif /* AVR_GPIO_H__ */
