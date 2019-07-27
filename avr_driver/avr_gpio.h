#ifndef AVR_GPIO_H__
#define AVR_GPIO_H__

#define __STATIC_INLINE 	                static inline

typedef enum
{
    AVR_GPIO_PIN_DIR_INPUT,
    AVR_GPIO_PIN_DIR_OUTPUT
} avr_gpio_pin_dir_t;

__STATIC_INLINE void avr_gpio_pins_dir_set(volatile uint8_t *p_ddr_reg, uint8_t pin_mask, avr_gpio_pin_dir_t dir);
__STATIC_INLINE void avr_gpio_pin_set(volatile uint8_t *p_port_reg, uint8_t pin);
__STATIC_INLINE void avr_gpio_pin_clear(volatile uint8_t *p_port_reg, uint8_t pin);


__STATIC_INLINE void avr_gpio_pins_dir_set(volatile uint8_t *p_ddr_reg, uint8_t pin_mask, avr_gpio_pin_dir_t dir)
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
