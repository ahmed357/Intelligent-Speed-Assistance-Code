/****************************************************
* File:gpio_driver.h
* This is the GPIO header file for STM32F401 Arm MCU
* 
* Part of ISA using Wireless Ad-hoc Networks Project
* Abdullah Ahmed
* City, University of London
*////////////////////////////////////////////////////

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32f401xe.h"

/* The following was obtained by referring to the GPIO registers reference manual on page 158 and 118 */

/* PIN INITIALISATION */

/* GPIO Mode Settings */
#define GPIO_INPUT_MODE                             ( (uint32_t)0x00 )
#define GPIO_OUTPUT_MODE                            ( (uint32_t)0x01 )
#define GPIO_ALT_FUN_MODE                           ( (uint32_t)0x02 )
#define GPIO_ANALOG_MODE                            ( (uint32_t)0x03 )

/* GPIO Output Type Settings */
#define GPIO_OUTPUT_TYPE_PUSHPULL                   ( (uint32_t)0x00 )
#define GPIO_OUTPUT_TYPE_OPENDRAIN                  ( (uint32_t)0x01 )

/* GPIO Speed Settings */
#define GPIO_SPEED_LOW                              ( (uint32_t)0x00 )                     
#define GPIO_SPEED_MEDIUM                           ( (uint32_t)0x01 )
#define GPIO_SPEED_HIGH                             ( (uint32_t)0x02 )
#define GPIO_SPEED_VERYHIGH                         ( (uint32_t)0x03 )

/* GPIO Pull up/down Resistor Settings */
#define GPIO_NO_PULL                                ( (uint32_t)0x00 )
#define GPIO_PULL_UP                                ( (uint32_t)0x01 )
#define GPIO_PULL_DOWN                              ( (uint32_t)0x02 )

/* GPIO Enable Clock to Pins */
#define GPIOA_CLK_ENABLE                            (RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_CLK_ENABLE                            (RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_CLK_ENABLE                            (RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_CLK_ENABLE                            (RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_CLK_ENABLE                            (RCC->AHB1ENR |= (1 << 4) )
#define GPIOH_CLK_ENABLE                            (RCC->AHB1ENR |= (1 << 7) )


/* DATA STRUCTUREs FOR PIN INITIALISATION */

typedef struct {

	uint32_t pin;
	uint32_t mode;
	uint32_t outputtype;
	uint32_t pull;
	uint32_t speed;
	uint32_t alternate;
	
}gpio_config_t;

typedef struct {
	gpio_register *GPIOx;
	gpio_config_t gpio_config;

}gpio_handler;

typedef enum {

	RISING_EDGE,
	FALLING_EDGE,
	RISING_FALLING_EDGE

}edge_selector;


/* DRIVER APIs */

void gpio_init(GPIO_TypeDef *GPIOx, gpio_config *gpio_conf);

uint8_t gpio_read(GPIO_TypeDef *GPIOx, uint16_t pin_no);

void gpio_write(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val);

void gpio_set_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint16_t function_value);

void gpio_configure_interrupt(uint16_t pin_no, edge_selector edge_select);

void gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no);

void gpio_clear_interrupt(uint16_t pin_no);



#endif