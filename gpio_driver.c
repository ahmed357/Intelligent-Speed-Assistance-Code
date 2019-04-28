/****************************************************
* File:gpio_driver.c
* This is the GPIO source file for STM32F401 Arm MCU
* 
* Part of ISA using Wireless Ad-hoc Networks Project
* Abdullah Ahmed
* City, University of London
*////////////////////////////////////////////////////

#include "gpio_driver.h"
#include "stm32f401xe.h"

/* The following was obtained by referring to the GPIO registers reference manual on page 158 */


 /*
 * Configures the mode of a GPIO pin 
 *
 * @param *GPIOx this is the GPIO port base address
 * @param pin_no this is the GPIO pin number
 * @param mode this is the mode that is to be set
 */
static void gpio_configure_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t mode) {

	GPIOx->MODER |= (mode << (2*pin_no));
}

 /*
 * Configures the output type of a GPIO pin 
 *
 * @param *GPIOx this is the GPIO port base address
 * @param pin_no this is the GPIO pin number
 * @param outputtype this is the output type that is to be set
 */
static void gpio_configure_outputtype(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t outputtype) {

	GPIOx->OTYPER |= (outputtype << (pin_no));
}

 /*
 * Configures the speed of a GPIO pin 
 *
 * @param *GPIOx this is the GPIO port base address
 * @param pin_no this is the GPIO pin number
 * @param speed this is the speed of the GPIO
 */
static void gpio_configure_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t speed) {

	GPIOx->OSPEEDR |= (speed << (2*pin_no));
}

 /*
 * Configures the resistor settings of a GPIO pin 
 *
 * @param *GPIOx this is the GPIO port base address
 * @param pin_no this is the GPIO pin number
 * @param pupd this is the resistor settings to be configured
 */
static void gpio_configure_pull_up_down(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pupd) {

	GPIOx->PUPDR |= (pupd << (2*pin_no));
}

 /*
 * Configures the alternate functionality of a GPIO pin 
 *
 * @param *GPIOx this is the GPIO port base address
 * @param pin_no this is the GPIO pin number
 * @param alt_function this is the alternate functionality to be configured
 */
static void gpio_configure_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint16_t alt_function) {

	if(pin_no <=7) {
		GPIOx->AFR[0] |= (alt_function << (pin_no*4));
	}
	else {
		GPIOx->AFR[1] |= (alt_function << ((pin_no % 8)*4));
	}
}

 /*
 * Reads an input from a GPIO pin 
 *
 * @param *GPIOx this is the GPIO port base address
 * @param pin_no this is the GPIO pin number
 * @return this returns a value
 */
uint8_t gpio_read(GPIO_TypeDef *GPIOx, uint16_t pin_no) {

	uint8_t value;
	value = ((GPIOx->IDR >> pin_no) & 0x00000001);
	return value;
}

 /*
 * Writes a value to a GPIO pin
 *
 * @param *GPIOx this is the GPIO port base address
 * @param pin_no this is the GPIO pin number
 * @param
 */
void gpio_write(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val) {

	if(val)
		GPIOx->ODR |= (1 << pin_no);
	else
		GPIOx->ODR &= ~(1 << pin_no);
}

 /*
 * Configures the interrupt settings of a GPIO pin 
 *
 * @param pin_no this is the GPIO pin number
 * @param edge_select selects the edge detection settings
 */
void gpio_configure_interrupt(uint16_t pin_no, edge_selector edge_select) {

	if (edge_select == RISING_EDGE) {
		EXTI->RTSR |= (1 << pin_no);
	}
	else if (edge_select == FALLING_EDGE) {
		EXTI->FTSR |= (1 << pin_no);
	}
	else if (edge_select == RISING_FALLING_EDGE) {
		EXTI->RTSR |= (1 << pin_no);
		EXTI->FTSR |= (1 << pin_no);
	}
	else {
		;
	}
}

 /*
 * Enables interrupts on a GPIO pin 
 *
 * @param pin_no this is the GPIO pin number
 * @param irq_no this is the IRQ number. Refer to datasheet
 */
void gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no) {

	EXTI->IMR |= (1 << pin_no);
	NVIC_EnableIRQ(irq_no);

}

 /*
 * Initialises IRQ handler
 *
 * @param PinNumber this is the pin number for IRQ handling
 */
void gpio_irq_handling(uint8_t PinNumber)
{
	if(EXTI->PR & ( 1 << PinNumber))
	{
		EXTI->PR |= ( 1 << PinNumber);
	}

}

	

