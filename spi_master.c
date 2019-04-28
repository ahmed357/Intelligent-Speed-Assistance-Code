/****************************************************
* File:spi_master.c
* This is the SPI file transmission for STM32F401 Arm MCU
* 
* Part of ISA using Wireless Ad-hoc Networks Project
* Abdullah Ahmed
* City, University of London
*////////////////////////////////////////////////////

#include <stdint.h>
#include "stm32f401xe.h"
#include "spi_driver.h"
#include "gpio_driver.h"
#include "led.h"

spi_handle_t spihandle;

void spi_gpio_init(void) {
	
	gpio_handler spi_config;
	
	GPIOA_CLK_ENABLE();

	spi_config.GPIOx = GPIOA;
	spi_config.gpio_config.mode = GPIO_ALT_FUN_MODE;
	spi_config.gpio_config.alternate = 5;
	spi_config.gpio_config.outputtype = GPIO_OUTPUT_TYPE_PUSHPULL;
	spi_config.gpio_config.speed = GPIO_SPEED_HIGH;

	/*MISO*/
	spi_config.gpio_config.pin = 6;
	gpio_init(&spi_config);
	/*MOSI*/
	spi_config.gpio_config.pin = 7;
	gpio_init(&spi_config);
	/*SCLK*/
	spi_config.gpio_config.pin = 5;
	gpio_init(&spi_config);
	/*NSS*/
	spi_config.gpio_config.pin = 4;
	gpio_init(&spi_config);
	
}

void spi_spi_init(void) {

	spi_handle_t spi_handle;

	SPI1_CLK_ENABLE();

	spi_handle.SPIx = SPI1;
	spi_handle.Init.Direction = BIDERECTIONAL;
	spi_handle.Init.Mode = SPI_MSTR_MASTER ;
	spi_handle.Init.BaudRatePrescaler = SPI_BR_2;
	spi_handle.Init.DataSize = SPI_8_BIT;
	spi_handle.Init.CLKPolarity = SPI_CPOL_LOW;
	spi_handle.Init.CLKPhase = SPI_CPHA_LOW; 
	spi_handle.Init.NSS = SPI_SSM_ENABLE;
	spi_handle.Init.FirstBit = SPI_MSB;

}

void button_led_init(void) {

	gpio_handler gpiobutton, gpioled;

	/*button initialisation*/
	gpiobutton.GPIOx = GPIOA;
	gpiobutton.gpio_config.pin = 0;
	gpiobutton.gpio_config.mode = GPIO_INPUT_MODE;
	gpiobutton.gpio_config.speed = GPIO_SPEED_HIGH;
	gpiobutton.gpio_config.pull = GPIO_NO_PULL;
	gpio_init(&gpiobutton)

	/*led initialisation*/
	gpioled.GPIOx = GPIOD;
	gpioled.gpio_config.pin = 12;
	gpioled.gpio_config.mode =  GPIO_OUTPUT_MODE;
	gpioled.gpio_config.speed = GPIO_SPEED_HIGH;
	gpioled.gpio_config.pull = GPIO_NO_PULL;

	GPIOD_CLK_ENABLE();

}

int main(void) {

	char user_data[] = "Hello World";
	uint8_t dummy = 0xFF;
	
	spi_gpio_init();

	spi_spi_init();
	
	button_led_init();
	
	gpio_configure_interrupt(GPIO_BUTTON, FALLING_EDGE);
	gpio_enable_interrupt(GPIO_BUTTON, EXTI0_IRQn);

	spi_enable(SPI1);

	/*waits till the button is pressed*/
	while(!(gpio_read(GPIOA,0)));

	while(1) {

		uint8_t commandcode = LED_ON;
		uint8_t ackbyte;

		/*Send some dummy bits*/
		spi_transmit(SPI1,&dummy,1);

		/*read the ack byte received*/
		spi_receive(SPI1,&ackbyte,1);

		if( spi_verify(ackbyte)) {
			/*send arguments*/
			spi_transmit(SPI1,&commandcode,1);
		}
	}
	
}

uint8_t spi_verify(uint8_t ackbyte) {

	if(ackbyte == (uint8_t)0xF5)
	{
		/*ack*/
		return 1;
	}
	else {
		return 0;
	}
}