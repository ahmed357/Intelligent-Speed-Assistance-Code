/****************************************************
* File:coordinator_RTOS.c
* This is the software to run on the RSU in the system
* utilising the Zigbee communication protocol. Software
* is written using RTOS tools
*
* Part of ISA using Wireless Ad-hoc Networks Project
* Abdullah Ahmed
* City, University of London
*////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32_lcd.h"
#include "stm32f401xe.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "list.h"
#include "queue.h"
#include "tasks.h"
#include <xbee.h>

#define LOW_SPEED 30
#define MEDIUM_SPEED 60
#define HIGH_SPEED 90

 /*
 * This thread constantly receives data by fetching from the buffer.
 * It will only execute if there is data in the buffer.
 *
 * @param xbee provides a handle to this instance.
 * @param con provides a handle to the connection.
 * @param pkt provides access to the data received.
 * @param data is the body of the packet.
 */
void receiveDataThread(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data) {
	while(1) {
		if ((*pkt)->dataLen > 0) {
			osSemaphoreWait(empty);
			osSemaphoreWait(mutex);
			xbee_conCallbackSet(con, NULL, NULL);
			osSemaphoreRelease(mutex);
			osSemaphoreRelease(full);
			printf("Received data: %s\n", (*pkt)->data);
		}
		else {
			;
		}
		osDelay(500);
	}
	osThreadTerminate(NULL);
}

 /*
 * This thread parses the data received to obtain both the source
 * address of the packet and the data within the frame. The speed
 * of the vehicles is also appropriately displayed on the LCD.
 */
void parseDataThread(void) {

	uint16_t addr_high = 0;
	uint16_t addr_low = 0;
	uint16_t sourceaddress = 0;
	uint8_t waste = 0;
  	uint8_t i = 0;

	while (1) {
		osSemaphoreWait(full);
		osSemaphoreWait(mutex);
		addr_high = (*pkt)->data[11];
		addr_low = (*pkt)->data[12];
		osSemaphoreRelease(mutex);
		osSemaphoreRelease(empty);
		sourceaddress = concatenate(addr_high, addr_low);

		/*Receive data and parsing*/
		if (sourceaddress == 0xAB01) {
			car1 = data[17];
			HD_44780_Puts(1,0,"c1: %f",car1);
		}
		else if (sourceaddress == 0xAB02) {
			car2 = data[17];
			HD_44780_Puts(1,8,"c2: %f",car2);
		}
		else {
			for (i=0;i<x;i++) {
				waste = data[i];
			}
			else {
				;
			}
		}
		/*Continuously receives frames*/
		while(1) {
			xbee.process_rx_frames();
			delay(100);
		}
		osDelay(500);
	}
	osThreadTerminate(NULL);
}

 /*
 * This thread transmits data upon button press. The data being
 * transmitted is the speed limit, and is also displayed on LCD.
 */
void transmitDataThread(void) {
	const uint32_t Data1 = LOW_SPEED ;
	const uint32_t Data2 = MEDIUM_SPEED;
	const uint32_t Data3 = HIGH_SPEED;
	while(1) {
		if((gpio_read(GPIOA,2))) {
		  
			
			if ((ret = xbee.send_data_broadcast(Data1,1)) != XBEE_ENONE){
				xbee_log(xbee, "send_data_broadcast failed to send", ret);
			}
			else {
				xbee_log(xbee, "Packet sent successfully.");
				HD_44780_Puts(0,0,"Speed Limit: %d", LOW_SPEED);
			}
		}
		else if((gpio_read(GPIOA,3))) {
		  
			if ((ret = xbee.send_data_broadcast(Data2,1)) != XBEE_ENONE){
				xbee_log(xbee, "send_data_broadcast failed to send", ret);
			}
			else {
				xbee_log(xbee, "Packet sent successfully.");
				HD_44780_Puts(0,0,"Speed Limit: %d", MEDIUM_SPEED);
			}
		}
		else if((gpio_read(GPIOA,4))) {
		  
			if ((ret = xbee.send_data_broadcast(Data3,1)) != XBEE_ENONE){
				xbee_log(xbee, "send_data_broadcast failed to send", ret);
			}
			else {
				xbee_log(xbee, "Packet sent successfully.");
				HD_44780_Puts(0,0,"Speed Limit: %d", HIGH_SPEED);
			}
		}
		else {
			;
		}
		osDelay(500);
	}
	osThreadTerminate(NULL);
}

 /*
 * This is a simple function that concatenates two numbers.
 * It is used to reconstruct the source address.
 *
 * @param x is any number
 * @param y is any number
 * @return this returns xy
 */
unsigned concatenate(unsigned x, unsigned y) {
	unsigned p = 10;
	while(y >= p) {
		p *=10;
	}
	return x * pow + y;
}

 /*
 * This is a function which initialises the systick which is
 * essential for the running of the RTOS.
 */
void SysTick_Handler(void) {
	IncTick();
	SYSTICK_IRQHandler();
	osSystickHandler();
}

 /*
 * Simple function used to initialise the GPIOs for button press
 * Refer to the GPIO drivers for detailed information.
 */
void button_init(void) {

	gpio_handler gpiobutton1, gpiobutton2, gpiobutton3;

	gpiobutton1.GPIOx = GPIOA;
	gpiobutton1.gpio_config.pin = 2;
	gpiobutton1.gpio_config.mode = GPIO_INPUT_MODE;
	gpiobutton1.gpio_config.speed = GPIO_SPEED_HIGH;
	gpiobutton1.gpio_config.pull = GPIO_NO_PULL;
	gpio_init(&gpiobutton1);

	gpiobutton2.GPIOx = GPIOA;
	gpiobutton2.gpio_config.pin = 3;
	gpiobutton2.gpio_config.mode = GPIO_INPUT_MODE;
	gpiobutton2.gpio_config.speed = GPIO_SPEED_HIGH;
	gpiobutton2.gpio_config.pull = GPIO_NO_PULL;
	gpio_init(&gpiobutton2);

	gpiobutton3.GPIOx = GPIOA;
	gpiobutton3.gpio_config.pin = 4;
	gpiobutton3.gpio_config.mode = GPIO_INPUT_MODE;
	gpiobutton3.gpio_config.speed = GPIO_SPEED_HIGH;
	gpiobutton3.gpio_config.pull = GPIO_NO_PULL;
	gpio_init(&gpiobutton3);

	GPIOA_CLK_ENABLE();
}

 /*
 * This is a simple function which initialises the interrupts.
 * Refer to the GPIO drivers for detailed information.
 */
void EXTI_IRQHandler(void) {

	GPIO_IRQHandling(5);
	GPIO_ToggleOutputPin(GPIOA,2);
	GPIO_ToggleOutputPin(GPIOA,3);
	GPIO_ToggleOutputPin(GPIOA,4);
}

 /*
 * This is the main file. Relevant initialisations are carried out.
 */
int main(void) {

  /* Initialising semaphores */
	osSemaphoreId fullid;
	osSemaphoreId emptyid;
	osSemaphoreId mutexid;

  /* Initialising threads */
	osThreadID receivedataTID;
	osThreadID parsedataTID;
	osThreadID transmitdataTID;

	osSemaphoreDef(full);
	fullid = osSemaphoreCreate(osSemaphore(full), sizeof(*pkt));

	osSemaphoreDef(empty);
	emptyid = osSemaphoreCreate(osSemaphore(empty),0);

	osSemaphoreDef(mutex);
	mutexid = osSemaphoreCreate(osSemaphore(mutex),1);

	osThreadDef(receivedata, receiveDataThread, osPriorityNormal, 0, 100);
	receivedataTID = osThreadCreate(osThread(receivedata), NULL);

	osThreadDef(parsedata, parseDataThread, osPriorityNormal, 0, 100);
	parsedataTID = osThreadCreate(osThread(parsedata), NULL);

	osThreadDef(transmitdata, transmitDataThread, osPriorityHigh, 0, 100);
	transmitdataTID = osThreadCreate(osThread(transmitdata), NULL);

  /* Starts the kernel */
	osKernelStart();

  /* Initialising LCD Display */
	HD_44780_init();

  /* Initialising buttons */
	button_init();

  /* Initialising interrupts */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	struct xbee *xbee;
	struct xbee_con *con;
	struct xbee_pkt *pkt;
	struct xbee_conAddress address1;
	struct xbee_conAddress address2;
	xbee_err ret;

	/* this is the connection to the first node
	*  via its 16-bit address 0xAB01
	*/
	memset(&address1, 0, sizeof(address1));
	address1.addr16_enabled = 1;
	address1.addr16[0] = 0xAB;
	address1.addr16[1] = 0x01;

	/*Making the connection to address1*/
	xbee_conNew(xbee, &con, "Data",address1);

	/* this is the connection to the second node
	*  via its 16-bit address 0xAB02
	*/
	memset(&address2, 0, sizeof(address2));
	address2.addr16_enabled = 1;
	address2.addr16[0] = 0xAB;
	address2.addr16[1] = 0x02;

	/*Making the connection to address2*/
	xbee_conNew(xbee, &con, "Data",address2);

}