/****************************************************
* File:router_RTOS.c
* This is the software to run on the OBUs in the system
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
#include "stm32_timers.h"
#include <xbee.h>

#define LOW_SPEED 1000
#define MEDIUM_SPEED 2000
#define HIGH_SPEED 3000

int pulse = 0;
int pwm_change = 0;
int limit = 0;

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
 * This thread constantly monitors the speed of the motors
 * and prints the data on the LCD.
 *
 * @return frequency returns the frequency of the motors in rpm.
 */
float monitorSpeedThread(void) {

	volatile uint8_t captureDone = 0;
	uint16_t captures[2];
	uint16_t diffCapture = 0;
	while(1) {
		if (captureDone != 0) {
			if(captures[1] >= captures[0]) {
				diffCapture = captures[1]-captures[0];
			}
			else {
				;
			}
		}
		else {
			;
		}
		frequency = (float) (frequency /diffCapture)*60;
		if (frequency => 30){
			return frequency;
		}
		else {
			;
		}
		HD_44780_Puts(1,0,"speed:%f", frequency);
		osDelay(500);
	}
	osThreadTerminate(NULL);
}

 /*
 * This thread reduces the speed of the motors if
 * the speed is faster than the limit. Automatically
 * reduces until the limit is reached.
 */
void reduceSpeedThread(void) {
	while(1) {
		if(limit = 30 ) {
			while(frequency>LOW_SPEED) {
				pwm_change = 0;
				pwm_change = pwm_change -5;
			}
		}
		else if (limit = 60 ) {
			while(frequency>MEDIUM_SPEED) {
				pwm_change = 0;
				pwm_change = pwm_change -5;
			}
		}
		else if (limit = 90 ) {
			while(frequency>HIGH_SPEED) {
				pwm_change = 0;
				pwm_change = pwm_change -5;
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
 * This thread provides access to motor control functionality
 * Essentially pwm is changed upon button press.
 */
void motorControlThread(void) {
	while(1) {
		if((gpio_read(GPIOA,2))) {
			if(pulse <= limit) {
				pwm_change = pwm_change +5;
			}
			else {
				;
			}
		}
		else if((gpio_read(GPIOA,3))) {
			if(pulse <= limit) {
				pwm_change = 0;
				pwm_change = pwm_change -5;
			}
			else {
				;
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
 * This thread transmits data upon button press. The data being
 * transmitted is the speed of vehicle, and is also displayed on LCD.
 */
void transmitDataThread(void) {

	while(1) {
		if ((ret = xbee.send_data_broadcast(speed,sizeof(speed))) != XBEE_ENONE){
			xbee_log(xbee, "send_data_broadcast failed to send", ret);
		}
		else {
			xbee_log(xbee, "Packet sent successfully.");
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
  	uint16_t i = 0;
  	uint16_t car2 = 0;
	uint8_t waste = 0;
	while (1) {
		osSemaphoreWait(full);
		osSemaphoreWait(mutex);
		addr_high = (*pkt)->data[11];
		addr_low = (*pkt)->data[12];
		osSemaphoreRelease(mutex);
		osSemaphoreRelease(empty);
		sourceaddress = concatenate(addr_high, addr_low);

		/*Receive data and parsing*/
		if (sourceaddress == 0x000) {
			limit = data[17];
			HD_44780_Puts(0,0,"Limit:%d", limit);
		}
		else if (sourceaddress == 0xAB02) {

			car2 = data[17];
			HD_44780_Puts(0,8,"c2:%d", car2);
		}
		else {
			for (i=0;i<x;i++) {
				waste = data[i];
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
 * Simple function used to initialise the GPIOs for button press
 * Refer to the GPIO drivers for detailed information.
 */
void button_init(void) {

	gpio_handler gpiobutton1, gpiobutton2;

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

	GPIOA_CLK_ENABLE();
}

 /*
 * Simple function used to initialise timers.
 */
void timer_init(void) {
	timer_handler timer;
	timer.timer_prescaler = 4000;
	timer.timer_counter_mode = CounterMode_Up;
	timer.timer_period = 500;
	timer.timer_clock_division = TIM_DIV1;
	timer_init(&timer);
	GPIOA_CLK_ENABLE();
}

 /*
 * Simple function used to initialise PWM.
 */
void pwm_init(pulse) {
	timer_handler pwm;
	pwm.mode = TIM_PWM1;
	pwm.pulse = pulse + pwm_change;
	pwm.output = TIM_OUTPUT_ENABLE;
	pwm.poalrity = TIM_POLARITY_HIGH;

	pwm_init(&pwm);
	gpio_configure_alt_function(GPIOA,12,GPIO_ALT_FUN_MODE);
}

 /*
 * This is a simple function which initialises the interrupts.
 * Refer to the GPIO drivers for detailed information.
 */
void EXTI_IRQHandler(void) {

	GPIO_IRQHandling(5);//pin name
	GPIO_ToggleOutputPin(GPIOA,2);
	GPIO_ToggleOutputPin(GPIOA,3);
	GPIO_ToggleOutputPin(GPIOA,12);
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
	osThreadID monitorSpeedTID;
	osThreadID motorControlTID;
	osThreadID reduceSpeedTID;

	osSemaphoreDef(full);
	fullid = osSemaphoreCreate(osSemaphore(full), sizeof(pkt));

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

	osThreadDef(monitorspeed, monitorSpeedThread, osPriorityHigh, 0, 100);
	monitorSpeedTID = osThreadCreate(osThread(monitorspeed), NULL);

	osThreadDef(motorcontrol, motorControlThread, osPriorityHigh, 0, 100);
	motorControlTID = osThreadCreate(osThread(motorcontrol), NULL);

	osThreadDef(reducespeed, reduceSpeedThread, osPriorityNormal, 0, 100);
	reduceSpeedTID = osThreadCreate(osThread(reducespeed), NULL);
  /* Starts the kernel */
	osKernelStart();

  /* Initialising LCD Display */
	HD_44780_init();

  /* Initialising buttons */
	button_init();

  /* Initialising timer */
	timer_init();

  /* Initialising PWM */
	pwm_init();

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
	*  via its 16-bit address 0x000
	*/
	memset(&address1, 0, sizeof(address1));
	address1.addr16_enabled = 1;
	address1.addr16[0] = 0x00;
	address1.addr16[1] = 0x00;

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


