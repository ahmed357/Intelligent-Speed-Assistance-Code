/****************************************************
* File:spi_driver.h
* This is the SPI header file for STM32F401 Arm MCU
* 
* Part of ISA using Wireless Ad-hoc Networks Project
* Abdullah Ahmed
* City, University of London
*////////////////////////////////////////////////////

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stm32f401xe.h"

/* SPI control register CR1 settings */

/* Biderectional data mode enable */
#define SPI_BIDIMODE                          ( (uint32_t) 1 << 15 )
#define SPI_UNIDERECTIONAL                    0
#define SPI_BIDERECTIONAL                     1

/* Data frame format (either 8-bit or 16-bit) */
#define SPI_DFF                               ( (uint32_t) 1 << 11 )
#define SPI_8_BIT                             0
#define SPI_16_BIT                            1

/* Software slave management */
#define SPI_SSM                               ( (uint32_t) 1 << 9 )
#define SPI_SSM_ENABLE                        1
#define SPI_SSM_DISABLE                       0

/* Internal slave select */
#define SPI_SSI                               ( (uint32_t) 1 << 8 )

/* Frame format (either MSB first or LSB) */
#define SPI_LSBFIRST                          ( (uint32_t) 1 << 7 )
#define SPI_LSB                               1
#define SPI_MSB                               0

/* SPI enable */
#define SPI_ENABLE                            ( (uint32_t) 1 << 6 )

/* SPI Master selection */
#define SPI_MSTR                              ( (uint32_t) 1 << 2 )
#define SPI_MSTR_MASTER                       1
#define SPI_MSTR_SLAVE                        0

/* SPI Clock polarity */
#define SPI_CPOL                              ( (uint32_t) 1 << 1 )
#define SPI_CPOL_LOW                          0
#define SPI_CPOL_HIGH                         1

/* SPI Clock phase */
#define SPI_CPHA                              ( (uint32_t) 1 << 0 )
#define SPI_CPHA_FIRST                        0
#define SPI_CPHA_SECOND                       1

/* SPI Baud rate control */
#define SPI_BR_2                              ( (uint32_t) 0 << 3 )
#define SPI_BR_4                              ( (uint32_t) 1 << 3 )
#define SPI_BR_8                              ( (uint32_t) 2 << 3 )
#define SPI_BR_16                             ( (uint32_t) 3 << 3 )
#define SPI_BR_32                             ( (uint32_t) 4 << 3 )
#define SPI_BR_64                             ( (uint32_t) 5 << 3 )
#define SPI_BR_128                            ( (uint32_t) 6 << 3 )
#define SPI_BR_256                            ( (uint32_t) 7 << 3 )

/* SPI Control register CR2 settings */

/* Tx buffer empty interrupt enable */
#define SPI_TXEIE                             ( (uint32_t) 1 << 7 )

/* Rx buffer not empty interrupt enable */
#define SPI_RXNEIE                            ( (uint32_t) 1 << 6 )

/* Error interrupt enable (generates interrupt when error occurs) */
#define SPI_ERRIE                             ( (uint32_t) 1 << 5 )

/* Frame Format */
#define SPI_FRF                               ( (uint32_t) 1 << 4 )
#define SPI_FRF_MOTOROLA                      0
#define SPI_TI                                1

/* SS Output Enable */
#define SPI_SSOE                              ( (uint32_t) 1 << 2 )

/* SPI status register SR settings */

/* SPI Frame format error */
#define SPI_FLAG_FRE                               ( (uint32_t) 1 << 8 )

/* SPI busy flag */
#define SPI_FLAG_BSY                               ( (uint32_t) 1 << 7 )

/* SPI transmit buffer empty flag */
#define SPI_FLAG_TXE                               ( (uint32_t) 1 << 1 )

/* SPI receive buffer not empty flag */
#define SPI_FLAG_RXNE                              ( (uint32_t) 1 << 0 )

#define SPI_IS_BUSY 1
#define SPI_IS_NOT_BUSY 0

/* Enable clock to SPI peripherals */

#define SPI1_CLK_ENABLE()                       ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_CLK_ENABLE()                       ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_CLK_ENABLE()                       ( RCC->APB1ENR |= (1 << 15) )
#define SPI4_CLK_ENABLE()                       ( RCC->APB2ENR |= (1 << 13) )

/* SPI Data structures */
typedef enum
{
	SPI_STATE_RESET = 0x00, 
	SPI_STATE_READY = 0x01,
	SPI_STATE_BUSY = 0x02,
	SPI_STATE_BUSY_TX = 0x12,
	SPI_STATE_BUSY_RX = 0x22,
	SPI_STATE_BUSY_RX_TX = 0x32,
	SPI_STATE_ERROR = 0x03

}spi_state_t;

typedef struct
{
	uint32_t Mode;
	uint32_t Direction;
	uint32_t DataSize;
	uint32_t CLKPolarity;
	uint32_t CLKPhase;
	uint32_t NSS;
	uint32_t BaudRatePrescaler;
	uint32_t FirstBit;
}spi_init_t;

typedef struct __spi_handle_t
{
	SPI_TypeDef *Instance;
	spi_init_t Init;
	uint8_t *pTxBuffer;
	uint16_t TxSize;
	uint16_t TxCounter;
	uint8_t *pRxBuffer;
	uint16_t RxSize;
	uint16_t RxCounter;
	spi_state_t State;

}spi_handle_t;

/* SPI Driver API's */

void spi_init(spi_handle_t *spi_handle);
void spi_master_tx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len);
void spi_slave_rx(spi_handle_t *spi_handle, uint8_t *receive_buffer, uint32_t len);
void spi_master_rx(spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len);
void spi_slave_tx(spi_handle_t *spi_handle, uint8_t *receive_buffer, uint32_t len);
void spi_irq_handler(spi_handle_t *hspi);



#endif