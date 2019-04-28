/****************************************************
* File:spi_driver.c
* This is the SPI source file for STM32F401 Arm MCU
* 
* Part of ISA using Wireless Ad-hoc Networks Project
* Abdullah Ahmed
* City, University of London
*////////////////////////////////////////////////////

#include <stdint.h>
#include "spi_driver.h"

#define RESET 0
#define SET !RESET


 /*
 * Configures whether the device is a master or a slave
 *
 * @param *SPIx this is the bases address of the SPI
 * @param master select whethere device is master/slave
 */
static void spi_configure_mode(SPI_TypeDef *SPIx, uint32_t master) {

	if(master) {
		SPIx->CR1 |= SPI_MSTR;
	}
	else {
		SPIx->CR1 &= ~SPI_MSTR;
	}
}

 /*
 * Configures the phase and polarity of the SPI
 *
 * @param *SPIx this is the bases address of the SPI
 * @param phase this is the phase value
 * @param polarity this is the polarity value
 */
static void spi_configure_phase_polarity(SPI_TypeDef *SPIx, uint32_t phase, uint32_t polarity) {

	if(phase) {
		SPIx->CR1 |= SPI_CPHA;
	}
	else {
		SPIx->CR1 &= ~SPI_CPHA;
	}

	if(polarity) {
		SPIx->CR1 |= SPI_CPOL;
	}
	else {
		SPIx->CR1 &= ~SPI_CPOL;
	}
}

 /*
 * Configres the datasize and direction of the SPI
 *
 * @param *SPIx this is the bases address of the SPI
 * @param datasize whether it is 8 or 16 bit
 * @param lsbfirst LSB or MSB first
 */
static void spi_configure_datasize_direction(SPI_TypeDef *SPIx, uint32_t datasize, uint32_t lsbfirst) {

	if(datasize) {
		SPIx->CR1 |= SPI_DFF;
	}
	else {
		SPIx->CR1 &= ~SPI_DFF;
	}

	if(lsbfirst) {
		SPIx->CR1 |= SPI_LSBFIRST;
	}
	else {
		SPIx->CR1 &= ~SPI_LSBFIRST;
	}
}

 /*
 * Configures the software select settings of the master
 *
 * @param *SPIx this is the bases address of the SPI
 * @param ssm this is the ssm setting
 */
static void spi_configure_nss_master(SPI_TypeDef *SPIx, uint32_t ssm) {

	if(ssm) {
		SPIx->CR1 |= SPI_SSM;
		SPIx->CR1 |= SPI_SSI;
	}
	else{
		SPIx->CR1 &= ~SPI_SSM;
	}
}

 /*
 * Configures the software select settings of the slave
 *
 * @param *SPIx this is the bases address of the SPI
 * @param ssm this is the ssm setting
 */
static void spi_configure_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm) {

	if(ssm) {
		SPIx->CR1 |= SPI_SSM;
	}
	else {
		SPIx->CR1 &= ~SPI_SSM;
	}
}

 /*
 * Configures the baudrate of the device
 *
 * @param *SPIx this is the bases address of the SPI
 * @param value this is the value of the baudrate
 */
void spi_configure_baudrate(SPI_TypeDef *SPIx, uint32_t value) {
	
	if(value > 7) {
		SPIx->CR1 |= (0x00 << 3);
	}
	else {
		SPIx->CR1 |= (value << 3);
	}
}

 /*
 * Configures the direction of the SPI device
 *
 * @param *SPIx this is the bases address of the SPI
 * @param dirction whether it is full duplex or half duplex
 */
void spi_configure_direction(SPI_TypeDef *SPIx, uint32_t direction) {
	
	if(direction) {
		SPIx->CR1 |= SPI_BIDIMODE;
	}
	else {
		SPIx->CR1 &= ~SPI_BIDIMODE;
	}
}

 /*
 * Initialises the SPI device
 *
 * @param *spi_handle this is the instance to the handler
 */
void spi_init(spi_handle_t *spi_handle) {
	
	spi_configure_phase_polarity(spi_handle->Instance, spi_handle->Init.CLKPhase, spi_handle->Init.CLKPolarity);
	spi_configure_mode(spi_handle->Instance, spi_handle->Init.Mode);
	spi_configure_datasize_direction(spi_handle->Instance, spi_handle->Init.DataSize, spi_handle->Init.FirstBit);
	if(spi_handle->Init.Mode == SPI_MSTR_MASTER) {
		spi_configure_nss_master(spi_handle->Instance,spi_handle->Init.NSS);
	}
	else {
		spi_configure_nss_slave(spi_handle->Instance,spi_handle->Init.NSS);
	}
}

 /*
 * Enables an SPI peripheral
 *
 * @param *SPIx this is the bases address of the SPI
 */
static void spi_enable(SPI_TypeDef *SPIx) {

	if( !(SPIx->CR1 & SPI_ENABLE) )
		SPIx->CR1 |= SPI_ENABLE;
	else {
		;
	}
}

 /*
 * Disables an SPI peripheral
 *
 * @param *SPIx this is the bases address of the SPI
 */
static void spi_disable(SPI_TypeDef *SPIx) {

	SPIx->CR1 &= ~SPI_ENABLE;
}

 /*
 * Determines whether the SPI is busy
 *
 * @param *SPIx this is the bases address of the SPI
 * @return returns a flag indicating whether it is busy
 */
uint8_t spi_is_busy(SPI_TypeDef *SPIx) {

	if(SPIx->SR & SPI_FLAG_BSY) {
		return SPI_IS_BUSY;
	}
	else {
		return SPI_IS_NOT_BUSY;
	}
}

/*
* Transmits SPI data
*
* @param *SPIx base address of the SPI peripheral
* @param *pTxBuffer Pointer to the data to be transferred
* @param Len Number of bytes to transmit
*/

void spi_transmit(spi_handle_t *SPIx, uint8_t *pTxBuffer, uint32_t Len) {

	while(Len>0) {
		/*waits until the TX buffer is set*/
		while(!(SPIx->SR & SPI_FLAG_TXE));
		/*Checks DFF bit*/
		if(SPIx->CR1 & (SPI_8_BIT << SPI_DFF)) {
			SPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
		else {
			SPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			pTxBuffer++;
			pTxBuffer++;
		}
	}

}

/*
* Receives SPI data
*
* @param *SPIx base address of the SPI peripheral
* @param *pRxBuffer Pointer to the data to be received
* @param Len Number of bytes to transmit
*/
void spi_receive(spi_handle_t *SPIx, uint8_t *pRxBuffer, uint32_t Len) {

	while(Len>0) {
		/*waits until the RX buffer is set*/
		while(!(SPIx->SR & SPI_FLAG_RXNE));
		/*Checks DFF bit*/
		if(SPIx->CR1 & (SPI_8_BIT << SPI_DFF)) {
			SPIx->DR = *(pRxBuffer);
			Len--;
			pRxBuffer++;
		}
		else {
			SPIx->DR = *((uint16_t*)pRxBuffer);
			Len--;
			Len--;
			pRxBuffer++;
		}
	}

}


