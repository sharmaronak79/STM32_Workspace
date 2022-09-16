/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Sep 13, 2022
 *      Author: Ronakkumar_Sharma
 */


#include "stm32f407xx_spi_driver.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}



	}else
	{
		if(pSPIx == SPI1)
				{
					SPI1_PCLK_DI();
				}else if(pSPIx == SPI2)
				{
					SPI2_PCLK_DI();
				}else if(pSPIx == SPI3)
				{
					SPI3_PCLK_DI();
				}else if(pSPIx == SPI4)
				{
					SPI4_PCLK_DI();
				}
	}

}



//SPI Init
void SPI_Init(SPI_Handle_t *pSPIHandle){


	//Peripheral clock ENable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first configure the SPI_CR1 Register, for that we will take one temporary variable
	uint32_t temp = 0;

	//1. Configure the device mode
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR; // Here SPI_CR1_MSTR=2 ,is a bit field of MSTR bit of CR1 register of SPI

	//2. Configure the Bus Config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
		{
			//bidi mode should be cleared
			temp &= ~( 1 << SPI_CR1_BIDIMODE);

		}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
		{
			//bidi mode should be set
			temp |= ( 1 << SPI_CR1_BIDIMODE);
		}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
		{
			//BIDI mode should be cleared
			temp &= ~( 1 << SPI_CR1_BIDIMODE);
			//RXONLY bit must be set
			temp |= ( 1 << SPI_CR1_RXONLY);
		}

	//3. Configure the spi serial clock speed(Baud rate)

	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;

	//4. Configure the DFF

	temp |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL

	temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA

	temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7.Configure the SSM

	temp |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->CR1 = temp;

}


// SPI DeInit
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx==SPI1){

		SPI1_REG_RESET();

	}else if(pSPIx==SPI2){

		SPI2_REG_RESET();

	}else if(pSPIx==SPI3){

		SPI3_REG_RESET();

	}else if(pSPIx==SPI4){

		SPI4_REG_RESET();
	}

}


/*
 *  SPI_GetFlagStatus Function
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


// SPI Send Data

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){
		//1. Wait until TXE is set
		//while( ! (pSPIx -> SR & (1<<1) ) );
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			//16 Bit DFF
			//1. LOad the data in to Data Register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			//8 Bit DFF
			// Load the data in to the Data register
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;

		}
	}

}



void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){

		pSPIx-> CR1 |=(1  << SPI_CR1_SPE);
	}else{
		pSPIx -> CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){

			pSPIx-> CR1 |=(1  << SPI_CR1_SSI);
		}else{
			pSPIx -> CR1 &= ~(1 << SPI_CR1_SSI);
		}
}



