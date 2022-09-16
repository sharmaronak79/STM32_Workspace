/*
 * main.c
 *
 *  Created on: Sep 13, 2022
 *      Author: Ronakkumar_Sharma
 */


#include<stdio.h>
#include "stm32f407xx.h"
#include<string.h>


/*
 * PB12 - SPI2 NSS
 * PB13 - SPI2 SCK
 * PB14 - SPI2 MISO
 * PB15 - SPI2 MOSI
 * ALT Function Mode - 5
 */

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins); // pass the address of GPIO handle

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void){
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_CLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;    // software slave management enable for NSS Pin


	SPI_Init(&SPI2Handle);


}

int main(){

	 char user_data[] = "Hello World";

	//this function is used to initialize the GPIO pins to behave as SPI2 Pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	// make SSI Enable to connect NSS to +VCC and avoid MODF Error
	SPI_SSIConfig(SPI2,ENABLE);

	//Enable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//to send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//After the successful transmission of byte we have to Disable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2, DISABLE);


	while(1);    // Infinite while loop to hang the application


	return 0;

}
