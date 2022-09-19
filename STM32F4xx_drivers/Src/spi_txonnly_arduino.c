/*
 * spi_txonly_arduino.c
 *
 *  Created on: Sep 16, 2022
 *      Author: ronakkumar_sharma
 */


/*
 * main.c
 *
 *  Created on: Sep 13, 2022
 *      Author: Ronakkumar_Sharma
 */


#include<stdio.h>
#include "stm32f407xx.h"
#include<string.h>


void delay(void){
	for(uint32_t i = 0; i < 500000/2 ; i++);
}

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
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_CLK_SPEED_DIV8;    //Generates Sclk of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;// Hardware slave management enable for NSS Pin, Because we are using Arduino as A Hardware


	SPI_Init(&SPI2Handle);

}


void GPIO_ButtonInit(void){


	GPIO_Handle_t GpioBtn;

	//this is Btn gpio configuration
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0 ;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	GPIO_Init(&GpioBtn);
}

int main(){

	 char user_data[] = "Hello World";

	 GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 Pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * making SSOE 1 does  NSS output Enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE = 1, NSS will be pulled to LOW
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){

			//Wait till button is pressed
			while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			//Enable the SPI2 Peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			//first send length information
			uint8_t datalen = strlen(user_data);
			SPI_SendData(SPI2,&datalen,1);


			//to send data
			SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

			//Before closing or Disable SPI Peripheral , check that is it busy or not
			// for that we will check the status register , there is one bit BSY,
			//BSY = 0; SPI not Busy
			//BSY = 1; SPI is busy in communication

			while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

			//After the successful transmission of byte we have to Disable the SPI2 Peripheral
			SPI_PeripheralControl(SPI2, DISABLE);

	}


	return 0;

}
