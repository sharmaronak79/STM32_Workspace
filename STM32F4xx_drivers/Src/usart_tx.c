/*
 * usart_tx.c
 *
 *  Created on: Oct 5, 2022
 *      Author: Ronakkumar_Sharma
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart3_handle;

void USART3_Init(void)
{
	usart3_handle.pUSARTx = USART3;
	usart3_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart3_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart3_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart3_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart3_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart3_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart3_handle);
}

void 	USART3_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOD;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART3 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_8;
	GPIO_Init(&usart_gpios);

	//USART3 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&usart_gpios);


}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_ButtonInit();



	USART3_GPIOInit();

    USART3_Init();

    USART_PeripheralControl(USART2,ENABLE);

    while(1)
    {
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&usart3_handle,(uint8_t*)msg,strlen(msg));

    }

	return 0;
}
