/*
 * Btn_Interrupt.c
 *
 *  Created on: Sep 12, 2022
 *      Author: Ronakkumar_Sharma
 */

#include"stm32f407xx.h"   //Device specific header file
#include"stm32f407xx_gpio_driver.h"
#include<string.h>





int main(){
	GPIO_Handle_t GpioLed , GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));



	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);



	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0 ;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);



	// we have taken GPIOA pin 0, so , PA0 will send interrupt over the EXTI0
	//so in below IRQ Configuration API we will send IRQ_NO_EXTI_0

	//IRQ Configurations and Priority, Priority can be any value between 0 to 15

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);



	while(1);




}


void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
