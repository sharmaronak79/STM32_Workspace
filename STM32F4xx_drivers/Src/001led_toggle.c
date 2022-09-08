/*
 * 001led_toggle.c
 *
 *  Created on: Sep 8, 2022
 *      Author: Ronakkumar_Sharma
 */

#include"stm32f407xx.h"   //Device specific header file
#include"stm32f407xx_gpio_driver.h"


int main(void){

	void delay(int n){
		for(uint32_t i =0;i<n;i++);
	}

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;





	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

		delay(500000);
	}

	return 0;
}
