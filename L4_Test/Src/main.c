/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#define AHB2ENR			0x4002104cU

void delay(int n){
		for(uint32_t i =0;i<n;i++);
	}



int main(void)
{
    /* Loop forever */
	uint32_t *pAHB2ENR = (uint32_t *)0x4002104C;
	//uint32_t *pGPIOB = (uint32_t *)0x48000400;
	uint32_t *pGPIOB_Mode = (uint32_t *)0x48000400;
	uint32_t *pGPIOB_OTYPER = (uint32_t *)0x48000404;
	uint32_t *pGPIOB_OSPEEDR = (uint32_t *)0x48000408;
	//uint32_t *pGPIOB_PUPDR = (uint32_t *)0x4800040C;
	//uint32_t *pGPIOB_IDR = (uint32_t *)0x48000410;
	uint32_t *pGPIOB_ODR = (uint32_t *)0x48000414;
	//uint32_t *pGPIOB_BSRR = (uint32_t *)0x48000418;


	*pAHB2ENR |= (1<<1);
	*pGPIOB_Mode &= ~(1<<15); //output mode pin 7
	*pGPIOB_Mode &= ~(1<<29); //output mode pin 14
	//*pGPIOB_Mode = 0xdfff7fff;
	//*pGPIOB_OTYPER = 0x4080; // Pin7 and 14 as open drain
	*pGPIOB_OSPEEDR |= (1<<14); //pin7 medium speed
	*pGPIOB_OSPEEDR |= (1<<28); //pin 14 medium speed

	while(1){
	//*pGPIOB_BSRR |= (1<<7); // set pin7
	//*pGPIOB_BSRR |= (1<<14); //set pin14

	*pGPIOB_ODR = 0xffff;
	delay(500000);

	//*pGPIOB_BSRR |= (1<<23); // Reset pin7
	//*pGPIOB_BSRR |= (1<<30); //Reset pin14
	*pGPIOB_ODR = 0x0;
	delay(500000);

	}



}