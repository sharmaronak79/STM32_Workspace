/*
 * stm32l496_gpio_driver.c
 *
 *  Created on: Oct 3, 2022
 *      Author: Ronakkumar_Sharma
 */

#include"stm32l496_gpio_driver.h"


/************************************Driver Code for APIs************************************************************/

/* Peripheral Clock setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}






/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	//1. Configure the Mode of the gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		//In Non Interrupt Mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<((2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );
		pGPIOHandle->pGPIOx->MODER |= temp;

	}else{

		//Interrupt Mode

	}

	//2. Configure the speed
	temp=0;
	temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<((2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;


	//3. COnfigure the PuPd setting
	temp = 0;
	temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 <<((2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	//4. COnfigure the O/P type, Open Drain or Push Pull
	temp = 0;
	temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 <<( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. COnfigurethe Alternate Functionality
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN_NO_7){
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode) << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFRL = temp;
		}else{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode) << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFRH = temp;
		}
	}


}



void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

}
















