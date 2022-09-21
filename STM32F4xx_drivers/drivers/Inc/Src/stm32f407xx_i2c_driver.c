/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 20, 2022
 *      Author: ronakkumar_sharma
 */


#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScalar[8] = {2,4,8,16,32,64,128,256};
uint8_t APB1_PreScalar[4] = {2,4,8,16};

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - Enable the clock for particular I2C peripheral
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}

	}else{
		if(pI2Cx == I2C1){
				I2C1_PCLK_DI();
			}else if(pI2Cx == I2C2){
				I2C2_PCLK_DI();
			}else if(pI2Cx == I2C3){
				I2C3_PCLK_DI();
			}
	}

}


uint8_t RCC_GetPLLOutputClock(){

	return 0;
}


uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1 , SystemClk;
	uint8_t clksrc , temp, ahbp , apb1;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){

		SystemClk = 16000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	//For AHB1 prescalar
	temp = ((RCC -> CFGR >> 4) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScalar[temp - 8];
	}


	//For APB1 prescalar
	temp = ((RCC -> CFGR >> 10) & 0x7);

	if(temp < 4){
		apb1 = 1;
	}else{
		ahbp = APB1_PreScalar[temp - 4];
	}


	pclk1 = (SystemClk/ahbp) / apb1;

	return pclk1;

}

/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - Initialize the I2C peripheral
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

   //program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations, Clock Control Register
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}


/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - Reset the peripheral I2C, by reset the register value in
 * 						APB1RSTR , here we have defined the macro for the same
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){

	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}



/*********************************************************************
 * @fn      		  - I2C_Peripheral CObntrol
 *
 * @brief             - Enable that particular peripheral , like I2C 1/2/3
 * 						Like here the PE- Peripheral Enable Bit will be 0 or 1 for Disable and Enable
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){

		pI2Cx ->CR1 |= (1<<0);
	}else{
		pI2Cx ->CR1 &= ~(1<<0);
	}
}




