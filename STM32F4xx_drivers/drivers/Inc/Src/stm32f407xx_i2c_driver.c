/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 20, 2022
 *      Author: ronakkumar_sharma
 */


#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScalar[8] = {2,4,8,16,32,64,128,256};
uint8_t APB1_PreScalar[4] = {2,4,8,16};


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlveAddr);
static void I2C_CLearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1 << I2C_CR1_START);

}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //Slave address is slave address + r/nw bit =0
	pI2Cx->DR = SlaveAddr; // Load that address to data register
}

static void I2C_CLearADDRFlag(I2C_RegDef_t *pI2Cx){

	uint32_t dummyRead = pI2Cx ->SR1;
	dummyRead = pI2Cx ->SR2;

	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

}

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
 * @fn      		  - I2C_Peripheral Control
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



/*********************************************************************
 * @fn      		  - I2C_Master SendData
 *
 * @brief             -
 *
 * @param[in]         -here we will need I2C Handle, one buffer where we will keep data to be sent,
 * 					   length of the data and the address of the slave where we want to send the data
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr){

	//1. Generate the start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx); // this is just one helper function

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));


	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )

		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1

		while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));


	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)

		I2C_CLearADDRFlag(pI2CHandle ->pI2Cx);

	//6. send the data until len becomes 0, before sending any data we have to check the DataRegister that is it Empty or not?
	    while(Len > 0){
			 while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // wait till TXE is set
			 pI2CHandle->pI2Cx->DR = *pTxbuffer;
			 *pTxbuffer++;
			 Len--;

		 }


	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF

	    I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);


}




uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName)
		{
			return FLAG_SET;
		}
		return FLAG_RESET;
}





