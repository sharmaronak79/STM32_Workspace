/*
 * i2c_master_Rx_Testing_IT.c
 *
 *  Created on: Sep 28, 2022
 *      Author: ronakkumar_sharma
 */



#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles();

#define MY_ADDR     0x61     // Refer the specification because we cannot use some specific address
#define SLAVE_ADDR  0x68	 // Arduino Address , we can get it from the Arduino sketch

//Flag Variable
uint8_t rxComplt = RESET;


//Rcv Data Buffer
uint8_t rcv_buf[32] ;


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */

I2C_Handle_t I2C1Handle;


void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4 ;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{

	I2C1Handle.pI2Cx= I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR; // this is needed when we treat our device as SLAVE
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

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

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

}


int main(void)
{

	uint8_t commandcode;

	uint8_t len;

	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//I2C IRQ Configurations,    we have to enable IRQ number to make connection/configurations between NVIC and controller
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );
		printf("Butteon is pressed \n");

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = 0x51;

		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); // here, SR_ENABLE is a macro to enable the Repeated the start

		while(I2C_MasterReceiveDataIT(&I2C1Handle,&len,1,SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);
		printf("Received Length of the data is : %x\n",len);
		rxComplt = RESET;


		commandcode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle,rcv_buf,len,SLAVE_ADDR,I2C_DISABLE_SR) != I2C_READY); // Disable SR to end this transaction

		while(rxComplt != SET);

		rcv_buf[len+1] = '\0';
		printf("Data : %s",rcv_buf);

		rxComplt = RESET;
	}

}



// Here we implement the IRQ handler by finding the name of them from start up file
//So, during the transmission when any Event Interrupt happen then our I2C_EV_IRQHandling API will be called and same for the Error Interrupt

void I2C1_EV_IRQHandler(void){

	I2C_EV_IRQHandling(&I2C1Handle);

}


void I2C1_ER_IRQHandler(void){

	I2C_ER_IRQHandling(&I2C1Handle);

}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv){


	if(AppEv == I2C_EV_TX_CMPLT){

		printf("Tx is completed \n");

	}else if(AppEv == I2C_EV_RX_CMPLT){

		printf("Rx is Completed \n");
		rxComplt = SET;

	}else if(AppEv == I2C_ERROR_AF){

		printf("Error : Ack Failure\n");
		//in master ack failure happens when slave fails to send ack for the byte sent from the master

		//so , here we have to close the communication, in this API we have reset or disable that Interrupt register
		I2C_CloseSendData(pI2CHandle);

		//generate STOP condition to release the Bus
		I2C_GenerateStopCondition(I2C1);

		//Hang in here in infinite loop
		while(1);
	}


}



