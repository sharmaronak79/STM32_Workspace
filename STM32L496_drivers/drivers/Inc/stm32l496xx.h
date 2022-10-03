/*
 * stm32l496xx.h
 *
 *  Created on: Sep 2, 2022
 *      Author: ronakkumar_sharma
 */

#ifndef INC_STM32L496XX_H_
#define INC_STM32L496XX_H_


#define __vo volatile



/* Some Generic Macros */

#define ENABLE 				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			1
#define FLAG_RESET    		0




/***************************  CONTROLLER SPECIFIC ADDRESSES  ********************************/

/*Here Some Basic and Required macros are created*/

/* BASE ADDRESS OF FLASH AND SRAM MEMORY */

#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x20040000U
#define ROM_BASEADDR		0x1FFF0000U
#define SRAM 				SRAM1_BASEADDR


/*Base Addresses of APBx and AHBx Bus Peripherals*/

#define PERIPH_BASE 		0X40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x48000000U
#define AHB3PERIPH_BASE		0xA0000000U
#define AHB4PERIPH_BASE		0xA0001000U


/* Base Address of Peripherals hanging on AHB1 Bus*/

#define RCC					0x40021000U

/* Base Address of Peripherals hanging on AHB2 Bus */

#define GPIOA_BASEADDR		0x48000000U
#define GPIOB_BASEADDR		0x48000400U
#define GPIOC_BASEADDR		0x48000800U
#define GPIOD_BASEADDR		0x48000C00U
#define GPIOE_BASEADDR		0x48001000U
#define GPIOF_BASEADDR		0x48001400U
#define GPIOG_BASEADDR		0x48001800U
#define GPIOH_BASEADDR		0x48001C00U
#define GPIOI_BASEADDR		0x48002000U


/* Base Address of Peripherals hanging on APB1 Peripheral Bus*/

#define SPI2_BASEADDR		0x40003800U
#define SPI3_BASEADDR		0x40003C00U

#define USART2_BASEADDR		0x40004400U
#define USART3_BASEADDR		0x40004800U
#define UART4_BASEADDR 		0x40004C00U
#define UART5_BASEADDR 		0x40005000U

#define I2C1_BASEADDR		0x40005400U
#define I2C2_BASEADDR		0x40005800U
#define I2C3_BASEADDR		0x40005C00U
#define I2C4_BASEADDR		0x40008400U


/*Base Address of Peripherals hanging on APB2 Peripheral Bus*/

#define SYSCFG_BASEADDR		0x40010000U
#define EXTI_BASEADDR		0x40010400U
#define SPI1_BASEADDR		0x40013000U
#define USART1_BASEADDR		0x40013800U



/* Peripheral Register Definition Structure for GPIO  */

typedef struct{

	__vo uint32_t MODER;                        /*!< GPIO port mode register,                          Address offset: 0x00     */
	__vo uint32_t OTYPER;                       /*!< GPIO port output type register 				   Address offset: 0x04     */
	__vo uint32_t OSPEEDR;						/*!< GPIO port output speed register  			   	   Address offset: 0x08     */
	__vo uint32_t PUPDR;						/*!< GPIO port pull-up/pull-down register        	   Address offset: 0x0c     */
	__vo uint32_t IDR;							/*!< GPIO port input data register               	   Address offset: 0x10     */
	__vo uint32_t ODR;							/*!< GPIO port output data register              	   Address offset: 0x14     */
	__vo uint32_t BSRR;							/*!< GPIO port bit set/reset register            	   Address offset: 0x18     */
	__vo uint32_t LCKR;							/*!< GPIO port configuration lock register       	   Address offset: 0x1c     */
	__vo uint32_t AFRL;					        /*!< AFR[0] : GPIO alternate function low register     Address offset: 0x20     */
	__vo uint32_t AFRH;							/* GPIO alternate function high register			   Address offset: 0x24     */
	__vo uint32_t BRR;							/* GPIO Port Bit Reset Register*/
	__vo uint32_t ASCR;							/* GPIO Port Analog Switch Control Register */

}GPIO_RegDef_t;



/* Peripheral Register Definition Structure for RCC */

typedef struct{

	__vo uint32_t CR;					// Clock Control Register							OFFSET: 0x00
	__vo uint32_t ICSCR;				// Internal CLock Source Calibration Register		OFFSET: 0x04
	__vo uint32_t CFGR;					// Clock Configuration Register						OFFSET: 0x08
	__vo uint32_t PLLCFGR;				// PLL Configuration Register						OFFSET: 0x0C
	__vo uint32_t PLLSAI1CFGR;			// PLLSAI1 Clock Config. Register					OFFSET: 0x10
	__vo uint32_t PLLSAI2CFGR;			// PLLSAI1 Clock Config. Register					OFFSET: 0x14
	__vo uint32_t CIER;					//Clock Interrupt Enable Register					OFFSET: 0x18
	__vo uint32_t CIFR;					//CLock Interrupt Flag Register						OFFSET: 0x1C
	__vo uint32_t CICR;					//Clock Interrupt Clear Register					OFFSET: 0x20
	     uint32_t RESERVED0;
	__vo uint32_t AHB1RSTR;				// AHB1 Peripheral Reset Register					OFFSET: 0x28
	__vo uint32_t AHB2RSTR;				// AHB2 Peripheral Reset Register					OFFSET: 0x2C
	__vo uint32_t AHB3RSTR;				// AHB3 Peripheral Reset Register					OFFSET: 0x30
	     uint32_t RESERVED1;
	__vo uint32_t APB1RSTR1;			// APB1 Peripheral Reset Register 1 				OFFSET: 0x38
	__vo uint32_t APB1RSTR2;			// APB1 Peripheral Reset Register 2 				OFFSET: 0x3C
	__vo uint32_t APB2RSTR;			   	// APB2 Peripheral Reset Register  					OFFSET: 0x40
		 uint32_t RESERVED2;
    __vo uint32_t AHB1ENR;				// AHB1 Peripheral Clock Enable Register			OFFSET: 0x48
    __vo uint32_t AHB2ENR;				// AHB2 Peripheral Clock Enable Register			OFFSET: 0x4C
    __vo uint32_t AHB3ENR;				// AHB3 Peripheral Clock Enable Register			OFFSET: 0x50
    	 uint32_t RESERVED3;
    __vo uint32_t APB1ENR1;				// APB1 Peripheral Clock Enable Register 1			OFFSET: 0x58
    __vo uint32_t APB1ENR2;				// APB1 Peripheral Clock Enable Register 2			OFFSET: 0x5C
    __vo uint32_t APB2ENR;				// APB2 Peripheral Clock Enable Register			OFFSET: 0x60
         uint32_t RESERVED4;
    __vo uint32_t AHB1SMENR;			// AHB1 Peripheral Clock Enable in Sleep and Stop Mode Register		OFFSET: 0x68
    __vo uint32_t AHB2SMENR;			// AHB2 Peripheral Clock Enable in Sleep and Stop Mode Register		OFFSET: 0x6C
    __vo uint32_t AHB3SMENR;			// AHB3 Peripheral Clock Enable in Sleep and Stop Mode Register		OFFSET: 0x70
    	 uint32_t RESERVED5;
    __vo uint32_t APB1SMENR1;			// APB1 Peripheral Clock Enable in Sleep and Stop Mode Register 1	OFFSET: 0x78
    __vo uint32_t APB1SMENR2;			// APB1 Peripheral Clock Enable in Sleep and Stop Mode Register 2	OFFSET: 0x7C
    __vo uint32_t APB2SMENR;			// APB2 Peripheral Clock Enable in Sleep and Stop Mode Register 	OFFSET: 0x80
         uint32_t RESERVED6;
    __vo uint32_t CCIPR;				// Peripheral Independent Clock Configuration Register				OFFSET: 0x88
         uint32_t RESERVED7;
    __vo uint32_t BDCR ;				// Backup Domain Control  Register									OFFSET: 0x90
    __vo uint32_t CSR;					// Control/Status Register											OFFSET: 0x94
    __vo uint32_t CRRCR;				// Clock Recovery RC Register										OFFSET: 0x98
    __vo uint32_t CCIPR2;				// Peripheral Independent Clock Configuration Register				OFFSET: 0x9C

}RCC_RegDef_t;



#endif /* INC_STM32L496XX_H_ */









