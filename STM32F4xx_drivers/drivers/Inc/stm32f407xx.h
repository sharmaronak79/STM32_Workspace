/*
 * stm32f407xx.h
 *
 *  Created on: Sep 2, 2022
 *      Author: ronakkumar_sharma
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#define __vo volatile


/* Base Address of Flash and SRAM memory */

#define FLASH_BASEADDR		0x80000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define ROM_BASEADDR		0x1FFF0000U
#define RAM 				SRAM1_BASEADDR




/* APBx and AHBx Bus peripheral Base Address */

#define PERIPH_BASE 		0X40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U


/*Base Address of Peripherals hanging on AHB1 Peripheral Bus*/

#define GPIOA_BASEADDR  	(APB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR  	(APB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR  	(APB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR  	(APB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR  	(APB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR  	(APB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR  	(APB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR  	(APB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR  	(APB1PERIPH_BASE + 0x2000)




/*Base Address of Peripherals hanging on APB1 Peripheral Bus*/

#define I2C1_BASEADDR     (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR     (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR     (APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR     (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR     (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR     (APB1PERIPH_BASE + 0x5400)
#define USART3_BASEADDR     (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR     (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR     (APB1PERIPH_BASE + 0x5000)


/*Base Address of Peripherals hanging on APB2 Peripheral Bus*/

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        		(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)



/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;                       /*!< ,     										Address offset: 0x04      */
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;



#endif /* INC_STM32F407XX_H_ */
