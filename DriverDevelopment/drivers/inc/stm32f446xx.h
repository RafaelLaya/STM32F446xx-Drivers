/*
 * stm32f446xx.h
 *
 *  Created on: Jul 17, 2019
 *      Author: rafae
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

/*
 * Libraries
 */
#include <stdint.h>
#include <stddef.h>

/*
 * Short-hand Notation
 */
#define __vo volatile


/******************** Cortex M-4 Processor ********************/

#define NO_BITS_IMPLEMENTED				4

/*
 * NVIC ISERx Registers
 */
#define NVIC_ISER0			(__vo uint32_t*) 0xE000E100
#define NVIC_ISER1			(__vo uint32_t*) 0xE000E104
#define NVIC_ISER2			(__vo uint32_t*) 0xE000E108
#define NVIC_ISER3			(__vo uint32_t*) 0xE000E10c

/*
 * Priority levels for the NVIC
 */
#define NVIC_IRQ_PRIO0			0
#define NVIC_IRQ_PRIO1			1
#define NVIC_IRQ_PRIO2			2
#define NVIC_IRQ_PRIO3			3
#define NVIC_IRQ_PRIO4			4
#define NVIC_IRQ_PRIO5			5
#define NVIC_IRQ_PRIO6			6
#define NVIC_IRQ_PRIO7			7
#define NVIC_IRQ_PRIO8			8
#define NVIC_IRQ_PRIO9			9
#define NVIC_IRQ_PRIO10			10
#define NVIC_IRQ_PRIO11			11
#define NVIC_IRQ_PRIO12			12
#define NVIC_IRQ_PRIO13			13
#define NVIC_IRQ_PRIO14			14
#define NVIC_IRQ_PRIO15			15

/*
 * IPR0 Register
 */

#define NVIC_IPR0			(__vo uint32_t*) 0xE000E400

/*
 * NVIC ISERx Registers
 */
#define NVIC_ICER0			(__vo uint32_t*) 0xE000E180
#define NVIC_ICER1			(__vo uint32_t*) 0xE000E184
#define NVIC_ICER2			(__vo uint32_t*) 0xE000E188
#define NVIC_ICER3			(__vo uint32_t*) 0xE000E188c

/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR				0x08000000U				/* Flash Memory Starts at this address */
#define SRAM1_BASEADDR				0x20000000U				/* SRAM1 Memory Starts at this address */
#define SRAM2_BASEADDR				0x2001C000U				/* SRAM2 Memory Starts at this address */
#define ROM_BASEADDR				0x1FFF0000U				/* ROM Memory Starts at this address */
#define SRAM_BASEADDR				SRAM1_BASEADDR			/* SRAM Memory Starts at this address */

/*
 * base addresses of APBx and AHBx peripheral buses
 */

#define PERIPH_BASEADDR				0x40000000U				/* Peripherals Start at this address */
#define APB1_BASEADDR				PERIPH_BASEADDR			/* APB1 Bus Starts at this address */
#define APB2_BASEADDR				0x40010000U				/* APB2 Bus Starts at this address */
#define AHB1_BASEADDR				0x40020000U				/* AHB1 Bus Starts at this address */
#define AHB2_BASEADDR				0x50000000U				/* AHB2 Bus Starts at this address */
#define AHB3_BASEADDR				0xA0001000U				/* AHB3 Bus Starts at this address */

/*
 * base addresses of peripherals hanging on the AHB1 Bus
 */

#define GPIOA_BASEADDR				(AHB1_BASEADDR + 0x0000U)				/* GPIO port A Starts at this address */
#define GPIOB_BASEADDR				(AHB1_BASEADDR + 0x0400U)				/* GPIO port B Starts at this address */
#define GPIOC_BASEADDR				(AHB1_BASEADDR + 0x0800U)				/* GPIO port C Starts at this address */
#define GPIOD_BASEADDR				(AHB1_BASEADDR + 0x0C00U)				/* GPIO port D Starts at this address */
#define GPIOE_BASEADDR				(AHB1_BASEADDR + 0x1000U)				/* GPIO port E Starts at this address */
#define GPIOF_BASEADDR				(AHB1_BASEADDR + 0x1400U)				/* GPIO port F Starts at this address */
#define GPIOG_BASEADDR				(AHB1_BASEADDR + 0x1800U)				/* GPIO port G Starts at this address */
#define GPIOH_BASEADDR				(AHB1_BASEADDR + 0x1C00U)				/* GPIO port H Starts at this address */

#define RCC_BASEADDR				(AHB1_BASEADDR + 0x3800U)				/* RCC Starts at this address */

/*
 * base addresses of peripherals hanging on the APB1 Bus
 */

#define I2C1_BASEADDR				(APB1_BASEADDR + 0x5400U)				/* I2C1 peripheral starts at this address */
#define I2C2_BASEADDR				(APB1_BASEADDR + 0x5800U)				/* I2C2 peripheral starts at this address */
#define I2C3_BASEADDR				(APB1_BASEADDR + 0x5C00U)				/* I2C3 peripheral starts at this address */
#define SPI2_BASEADDR				(APB1_BASEADDR + 0x3800U)				/* SPI2 peripheral starts at this address */
#define SPI3_BASEADDR				(APB1_BASEADDR + 0x3C00U)				/* SPI3 peripheral starts at this address */
#define USART2_BASEADDR				(APB1_BASEADDR + 0x4400U)				/* USART2 peripheral starts at this address */
#define USART3_BASEADDR				(APB1_BASEADDR + 0x4800U)				/* USART3 peripheral starts at this address */
#define UART4_BASEADDR				(APB1_BASEADDR + 0x4C00U)				/* UART4 peripheral starts at this address */
#define UART5_BASEADDR				(APB1_BASEADDR + 0x5000U)				/* UART5 peripheral starts at this address */

/*
 * base addresses of peripherals hanging on the APB2 Bus
 */

#define EXTI_BASEADDR				(APB2_BASEADDR + 0x3C00U)				/* EXTI peripheral starts at this address */
#define SPI1_BASEADDR				(APB2_BASEADDR + 0x3000U)				/* SPI1 peripheral starts at this address */
#define SPI4_BASEADDR				(APB2_BASEADDR + 0x3400U)				/* SPI4 peripheral starts at this address */
#define SYSCFG_BASEADDR				(APB2_BASEADDR + 0x3800U)				/* SYSCFG peripheral starts at this address */
#define USART1_BASEADDR				(APB2_BASEADDR + 0x1000U)				/* USART1 peripheral starts at this address */
#define USART6_BASEADDR				(APB2_BASEADDR + 0x1400U)				/* USART6 peripheral starts at this address */

/******************************************* Peripheral Structure Definitions *******************************************/

typedef struct {
	__vo uint32_t MODER;				/* Selects input/output/alternate/analog modes								0x00 offset */
	__vo uint32_t OTYPER;				/* Selects output type (push-pull/open-drain)								0x04 offset */
	__vo uint32_t OSPEEDER;				/* Selects output speed														0x08 offset */
	__vo uint32_t PUPDR;				/* Configures pull-up/down resistors										0x0C offset */
	__vo uint32_t IDR;					/* Contains input value of I/O port											0x10 offset */
	__vo uint32_t ODR;					/* Contains output value													0x14 offset */
	__vo uint32_t BSRR;					/* Bit Set/Reset Register for Output Data									0x18 offset */
	__vo uint32_t LCKR;					/* Locks bit configuration of GPIO											0x1C offset */
	__vo uint32_t AFR[2];				/* Selects Alternate Function for pins AFR[0]:1...7 and AFR[1]:8...15 		0x20 offset */
} GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;					/* Clock Control Register												offset 0x00 */
	__vo uint32_t PLLCFGR;				/* PLL Configuration Register											offset 0x04 */
	__vo uint32_t CFGR;					/* Clock Configuration Register											offset 0x08 */
	__vo uint32_t CIR;					/* Clock Interrupt Register												offset 0x0C */
	__vo uint32_t AHB1RSTR;				/* AHB1 Peripheral Reset Register										offset 0x10 */
	__vo uint32_t AHB2RSTR;				/* AHB2 Peripheral Reset Register										offset 0x14 */
	__vo uint32_t AHB3RSTR;				/* AHB3 Peripheral Reset Register										offset 0x18 */
	uint32_t RESERVED0;					/* Reserved 	0x1C */
	__vo uint32_t APB1RSTR;				/* APB1 Peripheral Reset Register										offset 0x20 */
	__vo uint32_t APB2RSTR;				/* APB2 Peripheral Reset Register										offset 0x24 */
	uint32_t RESERVED1[2];				/* Reserved		0x28 and 0x2C */
	__vo uint32_t AHB1ENR;				/* AHB1 Peripheral Clock Enable Register								offset 0x30 */
	__vo uint32_t AHB2ENR;				/* AHB2 Peripheral Clock Enable Register								offset 0x34 */
	__vo uint32_t AHB3ENR;				/* AHB3 Peripheral Clock Enable Register								offset 0x38 */
	uint32_t RESERVED2;					/* Reserved		0x3C */
	__vo uint32_t APB1ENR;				/* APB1 Peripheral Clock Enable Register								offset 0x40 */
	__vo uint32_t APB2ENR;				/* APB2 Peripheral Clock Enable Register								offset 0x44 */
	uint32_t RESERVED3[2];				/* Reserved	 	0x48 and 0x4C */
	__vo uint32_t AHB1LPENR;			/* AHB1 Peripheral Clock Enable in Low Power Mode Register				offset 0x50 */
	__vo uint32_t AHB2LPENR;			/* AHB2 Peripheral Clock Enable in Low Power Mode Register				offset 0x54 */
	__vo uint32_t AHB3LPENR;			/* AHB3 Peripheral Clock Enable in Low Power Mode Register				offset 0x58 */
	uint32_t RESERVED4;					/* Reserved		0x5C */
	__vo uint32_t APB1LPENR;			/* APB1 Peripheral Clock Enable in Low Power Mode Register				offset 0x60 */
	__vo uint32_t APB2LPENR;			/* APB2 Peripheral Clock Enable in Low Power Mode Register				offset 0x64 */
	uint32_t RESERVED5[2];				/* Reserved		0x68 and 0x6C */
	__vo uint32_t BDCR;					/* Backup Domain Control Register										offset 0x70 */
	__vo uint32_t CSR;					/* Clock Control and Status Register									offset 0x74 */
	uint32_t RESERVED6[2];				/* Reserved		0x78 and 0x7C */
	__vo uint32_t SSCGR;				/* Spread Spectrum Clock Generation Register							offset 0x80 */
	__vo uint32_t PLLI2SCFGR;			/* PLLI2S Configuration Register										offset 0x84 */
	__vo uint32_t PLLSAICFGR;			/* PLL Configuration Register											offset 0x88 */
	__vo uint32_t DCKCFGR;				/* Dedicated Clock Configuration Register								offset 0x8C */
	__vo uint32_t CKGATENR;				/* Clocks Gated Enable Register											offset 0x90 */
	__vo uint32_t DCKCFGR2;				/* Dedicated Clock Configuration Register 								offset 0x94 */
} RCC_RegDef_t;

typedef struct {
	__vo uint32_t IMR;				/* Interrupt Mask Register 						0ffset 0x00 */
	__vo uint32_t EMR;				/* Event Mask Register 							offset 0x04 */
	__vo uint32_t RTSR;				/* Rising Trigger Selection Register 			offset 0x08 */
	__vo uint32_t FTSR;				/* Falling Trigger Selection Register 			offset 0x0C */
	__vo uint32_t SWIER;			/* Software Interrupt Event Register 			offset 0x10 */
	__vo uint32_t PR;				/* Pending Request Register 					offset 0x14 */
} EXTI_RegDef_t ;

typedef struct {
	__vo uint32_t MEMRMP;			/* Memory remap register 							offset 0x00 */
	__vo uint32_t PMC;				/* Peripheral Mode Configuration register 			offset 0x04 */
	__vo uint32_t EXTICR[4];		/* External Interrupt Configuration registers:
									   {EXTICR1, EXTICR2, EXTICR3, EXTICR4} 			offset 0x08 */
	uint32_t RESERVED_0[2];
	__vo uint32_t CMPCR;			/* Compensation cell control register 				offset 0x20 */
	uint32_t RESERVED_1[2];
	__vo uint32_t CFGR;				/* Configuration Register 							offset 0x2C */
} SYSCFG_RegDef_t;

typedef struct {
	__vo uint32_t CR1;			/* SPI Control Register 1 			offset 0x0 */
	__vo uint32_t CR2;			/* SPI Control Register 2 			offset 0x04 */
	__vo uint32_t SR;			/* SPI Status Register				offset 0x08 */
	__vo uint32_t DR;			/* SPI Data Register				offset 0x0c */
	__vo uint32_t CRCPR;		/* SPI CRC polynomial register		offset 0x10 */
	__vo uint32_t RXCRCR;		/* SPI RX CRC Register				offset 0x14 */
	__vo uint32_t TXCRCR;		/* SPI TX CRC register				offset 0x18 */
	__vo uint32_t I2SCFGR;		/* SPI_I2S configuration register	offset 0x1c */
	__vo uint32_t I2SPR;		/* SPI_I2S prescaler register		offset 0x20 */
} SPI_RegDef_t;

typedef struct {
	__vo uint32_t CR1;			/* Control Register 1			offset 0x00 */
	__vo uint32_t CR2;			/* Control Register 2			offset 0x04 */
	__vo uint32_t OAR1;			/* Own Address Register 1		offset 0x08 */
	__vo uint32_t OAR2;			/* Own Address Register 2		offset 0x0C */
	__vo uint32_t DR;			/* Data Register				offset 0x10 */
	__vo uint32_t SR1;			/* Status Register 1			offset 0x14 */
	__vo uint32_t SR2;			/* Status Register 2			offset 0x18 */
	__vo uint32_t CCR;			/* Clock Control Register		offset 0x1C */
	__vo uint32_t TRISE;		/* TRISE Register				offset 0x20 */
	__vo uint32_t FLTR;			/* FLTR Register				offset 0x24 */
} I2C_RegDef_t;

typedef struct {
	__vo uint32_t SR;			/* Status Register							offset 0x00 */
	__vo uint32_t DR;			/* Data Register							offset 0x04 */
	__vo uint32_t BRR;			/* Baud Rate Register						offset 0x08 */
	__vo uint32_t CR1;			/* Configuration Register 1					offset 0x0C */
	__vo uint32_t CR2;			/* Configuration Register 2					offset 0x10 */
	__vo uint32_t CR3;			/* Configuration Register 3					offset 0x14 */
	__vo uint32_t GPTR;			/* Guard Time and prescaler Register		offset 0x18 */
} USART_RegDef_t;

/*
 * Peripheral Definitions
 */

#define GPIOA				((GPIO_RegDef_t *) GPIOA_BASEADDR)				/* Pointer to GPIOA peripheral */
#define GPIOB				((GPIO_RegDef_t *) GPIOB_BASEADDR)				/* Pointer to GPIOB peripheral */
#define GPIOC				((GPIO_RegDef_t *) GPIOC_BASEADDR)				/* Pointer to GPIOC peripheral */
#define GPIOD				((GPIO_RegDef_t *) GPIOD_BASEADDR)				/* Pointer to GPIOD peripheral */
#define GPIOE				((GPIO_RegDef_t *) GPIOE_BASEADDR)				/* Pointer to GPIOE peripheral */
#define GPIOF				((GPIO_RegDef_t *) GPIOF_BASEADDR)				/* Pointer to GPIOF peripheral */
#define GPIOG				((GPIO_RegDef_t *) GPIOG_BASEADDR)				/* Pointer to GPIOG peripheral */
#define GPIOH				((GPIO_RegDef_t *) GPIOH_BASEADDR)				/* Pointer to GPIOH peripheral */

#define RCC					((RCC_RegDef_t *) RCC_BASEADDR)				/* Pointer to RCC (Reset and Clock Control) peripheral */

#define EXTI				((EXTI_RegDef_t *) EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t *) SPI4_BASEADDR)

#define I2C1				((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t *) I2C3_BASEADDR)


#define USART1					((USART_RegDef_t *) USART1_BASEADDR)
#define USART2					((USART_RegDef_t *) USART2_BASEADDR)
#define USART3					((USART_RegDef_t *) USART3_BASEADDR)
#define UART4					((USART_RegDef_t *) UART4_BASEADDR)
#define UART5					((USART_RegDef_t *) UART5_BASEADDR)
#define USART6					((USART_RegDef_t *) USART6_BASEADDR)

/*
 * Clock Enable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << 0))				/* GPIOA peripheral clock enable */
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))				/* GPIOB peripheral clock enable */
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))				/* GPIOC peripheral clock enable */
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))				/* GPIOD peripheral clock enable */
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))				/* GPIOE peripheral clock enable */
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))				/* GPIOF peripheral clock enable */
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))				/* GPIOG peripheral clock enable */
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))				/* GPIOH peripheral clock enable */

/*
 * Clock Enable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))				/* I2C1 peripheral clock enable */
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))				/* I2C2 peripheral clock enable */
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))				/* I2C3 peripheral clock enable */

/*
 * Clock Enable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))				/* SPI1 peripheral clock enable */
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))				/* SPI2 peripheral clock enable */
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))				/* SPI3 peripheral clock enable */
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13))				/* SPI4 peripheral clock enable */

/*
 * Clock Enable Macros for USART/UART Peripherals
 */

#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))				/* USART1 peripheral clock enable */
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))				/* USART2 peripheral clock enable */
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))				/* USART3 peripheral clock enable */
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))				/* UART4 peripheral clock enable */
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))				/* UART5 peripheral clock enable */
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))				/* USART6peripheral clock enable */

/*
 * Clock Enable Macros for SYSCFG Peripheral
 */

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))				/* SYSCFG peripheral clock enable */

/*
 * Clock Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))				/* GPIOA peripheral clock disable */
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))				/* GPIOB peripheral clock disable */
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))				/* GPIOC peripheral clock disable */
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))				/* GPIOD peripheral clock disable */
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))				/* GPIOE peripheral clock disable */
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))				/* GPIOF peripheral clock disable */
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))				/* GPIOG peripheral clock disable */
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))				/* GPIOH peripheral clock disable */

/*
 * Clock Disable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))				/* I2C1 peripheral clock disable */
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))				/* I2C2 peripheral clock disable */
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))				/* I2C3 peripheral clock disable */

/*
 * Clock Disable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))				/* SPI1 peripheral clock disable */
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))				/* SPI2 peripheral clock disable */
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))				/* SPI3 peripheral clock disable */
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 13))				/* SPI4 peripheral clock disable */

/*
 * Clock Disable Macros for USART/UART Peripherals
 */

#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))					/* USART1 peripheral clock disable */
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))				/* USART2 peripheral clock disable */
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))				/* USART3 peripheral clock disable */
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))				/* UART4 peripheral clock disable */
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))				/* UART5 peripheral clock disable */
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))					/* USART6 peripheral clock disable */

/*
 * Clock Disable Macro for SYSCFG Peripheral
 */

#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))				/* SYSCFG peripheral clock disable */

/*
 * Some Generic Macros
 */

#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_RESET					RESET
#define FLAG_SET					SET

/*
 * Resets for GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do { RCC->AHB1RSTR |= 0x1 << 0; RCC->AHB1RSTR &= ~(0x1 << 0); } while(0)
#define GPIOB_REG_RESET()			do { RCC->AHB1RSTR |= 0x1 << 1; RCC->AHB1RSTR &= ~(0x1 << 1); } while(0)
#define GPIOC_REG_RESET()			do { RCC->AHB1RSTR |= 0x1 << 2; RCC->AHB1RSTR &= ~(0x1 << 2); } while(0)
#define GPIOD_REG_RESET()			do { RCC->AHB1RSTR |= 0x1 << 3; RCC->AHB1RSTR &= ~(0x1 << 3); } while(0)
#define GPIOE_REG_RESET()			do { RCC->AHB1RSTR |= 0x1 << 4; RCC->AHB1RSTR &= ~(0x1 << 4); } while(0)
#define GPIOF_REG_RESET()			do { RCC->AHB1RSTR |= 0x1 << 5; RCC->AHB1RSTR &= ~(0x1 << 5); } while(0)
#define GPIOG_REG_RESET()			do { RCC->AHB1RSTR |= 0x1 << 6; RCC->AHB1RSTR &= ~(0x1 << 6); } while(0)
#define GPIOH_REG_RESET()			do { RCC->AHB1RSTR |= 0x1 << 7; RCC->AHB1RSTR &= ~(0x1 << 7); } while(0)

/*
 * Reset for SPI peripherals
 */
#define SPI1_REG_RESET()			do { RCC->APB2RSTR |= 0x1 << 12; RCC->APB2RSTR &= ~(0x1 << 12); } while(0)
#define SPI2_REG_RESET()			do { RCC->APB1RSTR |= 0x1 << 14; RCC->APB1RSTR &= ~(0x1 << 14); } while(0)
#define SPI3_REG_RESET()			do { RCC->APB1RSTR |= 0x1 << 15; RCC->APB1RSTR &= ~(0x1 << 15); } while(0)
#define SPI4_REG_RESET()			do { RCC->APB2RSTR |= 0x1 << 13; RCC->APB2RSTR &= ~(0x1 << 13); } while(0)

/*
 * Reset for I2C peripherals
 */
#define I2C1_REG_RESET()			do { RCC->APB1RSTR |= (0x1 << 21); RCC->APB1RSTR &= ~(0x1 << 21); } while(0)
#define I2C2_REG_RESET()			do { RCC->APB1RSTR |= (0x1 << 22); RCC->APB1RSTR &= ~(0x1 << 22); } while(0)
#define I2C3_REG_RESET()			do { RCC->APB1RSTR |= (0x1 << 23); RCC->APB1RSTR &= ~(0x1 << 23); } while(0)

/*
 * USART/UART
 */
#define USART1_REG_RESET()			do { RCC->APB2RSTR |= (0x1 << 4); RCC->APB2RSTR &= ~(0x1 << 4); } while(0)
#define USART2_REG_RESET()			do { RCC->APB1RSTR |= (0x1 << 17); RCC->APB1RSTR &= ~(0x1 << 17); } while(0)
#define USART3_REG_RESET()			do { RCC->APB1RSTR |= (0x1 << 18); RCC->APB1RSTR &= ~(0x1 << 18); } while(0)
#define UART4_REG_RESET()			do { RCC->APB1RSTR |= (0x1 << 19); RCC->APB1RSTR &= ~(0x1 << 19); } while(0)
#define UART5_REG_RESET()			do { RCC->APB1RSTR |= (0x1 << 20); RCC->APB1RSTR &= ~(0x1 << 20); } while(0)
#define USART6_REG_RESET()			do { RCC->APB2RSTR |= (0x1 << 5); RCC->APB2RSTR &= ~(0x1 << 5); } while(0)

/*
 * IRQ Numbers/Positions
 */
#define IRQ_EXTI0			6
#define IRQ_EXTI1			7
#define IRQ_EXTI2			8
#define IRQ_EXTI3			9
#define IRQ_EXTI4			10
#define IRQ_EXTI9_5			23
#define IRQ_EXTI15_10		40
#define IRQ_SPI1			35
#define IRQ_SPI2			36
#define IRQ_SPI3			51
#define IRQ_SPI4			84
#define IRQ_I2C1_EV			31
#define IRQ_I2C1_ER			32
#define IRQ_I2C2_EV			33
#define IRQ_I2C2_ER			34
#define IRQ_I2C3_EV			72
#define IRQ_I2C3_ER			73
#define IRQ_USART1			37
#define IRQ_USART2			38
#define IRQ_USART3			39
#define IRQ_UART4			52
#define IRQ_UART5			53
#define IRQ_USART6			71

/*
 * others
 */
// X possible values are: GPIOA, GPIOB, ..., GPIOH
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOF) ? 5 : \
									(x == GPIOG) ? 6 : \
									(x == GPIOH) ? 7 : 0)

/**************** Bit position definitions of RCC ************************/
/*
 * CFGR
 */
#define RCC_CFGR_SW				0
#define RCC_CFGR_SWS			2
#define RCC_CFGR_HPRE			4
#define RCC_CFGR_PPRE1			10
#define RCC_CFGR_PPRE2			13
#define RCC_CFGR_RTCPRE			16
#define RCC_CFGR_MCO1			21
#define RCC_CFGR_MCO1PRE		24
#define RCC_CFGR_MCO2PRE		27
#define RCC_CFGR_MCO2			30

/******************** Bit position definitions of SPI ********************/

/*
 * CR1
 */

#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

/*
 * CR2
 */
#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

/*
 * SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/************************ I2C Bit Definitions **************************/
/*
 * CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/*
 * CR2
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

/*
 * SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/*
 * SR2
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

/*
 * CCR
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/*************** USART Bit definitions *****************/

/*
 * SR
 */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

/*
 * BRR
 */
#define USART_BRR_DIV_FRACTION				0
#define USART_BRR_DIV_MANTISSA				4

/*
 * CR1
 */
#define USART_CR1_SBK				0
#define USART_CR1_RWU				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15

/*
 * CR2
 */
#define USART_CR2_ADD				0
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14

/*
 * CR3
 */
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11

/*
 * GPTR
 */
#define USART_GPTR_PSC			0
#define USART_GPTR_GT			8

#define __weak				__attribute__((weak))

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_I2C_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* INC_STM32F446XX_H_ */
