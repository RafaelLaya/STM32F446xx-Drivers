/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jul 18, 2019
 *      Author: rafae
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;			/* Possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;			/* Possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* Possible values from @GPIO_PIN_SPEEDS */
	uint8_t GPIO_PinPuPdControl;	/* Possible values from @GPIO_PUPD_VALUES */
	uint8_t GPIO_PinOPType;			/* Possible values from @GPIO_OP_Type */
	uint8_t GPIO_PinAltFunMode;		/* Possible values from @GPIO_ALTFN_MODES */
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIO;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/* @GPIO_PIN_NUMBERS */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/* @GPIO_PIN_MODES */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_RT			4
#define GPIO_MODE_IT_FT			5
#define GPIO_MODE_IT_RFT		6

/* Possible values from @GPIO_OP_Type */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/* @GPIO_PIN_SPEEDS */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/* @GPIO_PUPD_VALUES */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2

/* @GPIO_ALTFN_MODES */
#define GPIO_AF_0			0
#define GPIO_AF_1			1
#define GPIO_AF_2			2
#define GPIO_AF_3			3
#define GPIO_AF_4			4
#define GPIO_AF_5			5
#define GPIO_AF_6			6
#define GPIO_AF_7			7
#define GPIO_AF_8			8
#define GPIO_AF_9			9
#define GPIO_AF_10			10
#define GPIO_AF_11			11
#define GPIO_AF_12			12
#define GPIO_AF_13			13
#define GPIO_AF_14			14
#define GPIO_AF_15			15

/************ APIS SUPPORTED BY THIS DRIVER ************/

/*
 * Peripheral Clock Setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIO, uint8_t EnOrDi);

/*
 * Initialization and De-initialization
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIO);

/*
 * Data read and Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * others
 */

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
