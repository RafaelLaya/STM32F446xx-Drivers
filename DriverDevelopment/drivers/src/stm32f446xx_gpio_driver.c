/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Jul 18, 2019
 *      Author: rafae
 */

#include "stm32f446xx_gpio_driver.h"
#include <string.h>

/************ APIS SUPPORTED BY THIS DRIVER ************/

/*
 * @fn				- GPIO_PeriClockControl
 * @brief			- Enables or Disabled Clock to given GPIO Port
 * @param[in]		- Base address of the GPIO Peripheral Port
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIO, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		if(pGPIO == GPIOA) {
			GPIOA_PCLK_EN();
		} else if(pGPIO == GPIOB) {
			GPIOB_PCLK_EN();
		} else if(pGPIO == GPIOC) {
			GPIOC_PCLK_EN();
		} else if(pGPIO == GPIOD) {
			GPIOD_PCLK_EN();
		} else if(pGPIO == GPIOE) {
			GPIOE_PCLK_EN();
		} else if(pGPIO == GPIOF) {
			GPIOF_PCLK_EN();
		} else if(pGPIO == GPIOG) {
			GPIOG_PCLK_EN();
		} else if(pGPIO == GPIOH) {
			GPIOH_PCLK_EN();
		}
	} else if(EnOrDi == DISABLE) {
		if(pGPIO == GPIOA) {
			GPIOA_PCLK_DI();
		} else if(pGPIO == GPIOB) {
			GPIOB_PCLK_DI();
		} else if(pGPIO == GPIOC) {
			GPIOC_PCLK_DI();
		} else if(pGPIO == GPIOD) {
			GPIOD_PCLK_DI();
		} else if(pGPIO == GPIOE) {
			GPIOE_PCLK_DI();
		} else if(pGPIO == GPIOF) {
			GPIOF_PCLK_DI();
		} else if(pGPIO == GPIOG) {
			GPIOG_PCLK_DI();
		} else if(pGPIO == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * @fn				- GPIO_DeInit
 * @brief			- Initializes the given GPIOx peripheral with the given PinConfig
 * @param[in]		- GPIO Object/Handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	// enable clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIO, ENABLE);

	// Configure Mode
	// first clear bits
	pGPIOHandle->pGPIO->MODER &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	// now configure. <= than analog are modes without interrupt
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		pGPIOHandle->pGPIO->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	} else {
		// configure as input. Usually not necessary since reset value is 0x0 for most
		// of the ports
		pGPIOHandle->pGPIO->MODER |= (GPIO_MODE_IN << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// clear RT
			EXTI->RTSR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// configure FT
			EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// clear FT
			EXTI->FTSR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// configure RT
			EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// configure FT and RT
			EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// Select this port through SYSCFG
		// enable clock of SYSCFG first
		SYSCFG_PCLK_EN();
		int CR_Array_Position = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		int CR_Pin_Position = 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		SYSCFG->EXTICR[CR_Array_Position] &= ~(0xF << CR_Pin_Position);
		SYSCFG->EXTICR[CR_Array_Position] |= GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIO) << CR_Pin_Position;

		// Enable delivery through IMR
		EXTI->IMR |= 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	// Configure Output Type
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT || pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		pGPIOHandle->pGPIO->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIO->OTYPER |= pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	// Configure Input/Output Speed
	pGPIOHandle->pGPIO->OSPEEDER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	pGPIOHandle->pGPIO->OSPEEDER |= pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2);

	// Configure Pull-up/Pull-down resistors
	pGPIOHandle->pGPIO->PUPDR &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	pGPIOHandle->pGPIO->PUPDR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2);

	// Alternate Function (Low/High Registers)
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t LowOrHighR = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t offset = 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIO->AFR[LowOrHighR] &= ~(0xF << offset);
		pGPIOHandle->pGPIO->AFR[LowOrHighR] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << offset;
	}
}

/*
 * @fn				- GPIO_PeriClockControl
 * @brief			- Deinitializes/Resets the given GPIOx peripheral with the given PinConfig
 * @param[in]		- Base address of GPIO port
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIO) {
	if(pGPIO == GPIOA) {
		GPIOA_REG_RESET();
	} else if(pGPIO == GPIOB) {
		GPIOB_REG_RESET();
	} else if(pGPIO == GPIOC) {
		 GPIOC_REG_RESET();
	} else if(pGPIO == GPIOD) {
		GPIOD_REG_RESET();
	} else if(pGPIO == GPIOE) {
		GPIOE_REG_RESET();
	} else if(pGPIO == GPIOF) {
		GPIOF_REG_RESET();
	} else if(pGPIO == GPIOG) {
		GPIOG_REG_RESET();
	} else if(pGPIO == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*
 * @fn				- GPIO_ReadFromInputPin
 * @brief			- Reads data from the given GPIOx port's pin number PinNumber
 * @param[in]		- Base address of the given GPIOx port
 * @param[in]		- Pin Number
 *
 * @return			- The data that is read at the given pin of the given GPIOx port
 *
 * @Note			- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	return (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
}

/*
 * @fn				- GPIO_ReadFromInputPort
 * @brief			- Reads data from the given GPIOx port
 * @param[in]		- Base address of the given GPIOx port
 *
 * @return			- The data that is present at the given GPIOx port
 *
 * @Note			- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t) (pGPIOx->IDR);
}

/*
 * @fn				- GPIO_WriteToOutputPin
 * @brief			- Writes data to the given GPIOx port's pin number PinNumber
 * @param[in]		- Base address of the given GPIOx port
 * @param[in]		- Pin Number
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value) {
	//pGPIOx->ODR = (pGPIOx->ODR & ~(0x1 << PinNumber)) | (value << PinNumber);
	if(value == GPIO_PIN_SET) {
		pGPIOx->ODR |= 0x1 << PinNumber;
	} else {
		pGPIOx->ODR &= ~(0x1 << PinNumber);
	}
}

/*
 * @fn				- GPIO_WriteToOutputPort
 * @brief			- Writes data to the given GPIOx port
 * @param[in]		- Base address of the given GPIOx port
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}

/*
 * @fn				- GPIO_ToggleOutputPin
 * @brief			- Toggles Output Pin (of number PinNumber) at the given GPIOx port
 * @param[in]		- Base address of the GPIO Peripheral Port
 * @param[in]		- Pin Number
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (0x1 << PinNumber);
}

/*
 * @fn				- GPIO_IRQInterruptConfig
 * @brief			- Configures IRQ for a given peripheral identified by IRQNumber. Enables CPU to receive the interrupt from
 * 						this IRQ number.
 * @param[in]		- IRQ Number of the peripheral
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
	// Enable or Disable Interrupts for this IRQ number
	if(EnOrDi == ENABLE) {
		*(NVIC_ISER0 + IRQNumber / 32) |= 0x1 << (IRQNumber % 32);
	} else {
		*(NVIC_ICER0 + IRQNumber / 32) &= ~(0x1 << (IRQNumber % 32));
	}
}

/*
 * @fn				- GPIO_IRQConfig
 * @brief			- Configures priority for this IRQ
 * @param[in]		- IRQ Number of the peripheral
 * @param[in]		- The priority to use for this peripheral. Possible values 0 to 15
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	*(NVIC_IPR0 + IRQNumber / 4) &= ~(0xff << (8 * (IRQNumber % 4)));
	*(NVIC_IPR0 + IRQNumber / 4) |= IRQPriority << ((8 * (IRQNumber % 4)) + 8 - NO_BITS_IMPLEMENTED);
}

/*
 * @fn				- GPIO_IRQHandling
 * @brief			- Does the job that has to be done after the interrupt has occurred in order to keep operating
 * 						in the case of GPIO, this is checking if the interrupt request has been made and then
 * 						clearing the request
 * @param[in]		- Pin Number
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	if((EXTI->PR >> PinNumber) & 0x1) {
		EXTI->PR |= 0x1 << PinNumber;
	}
}

void GPIO_ClearConfig(GPIO_Handle_t *pHandle){
	memset(pHandle, 0, sizeof(*pHandle));
}

