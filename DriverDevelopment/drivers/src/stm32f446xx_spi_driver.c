/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Jul 25, 2019
 *      Author: rafae
 */

#include "stm32f446xx_SPI_driver.h"

static void SPI_Ovr_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_Rxe_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_Txe_IT_Handle(SPI_Handle_t *pSPIHandle);

/*
 * @fn				- SPI_PeriClockControl
 * @brief			- Enables or Disables Clock to given SPI peripheral
 * @param[in]		- Base address of the SPI Peripheral
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if(pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	} else {
		if(pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if(pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

/*
 * @fn				- SPI_Init
 * @brief			- Configures SPI
 * @param[in]		- Handle structure (pointer to base address + configuration structure) of SPI
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	/* Configure CR1 Register */

	// enable clock
	SPI_PeriClockControl(pSPIHandle->pSPI, ENABLE);

	// Select Mode
	pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_MSTR);
	pSPIHandle->pSPI->CR1 |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	// Select BusConfig (Full Duplex, Half-Duplex, Simplex RX)
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// 2 unidirectional lines
		pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_BIDIMODE);

		// clearn rxonly
		pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_RXONLY);

	} else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// 1 bidirectional line
		pSPIHandle->pSPI->CR1 |= (0x1 << SPI_CR1_BIDIMODE);

		// clear rxonly = transmit and receive
		pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_RXONLY);

	} else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// unidirectional lines
		pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_BIDIMODE);

		// set rxonly = only receive
		pSPIHandle->pSPI->CR1 |= (0x1 << SPI_CR1_RXONLY);

	}

	// select Clock Speed
	pSPIHandle->pSPI->CR1 &= ~(0x7 << SPI_CR1_BR);
	pSPIHandle->pSPI->CR1 |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	// select data format frame
	pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_DFF);
	pSPIHandle->pSPI->CR1 |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	// select polarity
	pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_CPOL);
	pSPIHandle->pSPI->CR1 |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	// select phase
	pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_CPHA);
	pSPIHandle->pSPI->CR1 |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	// select SSM
	pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_SSM);
	pSPIHandle->pSPI->CR1 |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	// select LSB or MSB
	pSPIHandle->pSPI->CR1 &= ~(0x1 << SPI_CR1_LSBFIRST);
	pSPIHandle->pSPI->CR1 |= (pSPIHandle->SPI_Config.SPI_LSBFIRST << SPI_CR1_LSBFIRST);
}

/*
 * @fn				- SPI_DeInit
 * @brief			- Resets the given SPI peripheral
 * @param[in]		- Base address of the SPI Peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPI) {
	if(pSPI == SPI1) {
		SPI1_REG_RESET();
	} else if(pSPI == SPI2) {
		SPI2_REG_RESET();
	} else if(pSPI == SPI3) {
		SPI3_REG_RESET();
	} else if(pSPI == SPI4) {
		SPI4_REG_RESET();
	}
}

/*
 * @fn				- SPI_SendData
 * @brief			- Sends data in pTxBuffer (of length len) through MOSI of pSPI
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		- Pointer to the buffer where data lies
 * @param[in]		- Length of data
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPI, uint8_t *pTxBuffer, uint32_t len) {
	while(len > 0) {
		while(SPI_GetFlagStatus(pSPI, SPI_TXE_FLAG) == FLAG_RESET);
		if((pSPI->CR1 >> SPI_CR1_DFF) & 0x1) {
			// dff = 1 = 16 bits
			pSPI->DR = *((uint16_t *) pTxBuffer);
			(uint16_t *)pTxBuffer++;
			len--;
			len--;
		} else {
			// dff = 0 = 8 bits
			pSPI->DR = *((uint8_t *) pTxBuffer);
			pTxBuffer++;
			len--;
		}
	}
}

/*
 * @fn				- SPI_GetFlagStatus
 * @brief			- Retrives the flag status FlagName from the SR
 * @param[in]		- Base address of SPI peripheral
 * @param[in]		- The name of the Flag from @SPI_Flags in stm32f446xx_spi.driver.h
 *
 * @return			- The value of the flag
 *
 * @Note			- none
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if(pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * @fn				- SPI_ReadData
 * @brief			- Reads data to pRxBuffer (of length len) from the pSPI
 * @param[in]		- Base address of the given SPI peripheral
 * @param[in]		- Pointer where data will be read
 * @param[in]		- length of the data
 *
 * @return			- none
 *
 * @Note			- Block (Non-Interrupt) based
 *
 */
void SPI_ReadData(SPI_RegDef_t *pSPI, uint8_t *pRxBuffer, uint32_t len) {
	while(len > 0) {
		while(SPI_GetFlagStatus(pSPI, SPI_RXNE_FLAG) == FLAG_RESET);
		if((pSPI->CR1 >> SPI_CR1_DFF) & 0x1) {
			// 16 bits
			*((uint16_t *) pRxBuffer) = pSPI->DR;
			(uint16_t *)pRxBuffer++;
			len--;
			len--;
		} else {
			// 8 bits
			*pRxBuffer = pSPI->DR;
			pRxBuffer++;
			len--;
		}
	}
}

/*
 * @fn				- SPI_IRQInterruptConfig
 * @brief			- Enables or Disabled Interrupts from Processor Side
 * @param[in]		- IRQ Number associated
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		*(NVIC_ISER0 + IRQNumber / 32) |= (0x1 << (IRQNumber % 32));
	} else {
		*(NVIC_ICER0 + IRQNumber / 32) &= ~(0x1 << (IRQNumber % 32));
	}
}

/*
 * @fn				- SPI_IRQHandling
 * @brief			- Figures why the flag occurred and handles each situation accordingly
 * @param[in]		- A pointer to the handle structure (base address + configuration structure) of the SPI peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
	// if status is Transmission buffer empty
	// and interrupts are enabled
	uint8_t flag1 = ((pSPIHandle->pSPI->CR2 >> SPI_CR2_TXEIE) & 0x1);
	if(flag1) {
		uint8_t flag2 = ((pSPIHandle->pSPI->SR >> SPI_SR_TXE) & 0x1);
		if(flag2) {
			// handle TXE
			SPI_Txe_IT_Handle(pSPIHandle);
		}
	}

	flag1 = ((pSPIHandle->pSPI->CR2 >> SPI_CR2_RXNEIE) & 0x1);
	if(flag1) {
		uint8_t flag2 = ((pSPIHandle->pSPI->SR >> SPI_SR_RXNE) & 0x1);
		if(flag2) {
			SPI_Rxe_IT_Handle(pSPIHandle);
		}
	}

	flag1 = ((pSPIHandle->pSPI->SR >> SPI_SR_OVR) & 0x1);
	if(flag1) {
		uint8_t flag2 = ((pSPIHandle->pSPI->CR2 >> SPI_CR2_ERRIE) & 0x1);
		if(flag2) {
			SPI_Ovr_IT_Handle(pSPIHandle);
		}
	}
}

/*
 * @fn				- SPI_Txe_IT_Handle
 * @brief			- Sens one or two bytes of data and checks if transmission needs to be closed
 * @param[in]		- A pointer to the handle structure (base address + configuration structure) of the SPI peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void SPI_Txe_IT_Handle(SPI_Handle_t *pSPIHandle) {
	// check dff
	if((pSPIHandle->pSPI->CR1 >> SPI_CR1_DFF) & 0x1) {
		// 16bits
		pSPIHandle->pSPI->DR = *((uint16_t *) (pSPIHandle->pTxBuffer));
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	} else {
		// 8bits
		pSPIHandle->pSPI->DR = *((uint8_t *) (pSPIHandle->pTxBuffer));
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(pSPIHandle->TxLen <= 0) {
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/*
 * @fn				- SPI_Txe_IT_Handle
 * @brief			- Receives one byte of data and checks if transmission should be closed
 * @param[in]		- A pointer to the handle structure (base address + configuration structure) of the SPI peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void SPI_Rxe_IT_Handle(SPI_Handle_t *pSPIHandle) {
	// check dff
	if((pSPIHandle->pSPI->CR1 >> SPI_CR1_DFF) & 0x1) {
		// 16bits
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPI->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	} else {
		// 8bits
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPI->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(pSPIHandle->RxLen <= 0) {
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/*
 * @fn				- SPI_Ovr_IT_Handle
 * @brief			- Clears the OVR flag and notifies user application
 * @param[in]		- A pointer to the handle structure (base address + configuration structure) of the SPI peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void SPI_Ovr_IT_Handle(SPI_Handle_t *pSPIHandle) {
	// if error, notify error and clear flag
	// the clearing sequence is: read DR and then read SR
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		uint8_t data = pSPIHandle->pSPI->DR;
		uint32_t status = pSPIHandle->pSPI->SR;
		(void) data;
		(void) status;
	}
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/*
 * @fn				- SPI_CloseReception
 * @brief			- Resets reception related fields in the handle structure and stops receiving related interrupts
 * @param[in]		- A pointer to the handle structure (base address + configuration structure) of the SPI peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->pSPI->CR2 &= ~(0x1 << SPI_CR2_RXNEIE);
}

/*
 * @fn				- SPI_CloseTransmission
 * @brief			- Resets transmission related fields in the handle structure and stops transmission related interrupts
 * @param[in]		- A pointer to the handle structure (base address + configuration structure) of the SPI peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
	pSPIHandle->pSPI->CR2 &= ~(0x1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
}

/*
 * @fn				- SPI_ClearOVRFlag
 * @brief			- Clears the OVR Flag
 * @param[in]		- A pointer to the handle structure (base address + configuration structure) of the SPI peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle) {
	uint8_t data = pSPIHandle->pSPI->DR;
	uint32_t status = pSPIHandle->pSPI->SR;
	(void) data;
	(void) status;
}

/*
 * @fn				- SPI_SendDataWithIT
 * @brief			- Sends data from pTxBuffer (of length len)
 * @param[in]		- Base address of the given SPI peripheral
 * @param[in]		- Pointer where data is
 * @param[in]		- length of the data
 *
 * @return			- Status of SPI
 *
 * @Note			- Interrupt based
 *
 */
uint8_t SPI_SendDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len) {
	uint8_t status = pSPIHandle->TxState;
	if(status != SPI_BUSY_IN_TX) {
		// Save pTxBuffer in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		// Put pSPI as busy
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		// Enable interrupt TXEIE when TXE flag is Set
		pSPIHandle->pSPI->CR2 |= (0x1 << SPI_CR2_TXEIE);
		// implement data transmission in ISR
	}
	return status;
}

/*
 * @fn				- SPI_ReadDataWithIT
 * @brief			- Receives data from SPI peripheral related to the pSPIHandle given
 * @param[in]		- Handle structure of SPI
 * @param[in]		- Pointer where data will be stored
 * @param[in]		- length of the remaining data
 *
 * @return			- Status of SPI
 *
 * @Note			- Interrupt based
 *
 */
uint8_t SPI_ReadDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len) {
	uint8_t status = pSPIHandle->RxState;
	if(status != SPI_BUSY_IN_RX) {
		// store message
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		// flag busy in rx
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// enable interrupt
		pSPIHandle->pSPI->CR2 |= (0x1 << SPI_CR2_RXNEIE);
	}
	return status;
}

/*
 * @fn				- SPI_IRQPriorityConfig
 * @brief			- Selects priority for the SPI peripheral
 * @param[in]		-
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	*(NVIC_IPR0 + IRQNumber / 4) &= ~(0xff << (8 * (IRQNumber % 4)));
	*(NVIC_IPR0 + IRQNumber / 4) |= (IRQPriority << (((IRQNumber % 4) * 8) + 8 - PRIO_BITS_IMPLEMENTED));
}

/*
 * @fn				- SPI_PeripheralControl
 * @brief			- Enables or Disables peripheral clock
 * @param[in]		- Pointer to the base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPI, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pSPI->CR1 |= (0x1 << SPI_CR1_SPE);
	} else if(EnOrDi == DISABLE) {
		pSPI->CR1 &= ~(0x1 << SPI_CR1_SPE);
	}
}

/*
 * @fn				- SPI_SSIConfig
 * @brief			- When using SSM (Software Slave Management) SSI (Internal Slave Select)
 * 					  This bit's value is forced onto the NSS pin
 * @param[in]		- Pointer to the base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			-
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPI, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pSPI->CR1 |= (0x1 << SPI_CR1_SSI);
	} else if(EnOrDi == DISABLE) {
		pSPI->CR1 &= ~(0x1 << SPI_CR1_SSI);
	}
}

/*
 * @fn				- SPI_SSOEConfig
 * @brief			- Enables or disables SSOE. When Set, the unit cannot work in
 * 					  multimaster configuration
 * @param[in]		- Pointer to the base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			-
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPI, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pSPI->CR2 |= (0x1 << SPI_CR2_SSOE);
	} else if(EnOrDi == DISABLE) {
		pSPI->CR2 &= ~(0x1 << SPI_CR2_SSOE);
	}
}

/*
 * @fn				- SPI_ApplicationEventCallback
 * @brief			- Weak function that the user application must override in order to receive
 * 				      events notifications
 * @param[in]		- Pointer to the handle structure of the relevant SPI
 * @param[in]		- SPI_EV macro
 *
 * @return			- none
 *
 * @Note			- Should be overridden
 *
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event) {

}

