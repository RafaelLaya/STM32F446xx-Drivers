/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Aug 4, 2019
 *      Author: rafae
 */

#include "stm32f446xx_usart_driver.h"


/*
 * @fn				- USART_PeriClockControl
 * @brief			- Enables or Disables Clock to given USART peripheral
 * @param[in]		- Base address of the USART Peripheral
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if(pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if(pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if(pUSARTx == UART4) {
			UART4_PCLK_EN();
		} else if(pUSARTx == UART5) {
			UART5_PCLK_EN();
		} else if(pUSARTx == USART6) {
			USART6_PCLK_EN();
		}
	} else {
		if(pUSARTx == USART1) {
			USART1_PCLK_DI();
		} else if(pUSARTx == USART2) {
			USART2_PCLK_DI();
		} else if(pUSARTx == USART3) {
			USART3_PCLK_DI();
		} else if(pUSARTx == UART4) {
			UART4_PCLK_DI();
		} else if(pUSARTx == UART5) {
			UART5_PCLK_DI();
		} else if(pUSARTx == USART6) {
			USART6_PCLK_DI();
		}
	}
}

/*
 * @fn				- USART_SetBaudRate
 * @brief			- Selects the baud rate
 * @param[in]		- Base address of the USART Peripheral
 * @param[in]		- BaudRate (Hz)
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {
	uint32_t APBxCLK = 0;
	uint8_t over8 = ((pUSARTx->CR1 >> USART_CR1_OVER8) & (0x1));
	uint32_t mantissa = 0;
	uint32_t fraction = 0;
	uint32_t UsartDiv = 0;


	// Get APBx CLK
	if(pUSARTx == USART1 || pUSARTx == USART6) {
		APBxCLK = RCC_Get_PCLK2Freq();
	} else {
		APBxCLK = RCC_Get_PCLK1Freq();
	}

	// apply formula to get USARTDIV
	// multiply by 100 in order to not deal with floats
	UsartDiv = 100 * APBxCLK / ((8) * (2 - over8) * (BaudRate));
	// ex: USARTDIV = 4.56 multiply by 100 USARTDIV = 456
	// mantissa is just omitting the units and tens places
	mantissa = UsartDiv / 100;
	// modulo gives me the two first decimals
	// 16 - 8 * (over8) = 1 when over8=1 and 16 when over8=0
	// so multiply the decimal part by the oversampling setting
	// add 50 to round and divide by 100 (if decimal >= 50 gets rounded up, <= 50 rounds down)
	fraction = (((UsartDiv % 100) * (16 - 8 * (over8))) + 50) / 100;

	pUSARTx->BRR &= ~(0xffff << 0);
	uint16_t temp = ((mantissa << USART_BRR_DIV_MANTISSA) & (0xfff0));
	temp |= ((fraction << USART_BRR_DIV_FRACTION) & (0xf));

	pUSARTx->BRR |= temp;
}

/*
 * @fn				- USART_Init
 * @brief			- Initializes/Configures the USART peripheral
 * @param[in]		- Handle structure of the given peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void USART_Init(USART_Handle_t *pUSARTHandle) {
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_RE);
	} else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) {
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TE);
	} else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX) {
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_RE);
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TE);
	}

	pUSARTHandle->pUSARTx->CR2 &= ~(0x3 << USART_CR2_STOP);
	if(pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_0_5) {
		pUSARTHandle->pUSARTx->CR2 |= (0x1 << USART_CR2_STOP);
	} else if(pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_1) {
		pUSARTHandle->pUSARTx->CR2 |= (0x0 << USART_CR2_STOP);
	} else if(pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_1_5) {
		pUSARTHandle->pUSARTx->CR2 |= (0x3 << USART_CR2_STOP);
	} else if(pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_STOPBITS_2) {
		pUSARTHandle->pUSARTx->CR2 |= (0x2 << USART_CR2_STOP);
	}

	if(pUSARTHandle->USART_Config.USART_ParityControl != USART_PARITY_DISABLE) {
		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
			pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_PCE);
			pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_PS);
		} else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) {
			pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_PCE);
			pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_PS);
		}
	} else {
		pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_PCE);
	}

	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS) {
		pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_M);
	} else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_M);
	}

	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		pUSARTHandle->pUSARTx->CR3 |= (0x1 << USART_CR3_CTSE);
		pUSARTHandle->pUSARTx->CR3 &= ~(0x1 << USART_CR3_RTSE);
	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
		pUSARTHandle->pUSARTx->CR3 |= (0x1 << USART_CR3_RTSE);
		pUSARTHandle->pUSARTx->CR3 &= ~(0x1 << USART_CR3_CTSE);
	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		pUSARTHandle->pUSARTx->CR3 |= (0x1 << USART_CR3_RTSE);
		pUSARTHandle->pUSARTx->CR3 |= (0x1 << USART_CR3_CTSE);
	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_NONE) {
		pUSARTHandle->pUSARTx->CR3 &= ~(0x1 << USART_CR3_CTSE);
		pUSARTHandle->pUSARTx->CR3 &= ~(0x1 << USART_CR3_RTSE);
	}

	// BRR
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

	pUSARTHandle->TxState = USART_STATUS_READY;
	pUSARTHandle->RxState = USART_STATUS_READY;
}

/*
 * @fn				- USART_DeInit
 * @brief			- De-initializes/Resetss the USART peripheral
 * @param[in]		- Base address of the given peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void USART_DeInit(USART_RegDef_t *pUSARTx) {
	if(pUSARTx == USART1) {
		USART1_REG_RESET();
	} else if(pUSARTx == USART2) {
		USART2_REG_RESET();
	} else if(pUSARTx == USART3) {
		USART3_REG_RESET();
	} else if(pUSARTx == UART4) {
		UART4_REG_RESET();
	} else if(pUSARTx == UART5) {
		UART5_REG_RESET();
	} else if(pUSARTx == USART6) {
		USART6_REG_RESET();
	}
}


/*
 * @fn				- USART_SendData
 * @brief			- Sends data from pTxBuffer of length len bytes
 * @param[in]		- Handle structure of the peripheral
 * @param[in]		- Address of the buffer where the data is
 * @param[in]		- Length of the data
 *
 * @return			- none
 *
 * @Note			- Poll/Block Based
 *
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len) {
	uint16_t *pData;
	while(len > 0) {
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			pData = (uint16_t *) pTxBuffer;
			if(pUSARTHandle->USART_Config.USART_ParityControl != USART_PARITY_DISABLE) {
				// 9 bits but the MSB is parity. So 8 bits of data
				pUSARTHandle->pUSARTx->DR = (uint8_t) (*pTxBuffer);
				pTxBuffer++;
				len--;
			} else {
				// 9 bits no parity
				// so all 9 bits are data
				// user is responsible that the bits are positioned appropriately
				// tip: send word by word
				pUSARTHandle->pUSARTx->DR = (*pData & ((uint16_t)0x1ff));
				pTxBuffer++;
				pTxBuffer++;
				len--;
			}
		} else {
			// 8 bits
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
			len--;
		}
	}
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

/*
 * @fn				- USART_ReceiveData
 * @brief			- Receives data from the external world
 * @param[in]		- Handle structure of the peripheral
 * @param[in]		- Address of the buffer where the data will be stored
 * @param[in]		- Length of the data
 *
 * @return			- none
 *
 * @Note			- Poll/Block Based
 *
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len) {
	while(len > 0) {
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				// 9bits no parity
				*((uint16_t*) pRxBuffer) = pUSARTHandle->pUSARTx->DR & ((uint16_t) 0x1ff);
				pRxBuffer++;
				pRxBuffer++;
				len--;
			} else {
				// 9bits w parity
				*pRxBuffer = pUSARTHandle->pUSARTx->DR & ((uint8_t) 0xff);
				pRxBuffer++;
				len--;
			}
		} else {
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				// 8 bits no parity
				*pRxBuffer = ((uint8_t) pUSARTHandle->pUSARTx->DR) & ((uint8_t) 0xff);
			} else {
				// 8 bits w parity
				*pRxBuffer = ((uint8_t) pUSARTHandle->pUSARTx->DR & ((uint8_t) 0x7f));
			}
			pRxBuffer++;
			len--;
		}
	}
	//while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

/*
 * @fn				- EnableCommonInterrupts
 * @brief			- Enables some interrupts useful during reception or transmission
 * @param[in]		- Base address of the peripheral
 *
 * @return			- none
 *
 */
static void EnableCommonInterrupts(USART_RegDef_t *pUSARTx) {
	pUSARTx->CR3 |= (0x1 << USART_CR3_CTSIE);
	pUSARTx->CR1 |= (0x1 << USART_CR1_TCIE);
	pUSARTx->CR1 |= (0x1 << USART_CR1_IDLEIE);
	pUSARTx->CR1 |= (0x1 << USART_CR2_LBDIE);
}

/*
 * @fn				- USART_SendDataIT
 * @brief			- Sends data through USART from pTxBuffer of length len bytes
 * @param[in]		- The handle structure of the peripheral
 * @param[in]		- The buffer where the data is located
 * @param[in]		- The length of the data
 *
 * @return			- none
 *
 * @Note			- Interrupt Based
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len) {
	if(pUSARTHandle->TxState != USART_TXSTATUS_BUSY_IN_TX) {
		pUSARTHandle->TxState = USART_TXSTATUS_BUSY_IN_TX;
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->TxBuffer = pTxBuffer;

		// enable interrupts
		EnableCommonInterrupts(pUSARTHandle->pUSARTx);
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TXEIE);
	}
	return pUSARTHandle->TxState;
}

/*
 * @fn				- USART_ReceiveDataIT
 * @brief			- Reeives data through USART and stores in pRxBuffer.
 * @param[in]		- The handle structure of the peripheral
 * @param[in]		- The address where the data will be stored
 * @param[in]		- The length of the data
 *
 * @return			- none
 *
 * @Note			- Interrupt Based
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {
	if(pUSARTHandle->RxState != USART_RXSTATUS_BUSY_IN_RX) {
		pUSARTHandle->RxState = USART_RXSTATUS_BUSY_IN_RX;
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->RxBuffer = pRxBuffer;

		// enable interrupts
		EnableCommonInterrupts(pUSARTHandle->pUSARTx);
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_RXNEIE);
		//pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_PEIE);
	}
	return pUSARTHandle->RxState;
}

/*
 * @fn				- ClearOREFlag8Bits
 * @brief			- Clears the ORE (8 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint8_t ClearOREFlag8Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0xff));
}

/*
 * @fn				- ClearOREFlag9Bits
 * @brief			- Clears the ORE (16 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint16_t ClearOREFlag9Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0x1ff));
}

/*
 * @fn				- ClearIDLEFlag8Bits
 * @brief			- Clears the IDLE (8 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint8_t ClearIDLEFlag8Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0xff));
}

/*
 * @fn				- ClearIDLEFlag8Bits
 * @brief			- Clears the IDLE (9 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint16_t ClearIDLEFlag9Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0x1ff));
}

/*
 * @fn				- ClearPEFlag8Bits
 * @brief			- Clears the PE (8 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint8_t ClearPEFlag8Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0xff));
}

/*
 * @fn				- ClearPEFlag8Bits
 * @brief			- Clears the PE (9 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint16_t ClearPEFlag9Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0x1ff));
}

/*
 * @fn				- ClearNFFlag8Bits
 * @brief			- Clears the NF (8 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint8_t ClearNFFlag8Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0xff));
}

/*
 * @fn				- ClearNFFlag8Bits
 * @brief			- Clears the NF (9 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint16_t ClearNFFlag9Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0x1ff));
}

/*
 * @fn				- ClearFEFlag8Bits
 * @brief			- Clears the FE (8 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint8_t ClearFEFlag8Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0xff));
}

/*
 * @fn				- ClearFEFlag8Bits
 * @brief			- Clears the FE (9 bits) flag
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
uint16_t ClearFEFlag9Bits(USART_Handle_t *pHandle) {
	uint32_t dummy = pHandle->pUSARTx->SR;
	(void) dummy;
	return (pHandle->pUSARTx->DR & (0x1ff));
}

/*
 * @fn				- USART_IRQHandling
 * @brief			- Identifies which event caused the interrupt and acts accordingly
 * @param[in]		- The handle structure of the peripheral
 *
 * @return			- none
 *
 */
void USART_IRQHandling(USART_Handle_t *pHandle) {
	uint8_t interruptEnFlag = 0;
	uint8_t flag = 0;

	// RXNE & RXNEIE
	interruptEnFlag = ((pHandle->pUSARTx->CR1 >> USART_CR1_RXNEIE) & 0x1);
	flag = ((pHandle->pUSARTx->SR >> USART_SR_RXNE) & 0x1);
	if(interruptEnFlag && flag) {
		if(pHandle->RxState == USART_RXSTATUS_BUSY_IN_RX && pHandle->RxLen > 0) {
			if(pHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
				if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				// 9bits no parity
				*((uint16_t*) pHandle->RxBuffer) = pHandle->pUSARTx->DR & ((uint16_t) 0x1ff);
				pHandle->RxBuffer++;
				pHandle->RxBuffer++;
				pHandle->RxLen--;
				} else {
					// 9bits w parity
					*pHandle->RxBuffer = pHandle->pUSARTx->DR & ((uint8_t) 0xff);
					pHandle->RxBuffer++;
					pHandle->RxLen--;
				}
			} else {
				if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
					// 8 bits no parity
					*pHandle->RxBuffer = ((uint8_t) pHandle->pUSARTx->DR) & ((uint8_t) 0xff);
				} else {
					// 8 bits w parity
					*pHandle->RxBuffer = ((uint8_t) pHandle->pUSARTx->DR & ((uint8_t) 0x7f));
				}
				pHandle->RxBuffer++;
				pHandle->RxLen--;
			}
		}
		if(pHandle->RxLen <= 0) {
			// disable interrupts:
			pHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_RXNEIE);

			// reset handle
			pHandle->RxBuffer = NULL;
			pHandle->RxLen = 0;
			pHandle->RxState = USART_STATUS_READY;

			// notify
			USART_ApplicationEventCallback(pHandle, USART_EV_RX_CMPLT);
		}
	}

	// TXE & TXEIE
	interruptEnFlag = ((pHandle->pUSARTx->CR1 >> USART_CR1_TXEIE) & 0x1);
	flag = ((pHandle->pUSARTx->SR >> USART_SR_TXE) & 0x1);
	if(interruptEnFlag && flag) {
		if(pHandle->TxState == USART_TXSTATUS_BUSY_IN_TX && pHandle->TxLen > 0) {
			if(pHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS) {
				// 8 bits
				pHandle->pUSARTx->DR = (uint8_t) *pHandle->TxBuffer;
				pHandle->TxBuffer++;
				pHandle->TxLen--;
			} else {
				// 9bits
				if(pHandle->USART_Config.USART_ParityControl != USART_PARITY_DISABLE) {
					// 9bits parity enabled
					pHandle->pUSARTx->DR = *pHandle->TxBuffer;
					pHandle->TxBuffer++;
					pHandle->TxLen--;
				} else {
					// 9bits no parity
					pHandle->pUSARTx->DR = (*((uint16_t *) pHandle->TxBuffer) & ((uint16_t) 0x1ff));
					pHandle->TxBuffer++;
					pHandle->TxBuffer++;
					pHandle->TxLen--;
				}
			}
		}
	}

	// TC & TCIE
	interruptEnFlag = ((pHandle->pUSARTx->CR1 >> USART_CR1_TCIE) & 0x1);
	flag = ((pHandle->pUSARTx->SR >> USART_SR_TC) & 0x1);
	if(interruptEnFlag && flag) {
		if(pHandle->TxLen <= 0 && pHandle->TxState == USART_TXSTATUS_BUSY_IN_TX) {
			// clear TC flag
			//pHandle->pUSARTx->SR &= ~(0x1 << USART_SR_TC);

			// disable interrupts:
			pHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_TXEIE);
			pHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_TCIE);

			// reset handle
			pHandle->TxBuffer = NULL;
			pHandle->TxLen = 0;
			pHandle->TxState = USART_STATUS_READY;

			// notify
			USART_ApplicationEventCallback(pHandle, USART_EV_TX_CMPLT);
		}
	}

	if(pHandle->pUSARTx != UART4 && pHandle->pUSARTx != UART5) {
		// CTS & CTSIE
		interruptEnFlag = ((pHandle->pUSARTx->CR3 >> USART_CR3_CTSIE) & 0x1);
		flag = ((pHandle->pUSARTx->SR >> USART_SR_CTS) & 0x1);
		if(interruptEnFlag && flag) {
			pHandle->pUSARTx->SR &= ~(0x1 << USART_SR_CTS);
			USART_ApplicationEventCallback(pHandle, USART_EV_CTS);
		}
	}


	// ORE & RXNEIE
	interruptEnFlag = ((pHandle->pUSARTx->CR1 >> USART_CR1_RXNEIE) & 0x1);
	flag = ((pHandle->pUSARTx->SR >> USART_SR_ORE) & 0x1);
	if(flag && interruptEnFlag) {
		USART_ApplicationEventCallback(pHandle, USART_EV_ORE);
	}

	// IDLE & IDLEIE
	interruptEnFlag = ((pHandle->pUSARTx->CR1 >> USART_CR1_IDLEIE) & 0x1);
	flag = ((pHandle->pUSARTx->SR >> USART_SR_IDLE) & 0x1);
	if(flag && interruptEnFlag) {
		USART_ApplicationEventCallback(pHandle, USART_EV_IDLE);
	}

	// PE & PEIE
	interruptEnFlag = ((pHandle->pUSARTx->CR1 >> USART_CR1_PEIE) & 0x1);
	flag = ((pHandle->pUSARTx->SR >> USART_SR_PE) & 0x1);
	if(flag && interruptEnFlag) {
		USART_ApplicationEventCallback(pHandle, USART_EV_PE);
	}

	// LBD & LBDIE
	interruptEnFlag = ((pHandle->pUSARTx->CR2 >> USART_CR2_LBDIE) & 0x1);
	flag = ((pHandle->pUSARTx->SR >> USART_SR_LBD) & 0x1);
	if(flag && interruptEnFlag) {
		pHandle->pUSARTx->SR &= ~(0x1 << USART_SR_LBD);
		USART_ApplicationEventCallback(pHandle, USART_EV_LBD);
	}

	// multibuffer errors
	interruptEnFlag = ((pHandle->pUSARTx->CR3 >> USART_CR3_EIE) & 0x1);
	if(interruptEnFlag) {
		// NF
		flag = ((pHandle->pUSARTx->SR >> USART_SR_NF) & 0x1);
		if(flag) {
			USART_ApplicationEventCallback(pHandle, USART_EV_NF);
		}

		// ORE
		flag = ((pHandle->pUSARTx->SR >> USART_SR_ORE) & 0x1);
		if(flag) {
			USART_ApplicationEventCallback(pHandle, USART_EV_ORE);
		}

		// FE
		flag = ((pHandle->pUSARTx->SR >> USART_SR_FE) & 0x1);
		if(flag) {
			USART_ApplicationEventCallback(pHandle, USART_EV_FE);
		}
	}


}

/*
 * @fn				- USART_IRQInterruptConfig
 * @brief			- Enables or Disables interrupts for the given IRQNumber
 * @param[in]		- The IRQNumber
 * @param[in] 		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		*(NVIC_ISER0 + IRQNumber / 32) |= (0x1 << (IRQNumber % 32));
	} else {
		*(NVIC_ISER0 + IRQNumber / 32) &= ~(0x1 << (IRQNumber % 32));
	}
}

/*
 * @fn				- USART_IRQPriorityConfig
 * @brief			- Sets the priority for the given IRQ Number
 * @param[in]		- The IRQNumber
 * @param[in] 		- The priority level
 *
 * @return			- none
 *
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	*(NVIC_IPR0 + IRQNumber / 4) &= ~(0xff << (IRQNumber % 4) * 8);
	*(NVIC_IPR0 + IRQNumber / 4) |= (IRQPriority << ((IRQNumber % 4) * 8 + 8 - PRIO_BITS_IMPLEMENTED));
}

/*
 * Other Peripheral Control APIs
 */

/*
 * @fn				- USART_PeripheralControl
 * @brief			- Enables or Disables USART communication
 * @param[in]		- The base address of the relevant USART
 * @param[in] 		- ENABLE or DISABLE macro
 *
 * @return			- none
 *
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pUSARTx->CR1 |= (0x1 << USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(0x1 << USART_CR1_UE);
	}
}

/*
 * @fn				- USART_GetFlagStatus
 * @brief			- Retrieves the status of a flag in the SR register
 * @param[in]		- The base address of the relevant USART
 * @param[in] 		- FlagName macro
 *
 * @return			- none
 *
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName) {
	return (pUSARTx->SR >> FlagName) & (0x1);
}

/*
 * @fn				- USART_ApplicationEventCallback
 * @brief			- Weak function that notifies the user of events related to the relevant USART
 * @param[in]		- The base address of the relevant USART
 * @param[in] 		- USART_EV Macro
 *
 * @return			- none
 *
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv) {
	return;
}
