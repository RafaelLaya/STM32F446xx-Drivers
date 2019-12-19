/*
 * stm32f446xx.h_I2C_driver.c
 *
 *  Created on: Jul 29, 2019
 *      Author: rafae
 */

#include "stm32f446xx.h"

/*
 * @fn				- I2C_PeripheralClockControl
 * @brief			- Enables or Disables Clock to given I2C peripheral
 * @param[in]		- Base address of the USART Peripheral
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		if(pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if(pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if(pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if(pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if(pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if(pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

/*
 * @fn				- I2C_GetSR1FlagStatus
 * @brief			- Retrieves the status of a flag in the SR1 register
 * @param[in]		- Base address of the USART Peripheral
 * @param[in]		- The FlagName macro (bit position of the flag)
 *
 * @return			- The status of the flag
 *
 * @Note			- none
 *
 */
uint8_t I2C_GetSR1FlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if(pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * @fn				- I2C_Init
 * @brief			- Initializes/Configures the I2C peripheral
 * @param[in]		- The handle structure of the relevant I2C peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	// enable clock
	I2C_PeripheralClockControl(pI2CHandle->pI2Cx, ENABLE);

	// calculate P1CLK for FREQ
	uint32_t P1CLK = RCC_Get_PCLK1Freq();
	pI2CHandle->pI2Cx->CR2 &= ~(63 << I2C_CR2_FREQ);
	pI2CHandle->pI2Cx->CR2 |= (((P1CLK / 1000000) << I2C_CR2_FREQ) & (0x3F));

	// address
	pI2CHandle->pI2Cx->OAR1 &= ~(0x7f << 1);
	pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	pI2CHandle->pI2Cx->OAR1 &= ~(0x1 << 15);
	pI2CHandle->pI2Cx->OAR1 |= (0x1 << 14);

	// CCR Calculation
	uint16_t ccr = 0;

	pI2CHandle->pI2Cx->CCR &= ~(0xfff << 0);

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// SM
		pI2CHandle->pI2Cx->CCR &= ~(0x1 << 15);
		// Thigh = CCR * Tpclk1
		// Tlow = CCR * Tpclk1
		// Thigh + Tlow = Tscl = 2 * CCR * Tpclk1
		// => CCR = Tscl / (2 * Tpclk1)
		// => CCR = fclk1 / (2 * fscl)
		// fscl = frequency requested by user for serial clock
		// fclk1 is the frequency at the APB bus
		ccr = RCC_Get_PCLK1Freq() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
	} else {
		// FM
		pI2CHandle->pI2Cx->CCR |= (0x1 << 15);
		// DutyCycle
		pI2CHandle->pI2Cx->CCR &= ~(0x1 << I2C_CCR_DUTY);
		pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2C_Config.FMDutyCycle == I2C_FM_DUTY_2) {
			// Thigh = CCR * Tpclk1
			// Tlow = 2 * CCr * Tpclk1
			// Thigh + Tlow = Tsclk = 3 * CCR * Tpclk1
			// CCR = Tsclk / (3 * Tpclk1)
			// CCR = fpclk1 / (3 * fsclk)
			ccr = RCC_Get_PCLK1Freq() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
			// Thigh = 9 * CCR * Tpclk1
			// Tlow = 16 * CCR * Tpclk1
			// Thigh + Tlow = 25 * CCR * Tpclk1 = Tsclk
			// 25 * CCR / fpclk1 = 1/fsclk
			// CCR = fpclk1 / (25 * fsclk)
			ccr = RCC_Get_PCLK1Freq() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
	}
	pI2CHandle->pI2Cx->CCR |= ((ccr << 0) & (0xfff));
	uint8_t tRise;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
		// tRise = Tmax/Tpclk1 + 1 = Tmax * Fpclk1 + 1; Tmax = 1000*10^-9 = 1/1000000
		tRise = (RCC_Get_PCLK1Freq() / 1000000U) + 1;
	} else {
		tRise = (RCC_Get_PCLK1Freq() * 3 / 10000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE &= ~(0x3f << 0);
	pI2CHandle->pI2Cx->TRISE |= ((tRise << 0) & (0x3f));
}

/*
 * @fn				- I2C_DeInit
 * @brief			- De-initializes/Resets the I2C peripheral
 * @param[in]		- Base address of the peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if(pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if(pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if(pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/*
 * @fn				- I2C_GenerateStartCondition
 * @brief			- Generates the start condition (master)
 * @param[in]		- Handle structure of the I2C peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void I2C_GenerateStartCondition(I2C_Handle_t *pI2CHandle) {
	pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_START);
}

/*
 * @fn				- I2C_ExecuteAddressPhase
 * @brief			- Executes the address phase
 * @param[in]		- Base address of the peripheral
 * @param[in]		- The address of the slave
 * @param[in]		- The RW_Bit (Read or Write) macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t RW_Bit) {
	if(RW_Bit == I2C_RW_BIT_WRITE) {
		pI2Cx->DR = ((SlaveAddr << 1) & ~(0x1));
	} else {
		pI2Cx->DR = ((SlaveAddr << 1) | (0x1));
	}
}

/*
 * @fn				- I2C_ClearADDRFlag
 * @brief			- Clear the ADDR flag
 * @param[in]		- Base address of the peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx) {
	uint32_t dummy_read = pI2Cx->SR1;
	dummy_read = pI2Cx->SR2;
	(void) dummy_read;
}

/*
 * @fn				- I2C_GenerateStopCondition
 * @brief			- Generates the stop condition (master)
 * @param[in]		- Handle structure of the I2C peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void I2C_GenerateStopCondition(I2C_Handle_t *pI2CHandle) {
	pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);
}

/*
 * @fn				- I2C_MasterSendData
 * @brief			- Sends data through I2C
 * @param[in]		- Handle structure of the I2C peripheral
 * @param[in]		- Points to where the data is
 * @param[in]		- The length of the data
 * @param[in]		- The address of the slave who will receive the data
 * @param[in]		- The repeated start macro
 *
 * @return			- none
 *
 * @Note			- Block/Poll based
 *
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t RepeatedStart) {
	// wait for bus to be free (this is done by hardware)
	// start condition
	I2C_GenerateStartCondition(pI2CHandle);

	// wait for EV5 and clear by software sequence (cleared by reading SR1 and writing address to DR)
	while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// write address and read/write (write = 0) to the DR
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, I2C_RW_BIT_WRITE);
	
	// wait for EV6, clear by reading SR1 and then SR2
	// wait for ADDR=1 and clear
	while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	
	// wait for EV8_1 write first byte to DR
	while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
	pI2CHandle->pI2Cx->DR = *pTxBuffer;
	pTxBuffer++;
	len--;
		
	while(len > 0) {
		// wait for EV8, write to DR
		while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}
	
	while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
	while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	if(RepeatedStart == I2C_REPEATED_START_NO) {
		I2C_GenerateStopCondition(pI2CHandle);
	}
}

/*
 * @fn				- I2C_ManageAcking
 * @brief			- Enables or Disables ACK Control
 * @param[in]		- Base address of the peripheral
 * @param[in]		- ENABLE or DISABLE macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if(EnOrDi == I2C_ACK_ENABLE) {
		pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);
	} else {
		pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);
	}
}

/*
 * @fn				- I2C_RestoreAcking
 * @brief			- Restores ACK control to its configured value
 * @param[in]		- Handle structure of the peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void I2C_RestoreAcking(I2C_Handle_t *pI2CHandle) {
	I2C_ManageAcking(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.ACKControl);
}

/*
 * @fn				- I2C_MasterReceiveData
 * @brief			- receives data through I2C
 * @param[in]		- Handle structure of the I2C peripheral
 * @param[in]		- Points to where the data will be stored
 * @param[in]		- The length of the data
 * @param[in]		- The address of the slave who will send the data
 * @param[in]		- The repeated start macro
 *
 * @return			- none
 *
 * @Note			- Block/Poll based
 *
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t RepeatedStart) {
	// wait until bus is free
	// this is done by hardware when start is requested:
	// so request start condition
	I2C_GenerateStartCondition(pI2CHandle);

	// wait for EV5
	while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// clear SB by reading SR1 and writing to DR. Read has been done (otherwise we would still be in the loop!
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, I2C_RW_BIT_READ);

	// wait for EV6
	while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// here is where len == 1 and len == 2 part ways :(
	if(len == 1) {
		// disable acknowledge
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		// clear ADDR flag by reading SR1 and SR2
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// wait for EV7 RXNE flag
		while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));
		if(RepeatedStart == I2C_REPEATED_START_NO) {
			// Request stop
			I2C_GenerateStopCondition(pI2CHandle);
		}

		// read byte
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		// return but first restore acking to its original value before this function ran
		I2C_RestoreAcking(pI2CHandle);
		return;
	} else {
		// clear ADDR by reading SR1 and SR2
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// loop until len == 2
		while(len > 2) {
			// wait for EV7 RXNE flag
			while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));
			// read data
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			// decrease length
			len--;
			// advance buffer
			pRxBuffer++;
		}
		// when len == 2
		// wait for EV7_1 RXNE flag
		while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));
		// acknowledge clear
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		if(RepeatedStart == I2C_REPEATED_START_NO) {
			// request stop
			I2C_GenerateStopCondition(pI2CHandle);
		}
		// read second last byte
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		// increase buffer
		pRxBuffer++;
		// decrease len
		len--;
		// wait for EV7 RXNE flag
		while(!I2C_GetSR1FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));
		// read last byte
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		// return but first restore acking to its original value before this function ran
		I2C_RestoreAcking(pI2CHandle);
		return;
	}
}

/*
 * @fn				- I2C_EnableIT_EV, I2C_EnableIT_BU, I2C_EnableIT_ERR, I2C_EnableIT_EV,
 * 					  I2C_DisableIT_EV, I2C_DisableIT_BU, I2C_DisableIT_ERR, I2C_DisableIT_EV
 * @brief			- Enable/Disable an interrupt
 * @param[in]		- Base address to the relevant I2C peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void static I2C_EnableIT_EV(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVTEN);
}

void static I2C_EnableIT_BU(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);
}
void static I2C_EnableIT_ERR(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);
}

void static I2C_DisableIT_EV(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);
}

void static I2C_DisableIT_BU(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);
}
void static I2C_DisableIT_ERR(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITERREN);
}

/*
 * @fn				- I2C_SlaveConfigureAllback
 * @brief			- Enable/Disable all interrupts
 * @param[in]		- Base address to the relevant I2C peripheral
 * @param[in]		- ENABLE or DISABLE macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_SlaveConfigureAllback(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		I2C_EnableIT_All(pI2Cx);
	} else {
		I2C_DisableIT_All(pI2Cx);
	}
}

/*
 * @fn				- I2C_EnableIT_All
 * @brief			- Enable all interrupts
 * @param[in]		- Base address to the relevant I2C peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_EnableIT_All(I2C_RegDef_t *pI2Cx) {
	I2C_EnableIT_BU(pI2Cx);
	I2C_EnableIT_ERR(pI2Cx);
	I2C_EnableIT_EV(pI2Cx);

}

/*
 * @fn				- I2C_DisableIT_All
 * @brief			- Disable all interrupts
 * @param[in]		- Base address to the relevant I2C peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_DisableIT_All(I2C_RegDef_t *pI2Cx) {
	I2C_DisableIT_BU(pI2Cx);
	I2C_DisableIT_ERR(pI2Cx);
	I2C_DisableIT_EV(pI2Cx);

}

/*
 * @fn				- I2C_MasterSendDataIT
 * @brief			- Sends data through I2C
 * @param[in]		- Handle structure of the I2C peripheral
 * @param[in]		- Points to where the data is stored
 * @param[in]		- The length of the data
 * @param[in]		- The address of the slave who will receive the data
 * @param[in]		- The repeated start macro
 *
 * @return			- status of peripheral
 *
 * @Note			- Interrupt based
 *
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t RepeatedStart) {
	if(pI2CHandle->I2C_Config.RxTxStatus != I2C_STATUS_BUSY_IN_RX && pI2CHandle->I2C_Config.RxTxStatus != I2C_STATUS_BUSY_IN_TX) {
		pI2CHandle->I2C_Config.pTxBuffer = pTxBuffer;
		pI2CHandle->I2C_Config.TxLen = len;
		pI2CHandle->I2C_Config.Target_Address = SlaveAddr;
		pI2CHandle->I2C_Config.RepeatedStart = RepeatedStart;
		pI2CHandle->I2C_Config.RxTxStatus = I2C_STATUS_BUSY_IN_TX;

		I2C_GenerateStartCondition(pI2CHandle);

		I2C_EnableIT_All(pI2CHandle->pI2Cx);
	}
	return pI2CHandle->I2C_Config.RxTxStatus;
}

/*
 * @fn				- I2C_MasterReceiveDataIT
 * @brief			- Receives data through I2C
 * @param[in]		- Handle structure of the I2C peripheral
 * @param[in]		- Points to where the data will be stored
 * @param[in]		- The length of the data
 * @param[in]		- The address of the slave who will send the data
 * @param[in]		- The repeated start macro
 *
 * @return			- status of peripheral
 *
 * @Note			- Interrupt based
 *
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t RepeatedStart) {
	if(pI2CHandle->I2C_Config.RxTxStatus != I2C_STATUS_BUSY_IN_RX && pI2CHandle->I2C_Config.RxTxStatus != I2C_STATUS_BUSY_IN_TX) {
		pI2CHandle->I2C_Config.pRxBuffer = pRxBuffer;
		pI2CHandle->I2C_Config.RxLen = len;
		pI2CHandle->I2C_Config.RxSize = len;
		pI2CHandle->I2C_Config.Target_Address = SlaveAddr;
		pI2CHandle->I2C_Config.RepeatedStart = RepeatedStart;
		pI2CHandle->I2C_Config.RxTxStatus = I2C_STATUS_BUSY_IN_RX;

		I2C_GenerateStartCondition(pI2CHandle);

		I2C_EnableIT_All(pI2CHandle->pI2Cx);
	}
	return pI2CHandle->I2C_Config.RxTxStatus;
}

/*
 * @fn				- I2C_IRQInterruptConfig
 * @brief			- Enables or Disables interrupts for the given IRQNumber
 * @param[in]		- The IRQNumber
 * @param[in] 		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		*(NVIC_ISER0 + IRQNumber / 32) |= (0x1 << (IRQNumber % 32));
	} else {
		*(NVIC_ISER0 + IRQNumber / 32) &= ~(0x1 << (IRQNumber % 32));
	}
}

/*
 * @fn				- I2C_IRQPriorityConfig
 * @brief			- Sets the priority of the interrupt for this peripheral
 * @param[in]		- The IRQNumber
 * @param[in] 		- Configuration priority
 *
 * @return			- none
 *
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	*(NVIC_IPR0 + IRQNumber / 4) &= ~(0xff << ((IRQNumber % 4) * 8));
	*(NVIC_IPR0 + IRQNumber / 4) |= (IRQPriority << (((IRQNumber % 4) * 8) + 8 - PRIO_BITS_IMPLEMENTED));
}

/*
 * @fn				- I2C_PeripheralControl
 * @brief			- Enables or disables I2C communication
 * @param[in]		- The base address of the relevant peripheral
 * @param[in] 		- ENABLE or DISABLE macro
 *
 * @return			- none
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (0x1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(0x1 << I2C_CR1_PE);
	}
}

/*
 * @fn				- I2C_CloseMasterTx
 * @brief			- Disables interrupts and resets transmission related fields
 * @param[in]		- Handle structure of the peripheral
 *
 * @return			- none
 *
 */
void I2C_CloseMasterTx(I2C_Handle_t *pI2CHandle) {
	I2C_DisableIT_EV(pI2CHandle->pI2Cx);
	I2C_DisableIT_BU(pI2CHandle->pI2Cx);
	I2C_DisableIT_ERR(pI2CHandle->pI2Cx);

	pI2CHandle->I2C_Config.TxLen = 0;
	pI2CHandle->I2C_Config.pTxBuffer = NULL;
	pI2CHandle->I2C_Config.RxTxStatus = I2C_STATUS_READY;
}

/*
 * @fn				- I2C_CloseMasterRx
 * @brief			- Disables interrupts and resets reception related fields
 * @param[in]		- Handle structure of the peripheral
 *
 * @return			- none
 *
 */
void I2C_CloseMasterRx(I2C_Handle_t *pI2CHandle) {
	// disable ITBUFEN and ITEVFEN and ITERR

	I2C_DisableIT_EV(pI2CHandle->pI2Cx);
	I2C_DisableIT_BU(pI2CHandle->pI2Cx);
	I2C_DisableIT_ERR(pI2CHandle->pI2Cx);

	pI2CHandle->I2C_Config.pRxBuffer = NULL;
	pI2CHandle->I2C_Config.RxLen = 0;
	pI2CHandle->I2C_Config.RxSize = 0;
	I2C_RestoreAcking(pI2CHandle);
	pI2CHandle->I2C_Config.RxTxStatus = I2C_STATUS_READY;
}

/*
 * @fn				- I2C_SlaveSendData
 * @brief			- Sends data when requested by a master
 * @param[in]		- Handle structure of the peripheral
 * @param[in]		- Data to send
 *
 * @return			- none
 *
 */
void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle, uint8_t data) {
	pI2CHandle->pI2Cx->DR = data;
}

/*
 * @fn				- I2C_SlaveReceiveData
 * @brief			- Receives data sent by a master
 * @param[in]		- Handle structure of the peripheral
 *
 * @return			- the received data
 *
 */
uint8_t I2C_SlaveReceiveData(I2C_Handle_t *pI2CHandle) {
	return (uint8_t) pI2CHandle->pI2Cx->DR;
}

/*
 * @fn				- I2C_ClearSTOPF
 * @brief			- Clears STOPF flag
 * @param[in]		- Handle structure of the peripheral
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void I2C_ClearSTOPF(I2C_Handle_t *pI2CHandle) {
	uint8_t dummy_read;
	dummy_read = (uint8_t) pI2CHandle->pI2Cx->SR1;
	pI2CHandle->pI2Cx->CR1 |= 0x0000;
	(void) dummy_read;
}

/*
 * @fn				- I2C_EV_IRQhandling
 * @brief			- Figures out why interrupt occurred and act accordingly
 * @param[in]		- Handle structure of the peripheral
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void I2C_EV_IRQhandling(I2C_Handle_t *pI2CHandle) {
	uint8_t ITEVFEN = ((pI2CHandle->pI2Cx->CR2 >> I2C_CR2_ITEVTEN) & (0x1));
	uint8_t ITBUFEN = ((pI2CHandle->pI2Cx->CR2 >> I2C_CR2_ITBUFEN) & (0x1));
	uint8_t MSL = ((pI2CHandle->pI2Cx->SR2 >> I2C_SR2_MSL) & (0x1));
	uint8_t RxTxStatus = pI2CHandle->I2C_Config.RxTxStatus;

	// SBFlag
	uint8_t EventFlag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_SB) & (0x1));
	if(ITEVFEN && EventFlag) {
		if(MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_TX)) {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.Target_Address, I2C_RW_BIT_WRITE);
		} else if(MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_RX)) {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.Target_Address, I2C_RW_BIT_READ);
		} else if(!MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_TX)) {

		} else if(!MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_RX)) {

		}
	}

	// ADDR
	EventFlag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_ADDR) & (0x1));
	if(ITEVFEN && EventFlag) {
		if(MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_TX)) {
			I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		} else if(MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_RX)) {
			if(pI2CHandle->I2C_Config.RxSize == 1) {
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
			}
			I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		} else if(!MSL){
			I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		}
	}

	// BTF
	EventFlag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_BTF) & (0x1));
	if(ITEVFEN && EventFlag) {
		if(MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_TX) && (pI2CHandle->I2C_Config.TxLen <= 0)) {
			uint8_t TxEFlag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_TxE) & (0x1));
			if(TxEFlag) {
				if(pI2CHandle->I2C_Config.RepeatedStart == I2C_REPEATED_START_NO) {
					I2C_GenerateStopCondition(pI2CHandle);
				}
				I2C_CloseMasterTx(pI2CHandle);
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				return;
			}
		} else if(MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_RX)) {

		} else if(!MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_TX)) {

		} else if(!MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_RX)) {

		}
	}

	// STOPF
	uint8_t Transmitting = ((pI2CHandle->pI2Cx->SR2 >> I2C_SR2_TRA) & 0x1);
	EventFlag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_STOPF) & (0x1));
	if(ITEVFEN && EventFlag) {
		if(!MSL && Transmitting) {

		} else if(!MSL && !Transmitting) {
			I2C_ClearSTOPF(pI2CHandle);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOPF);
		}
	}

	// TxE
	EventFlag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_TxE) & (0x1));
	if(ITEVFEN && ITBUFEN && EventFlag) {
		if(MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_TX) && (pI2CHandle->I2C_Config.TxLen > 0)) {
			pI2CHandle->pI2Cx->DR = *(pI2CHandle->I2C_Config.pTxBuffer);
			pI2CHandle->I2C_Config.TxLen--;
			pI2CHandle->I2C_Config.pTxBuffer++;
			// if len == 0 and BTF, that is handled by the BTF block which
			// returns and impedes this one from running when not desired
		} else if(MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_RX)) {

		} else if(!MSL && Transmitting) {
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		} else if(!MSL && !Transmitting) {

		}
	}

	EventFlag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_RxNE) & (0x1));
	// RxNE
	if(ITEVFEN && ITBUFEN && EventFlag) {
		if(MSL && (RxTxStatus == I2C_STATUS_BUSY_IN_TX)) {

		} else if((MSL || pI2CHandle->I2C_Config.RxLen == 1) && (RxTxStatus == I2C_STATUS_BUSY_IN_RX)) {
			if(pI2CHandle->I2C_Config.RxLen == 2) {
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				if(pI2CHandle->I2C_Config.RepeatedStart == I2C_REPEATED_START_NO) {
					I2C_GenerateStopCondition(pI2CHandle);
				}
			} else if(pI2CHandle->I2C_Config.RxSize == 1) {
				if(pI2CHandle->I2C_Config.RepeatedStart == I2C_REPEATED_START_NO) {
					// stop
					I2C_GenerateStopCondition(pI2CHandle);
				}
			}
			*(pI2CHandle->I2C_Config.pRxBuffer) = pI2CHandle->pI2Cx->DR;
			pI2CHandle->I2C_Config.RxLen--;
			pI2CHandle->I2C_Config.pRxBuffer++;
			if(pI2CHandle->I2C_Config.RxLen == 0) {
				I2C_CloseMasterRx(pI2CHandle);
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				return;
			}
		} else if(!MSL && Transmitting) {

		} else if(!MSL && !Transmitting) {
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCVD);
		}
	}
}

/*
 * @fn				- I2C_ERR_IRQHandling
 * @brief			- Figures out why interrupt occurred and notify the user application
 * @param[in]		- Handle structure of the peripheral
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void I2C_ERR_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint8_t ITERR = ((pI2CHandle->pI2Cx->CR2 >> I2C_CR2_ITERREN) & (0x1));

	// BERR
	uint8_t Flag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_BERR) & (0x1));
	if(ITERR && Flag) {
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_BERR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_BERR);
	}

	// ARLO
	Flag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_ARLO) & (0x1));
	if(ITERR && Flag) {
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_ARLO);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_ARLO);
	}

	// AF
	Flag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_AF) & (0x1));
	if(ITERR && Flag) {
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_AF);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_AF);
	}

	// OVR
	Flag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_OVR) & (0x1));
	if(ITERR && Flag) {
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_OVR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_OVR);
	}

	// PECERR
	Flag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_PECERR) & (0x1));
	if(ITERR && Flag) {
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_PECERR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_PECERR);
	}

	// TIMEOUT
	Flag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_TIMEOUT) & (0x1));
	if(ITERR && Flag) {
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_TIMEOUT);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TIMEOUT);
	}

	// SMBALERT
	Flag = ((pI2CHandle->pI2Cx->SR1 >> I2C_SR1_SMBALERT) & (0x1));
	if(ITERR && Flag) {
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_SMBALERT);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_SMBALERT);
	}
}

/*
 * @fn				- I2C_ApplicationEventCallback
 * @brief			- Weak function that notifies the user of events related to the relevant I2C
 * @param[in]		- The base address of the relevant USART
 * @param[in] 		- I2C_EV Macro
 *
 * @return			- none
 *
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

}
