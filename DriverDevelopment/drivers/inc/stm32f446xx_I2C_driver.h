/*
 * stm32f446xx_I2C_driver.h
 *
 *  Created on: Jul 29, 2019
 *      Author: rafae
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

typedef struct {
	uint32_t I2C_SCLSpeed;			/* @I2C_SCLSpeed */
	uint8_t I2C_DeviceAddress;		/* @I2C_DeviceAddress */
	uint8_t ACKControl;				/* @ACKControl */
	uint16_t FMDutyCycle;			/* @FMDutyCycle */

	uint8_t RxTxStatus;
	uint8_t *pRxBuffer;
	uint8_t *pTxBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t Address;				/* Address of the slave that this master is communicating with */
	uint8_t RepeatedStart;
	uint32_t RxSize;
} I2C_Config_t;

typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

} I2C_Handle_t;

#define I2C_RW_BIT_READ				1
#define I2C_RW_BIT_WRITE			0

#define I2C_STATUS_READY			0
#define I2C_STATUS_BUSY_IN_RX		1
#define I2C_STATUS_BUSY_IN_TX		2

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM			100000
#define I2C_SCL_SPEED_FM2K			200000
#define I2C_SCLSpeed_FM4k			400000

/*
 * @ACKControl
 */
#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0

/*
 * FMDutyCycle
 */
#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1

/*
 * I2C Flags
 */
#define I2C_FLAG_SB					(0x1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR				(0x1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF				(0x1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF				(0x1 << I2C_SR1_STOPF)
#define I2C_FLAG_RxNE				(0x1 << I2C_SR1_RxNE)
#define I2C_FLAG_TxE				(0x1 << I2C_SR1_TxE)
#define I2C_FLAG_BERR				(0x1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO				(0x1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF					(0x1 << I2C_SR1_AF)
#define I2C_FLAG_OVR				(0x1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR				(0x1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT			(0x1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT			(0x1 << I2C_SR1_SMBALERT)

#define I2C_REPEATED_START_NO			RESET
#define I2C_REPEATED_START_YES			SET

/*
 * I2C application macros
 */
#define I2C_EV_TX_CMPLT				0
#define I2C_EV_RX_CMPLT				1
#define I2C_EV_STOPF				2
#define I2C_EV_BERR					3
#define I2C_EV_ARLO					4
#define I2C_EV_AF					5
#define I2C_EV_OVR					6
#define I2C_EV_PECERR				7
#define I2C_EV_TIMEOUT				8
#define I2C_EV_SMBALERT				9
#define I2C_EV_DATA_REQ				10
#define I2C_EV_DATA_RCVD			11

/******************** APIs ***************************/

void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t RepeatedStart);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t RepeatedStart);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t RepeatedStart);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t RepeatedStart);

void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_Handle_t *pI2CHandle);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQhandling(I2C_Handle_t *pI2CHandle);
void I2C_ERR_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetSR1FlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
void I2C_SlaveConfigureCallback(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_EnableIT_All(I2C_RegDef_t *pI2Cx);
void I2C_DisableIT_All(I2C_RegDef_t *pI2Cx);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
