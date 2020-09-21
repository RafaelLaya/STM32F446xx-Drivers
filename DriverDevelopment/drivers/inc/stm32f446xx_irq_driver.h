/*
 * stm32f446xx_irq_driver.h
 *
 *  Created on: Dec 16, 2019
 *      Author: Stephen Street (stephen@shapertools.com)
 *
 *      This driver allows dynamic installation and removal of STM32F4
 *      interrupt handlers and allows the user to provide the driver
 *      opaque pointer which is sent along when ever the interrupt is
 *      triggered
 */

#ifndef _STM32F446XX_IRQ_DRIVER_H_
#define _STM32F446XX_IRQ_DRIVER_H_

#include "stm32f446xx.h"
#include <stdint.h>

#define IRQ_SUCCESS				 1
#define IRQ_NO_SUCCESS 			-1

/* Allow the use of cmsis interrupt types which are undefined in the project
 * the valid range is -14 to to the last STMF446xx device interrupt with 0
 * being the first STMF446xx device interrupt */
/* Prototype for new interruption handler */
typedef void (*IRQ_Handler)(IRQn_Type irq, void *data);

/* Enable and disable a specific interrupt */
void IRQ_Enable(IRQn_Type irq);
void IRQ_Disable(IRQn_Type irq);

/* Get or set the specific opaque data for an interrupt */
void IRQ_SetData(IRQn_Type irq, void *data);
void *IRQ_GetData(IRQn_Type irq);

/* Get or set the specific interrupt priority */
void IRQ_SetPriority(IRQn_Type irq, uint32_t priority);
uint32_t IRQ_GetPriority(IRQn_Type irq);

/* Dynamically register or unregister an interrupt handler */
int IRQ_Register(IRQn_Type irq, IRQ_Handler handler, uint32_t priority, void *data);
int IRQ_Unregister(int irq);

/* Initialize the dynamic interrupt handling driver */
int IRQ_Init(void);

/* Default IRQ_Handler calls Default_Handler() */
void Default_Handler_Wrapper(IRQn_Type irq, void *data);

#endif
