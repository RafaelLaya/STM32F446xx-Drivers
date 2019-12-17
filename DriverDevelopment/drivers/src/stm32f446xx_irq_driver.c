/*
 * stm32f446xx_irq.c
 *
 *  Created on: Dec 16, 2019
 *      Author: Stephen Street (stephen@shapertools.com)
 */

#include <stm32f446xx_irq_driver.h>

void IRQ_Dispatch(void);

void IRQ_Dispatch(void)
{
	/* Hint the SCB (System Control Block) ICSR (Interrupt Control and State Register) contains the currently running interrupt */
}
/* These overide the STDM32F446xx interrupt handling function is a standard startup file and force IRQ_Dispatch to be called on all interrutps */
void NMI_Handler(void) __attribute__((alias("IRQ_Dispatch")));
void HardFault_Handler(void) __attribute__((alias("IRQ_Dispatch")));
void MemManage_Handler(void) __attribute__((alias("IRQ_Dispatch")));
void BusFault_Handler(void) __attribute__((alias("IRQ_Dispatch")));
void UsageFault_Handler(void) __attribute__((alias("IRQ_Dispatch")));
void SVC_Handler(void) __attribute__((alias("IRQ_Dispatch")));
void DebugMon_Handler(void) __attribute__((alias("IRQ_Dispatch")));
void PendSV_Handler(void) __attribute__((alias("IRQ_Dispatch")));
void SysTick_Handler(void) __attribute__((alias("IRQ_Dispatch")));
void WWDG_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void PVD_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TAMP_STAMP_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void RTC_WKUP_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void FLASH_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void RCC_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void EXTI0_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void EXTI1_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void EXTI2_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void EXTI3_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void EXTI4_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA1_Stream0_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA1_Stream1_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA1_Stream2_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA1_Stream3_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA1_Stream4_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA1_Stream5_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA1_Stream6_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void ADC_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void CAN1_TX_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void CAN1_RX0_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void CAN1_RX1_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void CAN1_SCE_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void EXTI9_5_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM1_BRK_TIM9_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM1_UP_TIM10_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM1_TRG_COM_TIM11_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM1_CC_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM2_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM3_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM4_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void I2C1_EV_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void I2C1_ER_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void I2C2_EV_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void I2C2_ER_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void SPI1_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void SPI2_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void USART1_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void USART2_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void USART3_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void EXTI15_10_IRQ(void) __attribute__((alias("IRQ_Dispatch")));
void RTC_Alarm_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void OTG_FS_WKUP_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM8_BRK_TIM12_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM8_UP_TIM13_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM8_TRG_COM_TIM14_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM8_CC_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA1_Stream7_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void FMC_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void SDIO_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM5_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void SPI3_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void UART4_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void UART5_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM6_DAC_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void TIM7_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA2_Stream0_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA2_Stream1_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA2_Stream2_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA2_Stream3_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA2_Stream4_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void CAN2_TX_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void CAN2_RX0_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void CAN2_RX1_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void CAN2_SCE_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void OTG_FS_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA2_Stream5_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA2_Stream6_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DMA2_Stream7_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void USART6_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void I2C3_EV_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void I2C3_ER_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void OTG_HS_EP1_OUT_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void OTG_HS_EP1_IN_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void OTG_HS_WKUP_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void OTG_HS_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void DCMI_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void FPU_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void SPI4_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void SAI1_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void SAI2_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void QUADSPI_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void CEC_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void SPDIF_RX_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void FMPI2C1_EV_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));
void FMPI2C1_ER_IRQHandler(void) __attribute__((alias("IRQ_Dispatch")));

void IRQ_Enable(IRQn_Type irq)
{
	/* Need implementation */
}

void IRQ_Disable(IRQn_Type irq)
{
	/* Need implementation */
}

void IRQ_SetData(IRQn_Type irq, void *data)
{
	/* Need implementation */
}

void *IRQ_GetData(IRQn_Type irq, void *data)
{
	/* Need implementation */
	return 0;
}

void IRQ_SetPriority(IRQn_Type irq, uint32_t priority)
{
	/* Need implementation */
}

uint32_t IRQ_GetPriority(IRQn_Type irq)
{
	/* Need implementation */
	return 0;
}

int IRQ_Register(IRQn_Type irq, IRQ_Handler handler, uint32_t priority, void *data)
{
	/* Need implementation */
	return -1;
}

int IRQ_Unregister(int irq)
{
	/* Need implementation */
	return -1;
}

int IRQ_Init(void)
{
	/* Need implementation */
	return -1;
}
