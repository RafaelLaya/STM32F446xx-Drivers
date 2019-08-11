/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Aug 4, 2019
 *      Author: rafae
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Aug 4, 2019
 *      Author: rafae
 */

#include "stm32f446xx_rcc_driver.h"



uint32_t Get_PLL_P_CLK(void);

uint32_t Get_PLL_R_CLK(void);

uint32_t RCC_Get_PCLK1Freq(void);

uint32_t RCC_Get_PCLK2Freq(void);



#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
