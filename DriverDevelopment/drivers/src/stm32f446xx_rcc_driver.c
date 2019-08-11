/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Aug 4, 2019
 *      Author: rafae
 */

#include "stm32f446xx_rcc_driver.h"

/*
 * PLL clock not supported yet
 */

uint32_t Get_PLL_P_CLK()  {
	return 16000000U;
}

uint32_t Get_PLL_R_CLK() {
	return 16000000U;
}

uint32_t RCC_Get_PCLK1Freq() {
	uint32_t SYSCLK, AHB1Pre, APB1Pre, srcClock;

	srcClock = ((RCC->CFGR) >> RCC_CFGR_SW) && (0x3);

	if(srcClock == 0) {
		// HSI is fixed in this MCU
		SYSCLK = 16000000U;
	} else if(srcClock == 1) {
		// HSE is not present. User can install one
		// in that case, I would put this as configurable
		// in the I2C_Config_t structure. Let's put a dummy
		// fixed value just to follow along since I don't even have an HSE to test
		SYSCLK = 8000000U;
	} else if (srcClock == 2) {
		SYSCLK = Get_PLL_P_CLK();
	} else if (srcClock == 3) {
		SYSCLK = Get_PLL_R_CLK();
	}

	uint8_t temp = (RCC->CFGR >> RCC_CFGR_HPRE) & (0xf);
	uint32_t AHB1Prescaler[] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
	if(temp < 8) {
		AHB1Pre = 1;
	} else {
		AHB1Pre = AHB1Prescaler[temp - 8];
	}

	temp = (RCC->CFGR >> RCC_CFGR_PPRE1) & (0x7);
	uint32_t APB1Prescaler[] = {2, 4, 8, 16};
	if(temp < 4) {
		APB1Pre = 1;
	} else {
		APB1Pre = APB1Prescaler[temp - 4];
	}

	return SYSCLK / AHB1Pre / APB1Pre;
}

uint32_t RCC_Get_PCLK2Freq() {
	uint32_t SYSCLK, AHBPre, APB2Pre, srcClock;

	srcClock = ((RCC->CFGR) >> RCC_CFGR_SW) && (0x3);

	if(srcClock == 0) {
		// HSI is fixed in this MCU
		SYSCLK = 16000000U;
	} else if(srcClock == 1) {
		// HSE is not present. User can install one
		// in that case, I would put this as configurable
		// in the I2C_Config_t structure. Let's put a dummy
		// fixed value just to follow along since I don't even have an HSE to test
		SYSCLK = 8000000U;
	} else if (srcClock == 2) {
		SYSCLK = Get_PLL_P_CLK();
	} else if (srcClock == 3) {
		SYSCLK = Get_PLL_R_CLK();
	}

	uint8_t temp = (RCC->CFGR >> RCC_CFGR_HPRE) & (0xf);
	uint32_t AHBPrescaler[] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
	if(temp < 8) {
		AHBPre = 1;
	} else {
		AHBPre = AHBPrescaler[temp - 8];
	}

	temp = (RCC->CFGR >> RCC_CFGR_PPRE2) & (0x7);
	uint32_t APB2Prescaler[] = {2, 4, 8, 16};
	if(temp < 4) {
		APB2Pre = 1;
	} else {
		APB2Pre = APB2Prescaler[temp - 4];
	}

	return SYSCLK / AHBPre / APB2Pre;
}
