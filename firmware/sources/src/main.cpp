/**---------------------------------------------------------------------------/
 * \file	main.cpp
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 * 
 * \brief 	main file, entry point and pin definitions.
 *---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/** \addtogroup Included dependencies
 *  @{
 */
#include <cstdio>
#include "main.h"
#include "stm32f4xx_HAL.h"
/** @}*/

/** \addtogroup function definitions 
 *  @{
 */
uint8_t GPIO_Init(void);
uint8_t TIM3_Init(void);
uint8_t TIM4_Init(void);
uint8_t PWM_Init(void);
uint8_t PWM_SetDuty(uint8_t duty);
uint8_t blinkLed4(void);
/** @} */

/**
  * @brief  Application entry point.
  * @retval int
  */
int main(void)
{
	GPIO_Init();
	TIM3_Init();
	TIM4_Init();
	PWM_Init();
	blinkLed4();
	return 0;
}

/**
  * @brief  Enables and sets peripherals.
  * @retval int
  */
uint8_t GPIO_Init(void)
{
	/* Clock for GPIOD and GPIOC */
	RCC_GPIOPortSetClock(2,1);
	RCC_GPIOPortSetClock(3,1);
	RCC_GPIOPortSetClock(0,1);

	/* Set LED4 (PD12) as GPIO output */
	GPIO_ModeSet(3,12,1);
	GPIO_OutData(3,12,1);

	/* Set User button (PA0) as GPIO input */
	GPIO_ModeSet(0,0,0);
	/* Set PA0 as pull-down */
	GPIO_SetPullUpPullDown(2,6,2);

	/* Configure interrupt for EXTI0 */
	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_IRQn);

	/* Unmask line 0 */
	EXTI->IMR |= 1;
	/* Set line 0 as rising edge */
	EXTI->RTSR |= 1;

	/* Set LED5 (PD14) as alternate function */
	GPIO_ModeSet(3,14,2);
	/* Set LED5 (PD14) to very high speed mode */
	GPIO_OutSpeed(3,14,3);
	
	/* PD14 mapped to AF2 (TIM4) */
	GPIO_SetAlternateFunction(3,14,2);
	
	/* PC6 & PC7 to alternate mode (for quadrature encoder) */
	GPIO_ModeSet(2,6,2);
	GPIO_ModeSet(2,7,2);
	
	/* Set PC6 to Pull-Up */
	GPIO_SetPullUpPullDown(2,6,1);
	/* Set PC7 to Pull-Up */
	GPIO_SetPullUpPullDown(2,7,1);

	/* PC6 mapped to AF2 (TIM3 CH1) */
	GPIO_SetAlternateFunction(2,6,2);
	/* PC7 mapped to AF2 (TIM3 CH2) */
	GPIO_SetAlternateFunction(2,7,2);
	
	return 0;
}



/**
  * @brief  Init sequence for TIM4.
  * @retval int
  */
uint8_t TIM4_Init(void)
{
	/* Clock for TIM4*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	/* Set TIM4 */
	uint16_t tmpcr1 = 0;
	tmpcr1 = TIM4->CR1;
	
	tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));				/* Set the Counter Mode (edge up) */
	
	TIM4->CR1 = tmpcr1;

	TIM4->ARR  = TIM4_ARR; 								/* Sets timer period */
	TIM4->PSC = 0;
	TIM4->EGR = TIM_PSCReloadMode_Immediate;  					/* Reload PSC */
	
	TIM4->CR1 |= TIM_CR1_CEN;							/* Enable TIM4 */
	return 0;
}

/**
  * @brief  Init sequence for TIM3.
  * @retval int
  */
uint8_t TIM3_Init(void)
{
	/* Clock for TIM3*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	/* Set TIM3 */
	uint16_t tmpcr1 = 0, tmpccmr1 = 0, tmpccer = 0, tmpsmcr = 0;
	
	tmpcr1 = TIM3->CR1;
    	tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CEN));		/* Set the Counter Mode (edge up) */
	TIM3->CR1 = tmpcr1;

	tmpccmr1 = TIM3->CCMR1;
	tmpccmr1 &= (uint16_t)~(TIM_CCMR1_CC1S_1 | TIM_CCMR1_CC2S_1);
	tmpccmr1 |= (uint16_t)(TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);			/* TI1FP1 mapped on TI1 & TI2FP2 mapped on TI2 */
	tmpccmr1 |= (uint16_t)(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F); 			/* Input filter for CH1 and CH2 */
	TIM3->CCMR1 = tmpccmr1;
	
	tmpccer = TIM3->CCER;
	tmpccer &= (uint16_t)(~(TIM_CCER_CC1P | TIM_CCER_CC1NP));			/* TI1FP1 noninverted */
	tmpccer &= (uint16_t)(~(TIM_CCER_CC2P | TIM_CCER_CC2NP));			/* TI2FP2 noninverted */
	TIM3->CCER = tmpccer;

	tmpsmcr = TIM3->SMCR;
	tmpsmcr |= (uint16_t)(TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
	tmpsmcr &= (uint16_t)(~(TIM_SMCR_SMS_2));					/* both inputs are active on both rising and falling edges */
	TIM3->SMCR = tmpsmcr;

	TIM3->ARR  = 0xFFFF; 								/* Sets timer period */
	TIM3->CNT  = 0;
	TIM3->EGR  |= 1; 								/* Reload PSC */
	
	TIM3->CR1 |= TIM_CR1_CEN;							/* Enable TIM3 */
	return 0;
}

/**
  * @brief  Init sequence for PWM.
  * @retval int
  */
uint8_t PWM_Init(void)
{
	/* Set PWM */
	uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
	
	TIM4->CCER &= (uint16_t)~TIM_CCER_CC3E;						/* Disable the Channel 1: Reset the CC1E Bit */
	
	tmpccer = TIM4->CCER;								/* Get the TIM4 CCER register value */
	tmpcr2 =  TIM4->CR2;								/* Get the TIM4 CR2 register value */
	tmpccmrx = TIM4->CCMR1;								/* Get the TIM4 CCMR1 register value */
	
	tmpccmrx &= (uint16_t)~TIM_CCMR2_OC3M;						/* Reset the Output Compare Mode Bits */
	tmpccmrx &= (uint16_t)~TIM_CCMR2_CC3S; 
	tmpccer &= (uint16_t)~TIM_CCER_CC3P;						/* Reset the Output Polarity level */
	
	tmpccmrx |= TIM4_CCMR2_OC3M;							/* Select the Output Compare Mode */
  
	tmpccer |= TIM4_CCER_CC3P;							/* Set the Output Compare Polarity */
	tmpccer |= TIM4_CCER_CC3E;							/* Set the Output State */
		  
	TIM4->CR2 = tmpcr2;								/* Write to TIMx CR2 */
	TIM4->CCMR2 = tmpccmrx;								/* Write to TIMx CCMR1 */
	TIM4->CCR3	= TIM4_CCR3;							/* Set duty */
	TIM4->CCER	= tmpccer;							/* Enable PWM2 output */
	TIM4->CCMR2 |= TIM4_OCPreload_Enable;
	return 0;
}

/**
  * @brief  Change pwm duty value.
  * @param	duty: integer between 0 and 100 corresponding to the duty 
  * 		percentage.
  * @retval int
  */
uint8_t PWM_SetDuty(uint8_t duty)
{
	uint16_t tempccr3 = 0;
	tempccr3 = (TIM4_ARR*duty)/100;
	TIM4->CCR3 = tempccr3;
	return 0;
}

/**
  * @brief  Creates an infinite loop where the LD4 blinks.
  * @retval int
  */
uint8_t blinkLed4(void)
{
	volatile unsigned int i =0;
	volatile unsigned int j =0;
	/* Infinite Loop */
	while(1)
	{
		for (i = 0; i < 100000; ++i) ; GPIO_OutData(3,12,1);
		//PWM_SetDuty((TIM3->CNT*100)/0xFFFF);
		PWM_SetDuty(j);
		for (i = 0; i < 100000; ++i) ; GPIO_OutData(3,12,0);
		j = j+10;
		if(j>100) j=0;
		printf("Printing position: ");
	}

	return 0;
}
