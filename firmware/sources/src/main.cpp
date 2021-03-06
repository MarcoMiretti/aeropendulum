/**
 * \file	main.cpp
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 * 
 * \brief 	main file, entry point and pin definitions.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup deps Included dependencies
 *  @{
 */
#include "main.h"
#include "driving.h"
#include "comms.h"
/** @}*/

/** \addtogroup defs Function definitions 
 *  @{
 */
/**
 * \brief Init Reset and clock control
 * */
uint8_t RCC_Init(void);
uint8_t GPIO_Init(void);
uint8_t TIM3_Init(void);
uint8_t TIM4_Init(void);
uint8_t PWM_Init(void);
/** @} */

/** @addtogroup queues
  * @{
  */
QueueHandle_t xDrivingQ; /**< Queue that communicates comms->driving */
/**
  * @}
  */

/**
  * @brief  Application entry point.
  * @retval 0 if success.
  */
int main(void)
{
	RCC_Init();
	GPIO_Init();
	TIM3_Init();
	TIM4_Init();
	PWM_Init();

	xDrivingQ = xQueueCreate( 1 , ( UBaseType_t ) sizeof( struct command ) );

	xTaskCreate( aero_driving, "Aero Driving", 512, ( void * ) &xDrivingQ, 1, NULL);
	xTaskCreate( aero_comms, "Aero Comms", 512, &xDrivingQ, 1, NULL);
	
	vTaskStartScheduler();
	return 0;
}

uint8_t RCC_Init(void)
{
	/* Reset CR to default state */
	RCC->CR 	&= ~(uint32_t)(0xFFFF00FF);
	RCC->CR		|=  (uint32_t)(0x83);

	/* Reset pll config to default state */
	RCC->PLLCFGR	&= ~(uint32_t)(0x0FC3FFFF);
	RCC->PLLCFGR	|=  (uint32_t)(0x04003010);

	RCC->CFGR	= 0;

	/* Enable UART5 clock */
	RCC->APB1ENR 	 = (1<<20);
	/* Enable DMA1 clock */
	RCC->AHB1ENR 		|=  (uint32_t)(1<<21);
	return 0;
}

/**
  * @brief  Enables and sets peripherals.
  * @retval 0 if success.
  */
uint8_t GPIO_Init(void)
{
	/* Clock for GPIOA,B,C,D */
	RCC_GPIOPortSetClock(0,1);
	RCC_GPIOPortSetClock(1,1);
	RCC_GPIOPortSetClock(2,1);
	RCC_GPIOPortSetClock(3,1);

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
	/* Set LED5 (PD14) to low speed mode */
	GPIO_OutSpeed(3,14,0);
	/* PD14 mapped to AF2 (TIM4) */
	GPIO_SetAlternateFunction(3,14,2);
	
	/* Set PB8 as alternate function */
	GPIO_ModeSet(1,8,2);
	/* Set PB8 to low speed mode */
	GPIO_OutSpeed(1,8,0);
	/* PD14 mapped to AF2 (TIM4) */
	GPIO_SetAlternateFunction(1,8,2);

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

	/* Set PC12 as alternate function */
	GPIO_ModeSet(2,12,2);
	/* Set PD2 as alternate function */
	GPIO_ModeSet(3,2,2);

	/* PC12 mapped to AF8 (UART5 TX) */
	GPIO_SetAlternateFunction(2,12,8);
	/* Set PC12 to Pull-Up */
	GPIO_SetPullUpPullDown(2,12,1);
	/* PD2 mapped to AF8 (UART5 RX) */
	GPIO_SetAlternateFunction(3,2,8);

	return 0;
}



/**
  * @brief  Init sequence for TIM4.
  * @retval 0 if success.
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
	TIM4->PSC = 8;
	TIM4->EGR = TIM_PSCReloadMode_Immediate;  					/* Reload PSC */
	
	TIM4->CR1 |= TIM_CR1_CEN;							/* Enable TIM4 */
	return 0;
}

/**
  * @brief  Init sequence for TIM3.
  * @retval 0 if success.
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
  * @retval 0 if success.
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
