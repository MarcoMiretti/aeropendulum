/**
  **********************************************************************
  * @file    main.c
  * @author  Marco Miretti
  * @brief   TODO: write brief
  *
  *	TODO: write long project description
  *
  **********************************************************************
  * @section LICENSE TODO: choose license
  **********************************************************************
  */

#include "main.h"

uint8_t GPIO_Init(void);
uint8_t TIM4_Init(void);
uint8_t PWM_Init(void);
uint8_t PWM_SetDuty(uint8_t duty);
uint8_t blinkLed4(void);


/**
  * @brief  Application entry point.
  * @retval int
  */
int main(void)
{
	GPIO_Init();
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
	/* Clock for GPIOD */
	RCC->AHB1ENR |= GPIOD_EN;
	
	/* Set LED4 (PD12) as GPIOD output */
	GPIOD->MODER |= LD4_MODE_OUT;
	GPIOD->ODR	 |= LD4_GPIO_PIN;
	
	GPIOD->MODER |= LD5_MODE_ALT;
	GPIOD->OSPEEDR |= PD14_OSPEEDR_VHS;
	GPIOD->AFR[1]	 |= PD14_AFRH_AF2;
	return 0;
}

/**
  * @brief  Init sequence for TIM4.
  * @retval int
  */
uint8_t TIM4_Init(void)
{
	/* Clock for TIM4*/
	RCC->APB1ENR |= TIM4_EN;
	
	/* Set TIM4 */
	uint16_t tmpcr1 = 0;
	tmpcr1 = TIM4->CR1;
	
    tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));					/* Set the Counter Mode (edge up) */
	
	TIM4->CR1 = tmpcr1;

	TIM4->ARR  = TIM4_ARR; 												/* Sets timer period */
	TIM4->PSC = 0;
	TIM4->EGR = TIM_PSCReloadMode_Immediate;  							/* Reload PSC */
	
	TIM4->CR1 |= TIM_CR1_CEN;											/* Enable TIM4 */
	return 0;
}

/**
  * @brief  Init sequence for TIM3.
  * @retval int
  */
uint8_t TIM3_Init(void)
{
	#define TIM3_EN  			(1 << 1) 
	//#define TIM3_ARR 								((uint16_t)0x20CF)		/* Auto reload register = 84000000 / 10000 - 1 = 8399 set for 10kHz PWM*/
	//#define TIM_PSCReloadMode_Immediate        		((uint16_t)0x0001)
	
	
	/* Clock for TIM3*/
	RCC->APB1ENR |= TIM3_EN;
	
	/* Set TIM3 */
	uint16_t tmpcr1 = 0, tmpccmr1 = 0, tmpccmr2 = 0, tmpccer = 0, tmpsmcr = 0;
	tmpcr1 = TIM3->CR1;
	
    tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));					/* Set the Counter Mode (edge up) */
	
	TIM3->CR1 = tmpcr1;


	tmpccmr1 = TIM3->CCMR1;
	tmpccmr1 |= TIM_CCMR1_CC1S_0;											/* TI1FP1 mapped on TI1 */
	//TIM3->CCMR1 = tmpccmr1;
	
	//tmpccmr1 = TIM3->CCMR2;
	tmpccmr1 |= TIM_CCMR1_CC2S_0;											/* TI2FP2 mapped on TI2 */
	TIM3->CCMR1 = tmpccmr1;
	
	tmpccer = TIM3->CCER;
	tmpccer &= (uint16_t)(~(TIM_CCER_CC1P | TIM_CCER_CC1NP));			/* TI1FP1 noninverted, TI1FP1=TI1 */
	tmpccer &= (uint16_t)(~(TIM_CCER_CC2P | TIM_CCER_CC2NP));			/* TI2FP2 noninverted, TI2FP2=TI2 */

	tmpsmcr = TIM3->SMCR;
	tmpsmcr |= (uint16_t)(~(TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1));
	tmpsmcr &= (uint16_t)(~(TIM_SMCR_SMS_2));							/* both inputs are active on both rising and falling edges */

	//TIM4->ARR  = TIM3_ARR; 											/* Sets timer period */
	//TIM4->PSC = 0;
	//TIM4->EGR = TIM_PSCReloadMode_Immediate;  						/* Reload PSC */
	
	TIM4->CR1 |= TIM_CR1_CEN;											/* Enable TIM4 */
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
	
	TIM4->CCER &= (uint16_t)~TIM_CCER_CC3E;								/* Disable the Channel 1: Reset the CC1E Bit */
	
	tmpccer = TIM4->CCER;												/* Get the TIM4 CCER register value */
	tmpcr2 =  TIM4->CR2;												/* Get the TIM4 CR2 register value */
	tmpccmrx = TIM4->CCMR1;												/* Get the TIM4 CCMR1 register value */
	
	tmpccmrx &= (uint16_t)~TIM_CCMR2_OC3M;								/* Reset the Output Compare Mode Bits */
	tmpccmrx &= (uint16_t)~TIM_CCMR2_CC3S; 
	tmpccer &= (uint16_t)~TIM_CCER_CC3P;								/* Reset the Output Polarity level */
	
	tmpccmrx |= TIM4_CCMR2_OC3M;										/* Select the Output Compare Mode */
  
	tmpccer |= TIM4_CCER_CC3P;											/* Set the Output Compare Polarity */
	tmpccer |= TIM4_CCER_CC3E;											/* Set the Output State */
		  
	TIM4->CR2 = tmpcr2;													/* Write to TIMx CR2 */
	TIM4->CCMR2 = tmpccmrx;												/* Write to TIMx CCMR1 */
	TIM4->CCR3	= TIM4_CCR3;											/* Set duty */
	TIM4->CCER	= tmpccer;												/* Enable PWM2 output */
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
	/* Infinite Loop */
	while(1)
	{
		for (i = 0; i < 1000000; ++i) ; GPIOD -> ODR |= LD4_GPIO_PIN;
		for (i = 0; i < 1000000; ++i) ; GPIOD -> ODR &= ~LD4_GPIO_PIN;
	}
	return 0;
}
