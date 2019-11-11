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
uint8_t TIM3_Init(void);
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
	RCC->AHB1ENR 	|= (uint32_t)(RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOCEN);
	
	/* PD12 (LED 4) */
	GPIOD->MODER 	&= (uint32_t)(~GPIO_MODER_MODE12_1);					
	GPIOD->MODER 	|= (uint32_t)GPIO_MODER_MODE12_0; 					/* Set LED4 (PD12) as GPIOD output */
	GPIOD->ODR	 	|= (uint32_t)GPIO_ODR_OD12;
	
	/* PD14 (PWM - LED 5) */
	GPIOD->MODER 	|= (uint32_t)GPIO_MODER_MODER14_1;
	GPIOD->MODER 	&= (uint32_t)(~GPIO_MODER_MODER14_0);					/* PD14 to alternate function MODER14 = 0b10 */
	
	GPIOD->OSPEEDR 	|= (uint32_t)(GPIO_OSPEEDER_OSPEEDR14_1 | GPIO_OSPEEDER_OSPEEDR14_0); 	/* PD14 to very high speed mode */
	
	GPIOD->AFR[1] 	&= (uint32_t)~(GPIO_AFRH_AFRH6_3 | GPIO_AFRH_AFRH6_2 | GPIO_AFRH_AFRH6_0);
	GPIOD->AFR[1] 	|= (uint32_t)(GPIO_AFRH_AFRH6_1);					/* PD14 mapped to AF2 (TIM4) */

	/* PC6 & PC7 (quadrature encoder) */
	GPIOC->MODER 	|= (uint32_t)GPIO_MODER_MODER6_1;
	GPIOC->MODER 	&= (uint32_t)~(GPIO_MODER_MODER6_0);					/* PC6 to alternate function MODER6 = 0b10 */
	GPIOC->MODER 	|= (uint32_t)GPIO_MODER_MODER7_1;
	GPIOC->MODER 	&= (uint32_t)~(GPIO_MODER_MODER7_0); 					/* PC7 to alternate function MODER7 = 0b10 */
	
	//GPIOC->OSPEEDR 	|= (uint32_t)(GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR6_0); 	/* PC6 to very high speed mode */
	//GPIOC->OSPEEDR 	|= (uint32_t)(GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR7_0); 	/* PC7 to very high speed mode */

	GPIOC->PUPDR 	|= (uint32_t)GPIO_PUPDR_PUPDR6_0; 					/* PC6 to pull-up */
	GPIOC->PUPDR 	|= (uint32_t)GPIO_PUPDR_PUPDR7_0; 					/* PC7 to pull-up */

	GPIOC->AFR[0] 	&= (uint32_t)~(GPIO_AFRL_AFRL6_3 | GPIO_AFRL_AFRL6_2 | GPIO_AFRL_AFRL6_0);
	GPIOC->AFR[0] 	|= (uint32_t)(GPIO_AFRL_AFRL6_1);					/* PC6 mapped to AF2 (TIM3 CH1) */
	GPIOC->AFR[0] 	&= (uint32_t)~(GPIO_AFRL_AFRL7_3 | GPIO_AFRL_AFRL7_2 | GPIO_AFRL_AFRL7_0);
	GPIOC->AFR[0] 	|= (uint32_t)(GPIO_AFRL_AFRL7_1);					/* PC7 mapped to AF2 (TIM3 CH2) */	
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
	/* Infinite Loop */
	while(1)
	{
		for (i = 0; i < 1000000; ++i) ; GPIOD -> ODR |= LD4_GPIO_PIN;
		PWM_SetDuty((TIM3->CNT*100)/0xFFFF);
		for (i = 0; i < 1000000; ++i) ; GPIOD -> ODR &= ~LD4_GPIO_PIN;
	}
	return 0;
}
