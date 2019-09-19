/**
  ******************************************************************************
  * @file    main.c
  * @author  Marco Miretti
  * @brief   TODO: write brief
  *
  *	TODO: write long project description
  *
  ******************************************************************************
  * @section LICENSE TODO: choose license
  ******************************************************************************
  */

#include "main.h"

/**
  * @brief  Application entry point.
  * @retval int
  */
int main(void)
{
	#define LD4_GPIO_PIN 	(1 << 12)  // PD12
	#define LD4_MODE_OUT	(1 << 24)  // General Purpose Input for PD12
	#define GPIOD_EN  		(1 << 3)   // RCC GPIOD Enable Bit
	RCC->AHB1ENR |= GPIOD_EN;
	volatile unsigned int i =0;
	
	GPIOD->MODER |= LD4_MODE_OUT;
	GPIOD->ODR	 |= LD4_GPIO_PIN;
	
	/* Infinite Loop */
	while(1)
	{
		for (i = 0; i < 1000000; ++i) ; GPIOD -> ODR |= LD4_GPIO_PIN;
		for (i = 0; i < 1000000; ++i) ; GPIOD -> ODR &= ~LD4_GPIO_PIN;
	}
	return 0;
}

