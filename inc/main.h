/**
  ******************************************************************************
  * @file    main.h
  * @author  Marco Miretti
  * @brief   TODO: write brief
  *
  *	TODO: write long project description
  *
  ******************************************************************************
  * @section LICENSE TODO: choose license
  ******************************************************************************
  */

/* Prevent recursive inclusion */
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"


/** @defgroup GPIO_Constants
  * @{
  */
#define LD4_GPIO_PIN 		(1 << 12)  				/* PD12 */
#define LD4_MODE_OUT		(1 << 24)  				/* General Purpose Input for PD12 */
#define LD5_GPIO_PIN 		(1 << 14)  				/* PD14 */
#define PD14_AFRH_AF2		((uint32_t)0x02000000)			/* PD14 to TIM4 */
#define PD14_OSPEEDR_VHS	((uint32_t)0x30000000)			/* PD14 High Speed (100Mhz and beyond) */
#define LD5_MODE_ALT		((uint32_t)0x20000000)  		/* General Purpose Input for PD14 */
/**
  * @}
  */
  

/** @defgroup TIM_Constants
  * @{
  */
#define TIM4_ARR 			((uint16_t)0x20CF)		/* Auto reload register = 84000000 / 10000 - 1 = 8399 set for 10kHz PWM*/
#define TIM_PSCReloadMode_Immediate     ((uint16_t)0x0001)
/** @defgroup PWM_Constants
  * @{
  */
#define TIM4_CCMR2_OC3M 		((uint16_t)0x0070)		/* PWM mode 2 */
#define TIM4_CCER_CC3E			((uint16_t)0x0001 << 8)
#define TIM4_CCER_CC3P			((uint16_t)0x0002 << 8) 	/* Polarity Low */
#define TIM4_CCR3			((uint16_t)0x00FF)  		/* 25% duty TODO: implement a macro */
#define TIM4_OCPreload_Enable           ((uint16_t)0x0008)
/**
  * @}
  */
/**
  * @}
  */
#endif /* __MAIN_H */
