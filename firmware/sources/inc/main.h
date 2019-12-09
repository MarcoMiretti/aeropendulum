/**
 * \file	main.h
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 * 
 * \brief 	main header
 */
/*---------------------------------------------------------------------------*/

/* Prevent recursive inclusion */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
#include <cstdio>
#endif

#include "stm32f4xx_HAL.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stream_buffer.h"

/** @addtogroup GPIO_Constants GPIO Constants
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
  

/** @addtogroup TIM_Constants TIM Constants
  * @{
  */
#define TIM4_PSC			((uint16_t)0x0008)
#define TIM4_ARR 			((uint16_t)(8000000/(TIM4_PSC*PWM_FREQ)-1))		/* Auto reload register = 8000000 / 46*350 - 1 = 0x1F0 set for 10kHz PWM*/
#define TIM_PSCReloadMode_Immediate     ((uint16_t)0x0001)
/** @addtogroup PWM_Constants PWM Constants
  * @{
  */
#define PWM_FREQ			((uint16_t)100)
#define TIM4_CCMR2_OC3M 		((uint16_t)0x0070)		/* PWM mode 2 */
#define TIM4_CCER_CC3E			((uint16_t)0x0001 << 8)
#define TIM4_CCER_CC3P			((uint16_t)0x0002 << 8) 	/* Polarity Low */
#define TIM4_CCR3			((uint16_t)0x0000)  		/* 25% duty TODO: implement a macro */
#define TIM4_OCPreload_Enable           ((uint16_t)0x0008)
/**
  * @}
  */
/**
  * @}
  */
enum variables
{
	onOff,
	mode,
	set_point,
	angle,
	motorPower,
	u0,
	mK,
	Kp,
	Ki,
	Kd,
	Ts,
};

enum instructions
{
	print,
	set,
};

struct command
{
	uint8_t instruction;
	uint8_t variable;
	float value;
};

/**
 * \brief Converts float to ascii string and writes it \TODO: separate in two functions
 * \param value = float to convert
 **/
void floatWrite(float value);

#endif /* __MAIN_H */
