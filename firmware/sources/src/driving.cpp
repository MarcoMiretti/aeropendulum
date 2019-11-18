/**
 * \file	driving.cpp
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 * 
 * \brief 	Hardware functions and tasks source.
 * \note 	This file handles everything related to the hardware driving, that is propulsion and measurement.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup deps
 *  @{
 */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "driving.h"
/** @}*/

/** \addtogroup defs 
 *  @{
 */
uint8_t PWM_SetDuty(float duty);
uint8_t	PWM_SetDuty_Milli(float duty);
uint8_t blinkLed4(void);
uint8_t aero_motorTest(void); 
/** @} */

/**
 * \brief 	Aeropendulum driving task.
 * \note 	The goal of this task is to update the position, calculate the propeller input voltage and drive the motor.
 */
void aero_driving(void *pvParameters)
{
	aero_motorTest();
}

/**
  * @brief  Change pwm duty value.
  * @param  duty: integer between 0 and 100 corresponding to the duty 
  * 		percentage.
  * @retval 0 if success
  */
uint8_t PWM_SetDuty(float duty)
{
	uint16_t tempccr3 = 0;
	tempccr3 = (TIM4_ARR*duty)/100;
	TIM4->CCR3 = tempccr3;
	return 0;
}

/**
  * @brief  Set PWM duty value in miliseconds.
  * @param  duty: float corresponding to the duty value in miliseconds.
  * @retval 0 if success
  */
uint8_t PWM_SetDuty_Milli(float duty)
{
	PWM_SetDuty(duty*PWM_FREQ/10);
	return 0;
}

/**
  * @brief  Creates an infinite loop where the LD4 blinks.
  * @retval 0 if success
  */
uint8_t blinkLed4(void)
{
	volatile unsigned int i =0;
	/* Infinite Loop */
	while(1)
	{
		for (i = 0; i < 1000000; ++i) ; GPIO_OutData(3,12,1);
		for (i = 0; i < 1000000; ++i) ; GPIO_OutData(3,12,0);
	}
	return 0;
}

/**
  * @brief  Test the aeropendulum motor with pwm.
  * @retval 0 if success
  */
uint8_t aero_motorTest(void)
{
	volatile unsigned int i=0;
	/* Infinite Loop */
	PWM_SetDuty_Milli(1.1);
	for (i = 0; i < 1000000; ++i) ; GPIO_OutData(3,12,1);
	while(1)
	{
		for (i = 0; i < 100000; ++i) ; GPIO_OutData(3,12,0);
		PWM_SetDuty_Milli(2.24);
		for (i = 0; i < 100000; ++i) ; GPIO_OutData(3,12,1);
		PWM_SetDuty_Milli(2.24);
		for (i = 0; i < 100000; ++i) ; GPIO_OutData(3,12,0);
		PWM_SetDuty_Milli(2.26);
		for (i = 0; i < 100000; ++i) ; GPIO_OutData(3,12,1);
		PWM_SetDuty_Milli(2.26);
	}

	return 0;
}
