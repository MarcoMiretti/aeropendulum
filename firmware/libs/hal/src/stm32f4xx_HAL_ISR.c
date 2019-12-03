/**
 * \file	stm32f4xx_HAL_ISR.c	
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/15
 * 
 * \brief 	ISR_HAL source file
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup includes
 *  @{
 */
#include "stm32f4xx_HAL_ISR.h"
/** @}*/

/**
 * \addtogroup ISR Interrupt Service Routines
 * @{
 */

/**
 * \brief 	Handle External interrupt 0
 * \retval 	0 if success, -1 if error
 */ 
uint8_t EXTI0_IRQHandler(void)
{
	/* PA0 (User button) */
	if((EXTI->PR & 0x1) != 0)
	{
		TIM4->CCR3 = 0;
		while(1)
		{
		}
		/* Seems counter intuitive, but flag is cleared when it's programmed to 1 */
		EXTI->PR = 0x1;
	}
	return 0;
}
/** @}*/


