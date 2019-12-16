/**
 * \file	stm32f4xx_HAL.h	
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/13
 * 
 * \brief 	principal HAL header file
 */
/*---------------------------------------------------------------------------*/
/* Prevent recursive inclusion */
#ifndef __STM32F4_HAL_H
#define __STM32F4_HAL_H

/** \addtogroup deps
 *  @{
 */
#include <assert.h>
#include "stm32f4xx.h"
#include "stm32f4xx_HAL_RCC.h"
#include "stm32f4xx_HAL_GPIO.h"
/** @}*/

/** \addtogroup function_macros Function like macros
 *  @{
 */
#define msToTick(ms) 		(ms/portTICK_PERIOD_MS)	/**< Macro that converts Milliseconds to RTOS ticks */
/** @} */
#endif /* __STM32F4_HAL_H */
