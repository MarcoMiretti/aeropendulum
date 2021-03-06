/**
 * \file	stm32f4xx_HAL_RCC.h	
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/13
 * 
 * \brief 	RCC_HAL header file
 */
/*---------------------------------------------------------------------------*/
/* Prevent recursive inclusion */
#ifndef __STM32F4_HAL_RCC_H
#define __STM32F4_HAL_RCC_H

/** \addtogroup deps
 *  @{
 */
#include "stm32f4xx_HAL.h"
/** @}*/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup primitives Function primitives
 * @{
 */
uint8_t RCC_GPIOPortSetClock(uint8_t port, uint8_t state);
/** @}*/

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4_HAL_RCC_H */
