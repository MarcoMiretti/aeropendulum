/**
 * \file	stm32f4xx_HAL_ISR.h	
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/15
 * 
 * \brief 	ISR_HAL header file
 */
/*---------------------------------------------------------------------------*/
/* Prevent recursive inclusion */
#ifndef __STM32F4_HAL_ISR_H
#define __STM32F4_HAL_ISR_H

/** \addtogroup deps
 *  @{
 */
#include "stm32f4xx_HAL.h"
/** @}*/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup primitives
 * @{
 */
uint8_t EXTI0_IRQHandler(void);
/** @}*/

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4_HAL_RCC_H */
