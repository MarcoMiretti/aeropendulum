/**
 * \file	stm32f4xx_HAL_GPIO.h	
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/13
 * 
 * \brief 	GPIO_HAL header file
 */
/*---------------------------------------------------------------------------*/
/* Prevent recursive inclusion */
#ifndef __STM32F4_HAL_GPIO_H
#define __STM32F4_HAL_GPIO_H

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
uint8_t GPIO_ModeSet(uint8_t port, uint8_t pin, uint8_t mode);
uint8_t GPIO_SetPullUpPullDown(uint8_t port, uint8_t pin, uint8_t pupd);
uint8_t GPIO_OutSpeed(uint8_t port, uint8_t pin, uint8_t speed);
uint8_t GPIO_OutData(uint8_t port, uint8_t pin, uint8_t data);
uint8_t GPIO_SetAlternateFunction(uint8_t port, uint8_t pin, uint8_t af);
/** @}*/

#ifdef __cplusplus
}
#endif
#endif /* __STM32F4_HAL_GPIO_H */
