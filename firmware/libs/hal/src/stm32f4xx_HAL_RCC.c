/**
 * \file	stm32f4_HAL_RCC.c	
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/13
 * 
 * \brief 	RCC_HAL source file
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup Included headers 
 *  @{
 */
#include "stm32f4xx_HAL_RCC.h"
/** @}*/

/**
 * \addtogroup RCC functions
 * @{
 */

/**
 * \brief 	Set GPIOx port clock
 * \param port 	represents the x port of GPIO
 * 		A=0 ,I=8
 * \param state	1=enable clock, 0=disable clock
 * \return 	0 if success, 1 if error
 */ 
uint8_t RCC_GPIOPortSetClock(uint8_t port, uint8_t state)
{
	/* Assert between limit values */
	assert((port > -1) & (port < 9));
	assert((state == 1) | (state == 0));
	/* Clean possible previous value */
	RCC->AHB1ENR	&= ~(uint32_t)(1 << port);
	/* Set new value */
	RCC->AHB1ENR 	|=  (uint32_t)(state << port);
	return 0;
}
/** @}*/
