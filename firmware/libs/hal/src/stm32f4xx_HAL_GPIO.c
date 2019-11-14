/**
 * \file	stm32f4_HAL_GPIO.c	
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/13
 * 
 * \brief 	GPIO_HAL source file
 */
/*---------------------------------------------------------------------------*/

/** \addtogroup Included headers 
 *  @{
 */
#include "stm32f4xx_HAL.h"
/** @}*/

/**
 * \addtogroup GPIO Functions
 * @{
 */

/**
 * \brief Set GPIO Mode (with MODER reg.)
 * \param port The GPIO Port (A=0, I=8)
 * \param pin The GPIO pin number
 * \param mode The selected mode:
 * 		0b00 = Input
 * 		0b01 = Output
 * 		0b10 = Alternate function
 * 		0b11 = Analog mode
 * \return 0 if success
 */
uint8_t GPIO_ModeSet(uint8_t port, uint8_t pin, uint8_t mode)
{
	/* Assert limit values */
	assert((port > -1) & (port < 8));
	assert((pin > -1) & (pin < 16));
       	assert((mode > -1) & (mode < 4));	
	/* Calculate address */
	uint32_t *GPIOx;
	uint32_t ADDRESS = (GPIOA_BASE + port*0x0400);
	GPIOx = (uint32_t*) ADDRESS;
	/* Erase pin function */
	*GPIOx	 	&= ~(uint32_t)(11<<(2*pin));
	/* Write new pin function */
	*GPIOx	 	|=  (uint32_t)(mode<<(2*pin));
	return 0;
}
/** @}*/

/**
 * \brief Set GPIO Pull up/down config (with PUPDR reg.)
 * \param port The GPIO Port (A=0, I=8)
 * \param pin The GPIO pin number
 * \param pupd The Configuration:
 * 		0b00 = No pull
 * 		0b01 = Pull-Up
 * 		0b10 = Pull-Down
 * 		0b11 = Reserved
 * \retval 0 if success
 */
uint8_t GPIO_SetPullUpPullDown(uint8_t port, uint8_t pin, uint8_t pupd)
{
	/* Assert limit values */
	assert((port > -1) & (port < 8));
	assert((pin > -1) & (pin < 16));
       	assert((pupd > -1) & (pupd < 4));	
	/* Calculate address */
	uint32_t *GPIOx;
	uint32_t ADDRESS = GPIOA_BASE + port*0x0400 + 0x0C;
	GPIOx = (uint32_t*) ADDRESS;
	/* Erase pin function */
	*GPIOx	 	&= ~(uint32_t)(11<<(2*pin));
	/* Write new pin function */
	*GPIOx	 	|=  (uint32_t)(pupd<<(2*pin));
	return 0;
}
/** @}*/

/**
 * \brief Set GPIO Out Speed (with OSPEEDR reg.)
 * \param port The GPIO Port (A=0, I=8)
 * \param pin The GPIO pin number
 * \param speed The Speed:
 * 		0b00 = Low (limit 2MHz)
 * 		0b01 = Medium (limit 10MHz)
 * 		0b10 = High (limit 50MHz)
 * 		0b11 = Very High (limit 100MHz)
 * \return 0 if success
 */
uint8_t GPIO_OutSpeed(uint8_t port, uint8_t pin, uint8_t speed)
{
	/* Assert limit values */
	assert((port > -1) & (port < 8));
	assert((pin > -1) & (pin < 16));
       	assert((speed > -1) & (speed < 4));	
	/* Calculate address */
	uint32_t *GPIOx;
	uint32_t ADDRESS = GPIOA_BASE + port*0x0400 + 0x08;
	GPIOx = (uint32_t*) ADDRESS;
	/* Erase pin function */
	*GPIOx	 	&= ~(uint32_t)(11<<(2*pin));
	/* Write new pin function */
	*GPIOx	 	|=  (uint32_t)(speed<<(2*pin));
	return 0;
}
/** @}*/

/**
 * \brief Set GPIO Alternate Function (with AFRL and AFRH reg.)
 * \param port The GPIO Port (A=0, I=8)
 * \param pin The GPIO pin number
 * \param af The Alternate function number
 * \return 0 if success
 */
uint8_t GPIO_SetAlternateFunction(uint8_t port, uint8_t pin, uint8_t af)
{
	/* Assert limit values */
	assert((port > -1) & (port < 8));
	assert((pin > -1) & (pin < 16));
       	assert((af > -1) & (af < 16));	
	/* Calculate address */
	uint8_t HIGH_OFFSET = 0;
	if(pin > 7)
	{
		HIGH_OFFSET = 4;
		pin -= 8;
	}
	uint32_t *GPIOx;
	uint32_t ADDRESS = GPIOA_BASE + port*0x0400 + 0x20 + HIGH_OFFSET;
	GPIOx = (uint32_t*) ADDRESS;
	/* Erase pin function */
	*GPIOx	 	&= ~(uint32_t)(1111<<(4*pin));
	/* Write new pin function */
	*GPIOx	 	|=  (uint32_t)(af<<(4*pin));
	return 0;
}
/** @}*/


/**
 * \brief Set GPIO Out Data (with ODR reg.)
 * \param port The GPIO Port (A=0, I=8)
 * \param pin The GPIO pin number
 * \param dir Output data:
 * 		0b00 = In
 * 		0b01 = Out
 * \return 0 if success
 */
uint8_t GPIO_OutData(uint8_t port, uint8_t pin, uint8_t data)
{
	/* Assert limit values */
	assert((port > -1) & (port < 8));
	assert((pin > -1) & (pin < 16));
       	assert((data == 1) | (data == 0));	
	/* Calculate address */
	uint32_t *GPIOx;
	uint32_t ADDRESS = GPIOA_BASE + port*0x0400 + 0x14;
	GPIOx = (uint32_t*) ADDRESS;
	/* Erase pin function */
	*GPIOx	 	&= ~(uint16_t)(1<<pin);
	/* Write new pin function */
	*GPIOx	 	|=  (uint16_t)(data<<pin);
	return 0;
}
/** @}*/
