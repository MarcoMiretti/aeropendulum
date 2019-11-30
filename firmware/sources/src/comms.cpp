/**
 * \file	comms.cpp
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/29
 * 
 * \brief 	Handles communication related tasks, contains communication related functions.
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

/** @}*/

/**
 * \brief 	Aeropendulum communications task.
 * \note 	The goal of this task is to receive commands and send them to the corresponding areas. It also sends data upon request.
 */

void aero_comms(void *pvParameters)
{
	/* Enable usart */
	UART5->CR1 	|=	 (uint32_t)(1<<13);
	/* 8 data bits */
	UART5->CR1	&=	~(uint32_t)(1<<12);
	/* 1 stop bit */
	UART5->CR2	&=	~(uint32_t)(11<<12);
	/* RXE interrupt inhibited */
	UART5->CR1	&=	~(uint32_t)(1<<5);
	/* TXE interrupt inhibited */
	UART5->CR1	&=	~(uint32_t)(1<<7);
	/* Parity control disable */
	UART5->CR1	&=	~(uint32_t)(1<<10);
	/* Oversampling 16 bits */
	UART5->CR1	&=	~(uint32_t)(1<<15);
	/* Half duplex mode not selected  */
	UART5->CR3	&=	~(uint32_t)(1<<3);

	/*
	 * Baud Rate = fck/[8*(2-OVER8)*USARTDIV]
	 * 
	 * hence: USARTDIV = (fck/BR)/(8*(2-OVER8))
	 * 
	 * Since:
	 * 	* fck = 16 MHz (HSI without prescallers)
	 * 	* OVER8 = 0 (16 bits oversampling)
	 * 	* BR = 9600 (depends on bluetooth dongle)
	 *
	 * Then:
	 * 	* USARTDIV = 104.166666
	 * 	Fraction: 0.1666*16=2.66==>3==>0x3
	 * 	Mantissa: 104 ==> 0x68
	 * */
	UART5->BRR	=	(uint32_t)((0x68<<4)|(0x3));

	while(1)
	{
		UART5->CR1	|=  (uint32_t)(1<<3);
		while(!(UART5->SR & USART_SR_TXE));
		UART5->DR	 =  (uint8_t)('a');
		vTaskDelay(msToTick(100));
	}
}
