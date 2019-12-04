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
#include "comms.h"
#include <math.h>

enum variables
{
	onOff,
	mode,
	angle,
	motorPower,
	linearization,
	Kp,
	Ki,
	Kd,
	Ts,
};

/** @}*/
/** \addtogroup defs 
 *  @{
 */
/**
 * \brief	Writes to dma_tx_buffer and enables transaction
 * \param 	str = string to send
 * \param	len = lenght of string
 * \retval	status = 0 if success, 1 if busy
 */
uint8_t bt_write(uint8_t* str, uint32_t len);
/**
 * \brief	Configure UART to match HB01 bluetooth converter specs. The transfers are done via DMA.
 */
void bt_UART5_conf(void);
/**
 * 	\brief DMA configuration for UART5
 * */
void DMA_config(void);
/**
 * \brief	Decodes and executes the instruction in rx_buffer
 * \note	All the sub funtions modify rx_buffer, they cut the readed part, fox example if rx_buffer = s motorPower 2.23, the function that recognizes the command s returns rx_buffer = motorPower 2.23.
 * Sintax:
 * 	command variableName (=value)
 *
 * Commands:
 * (0)	p -> prints variableName value
 * (1)	s -> sets variableName value
 **/
void decodeInstruction(uint8_t *rx_buffer, uint16_t len);
/**
 * \brief detects which command was called
 * */
uint8_t getCommand(uint8_t *rx_buffer, uint16_t len);
/**
 * \brief deletes the first x characters of a string
 * \param sting = string pointer
 * \param len	= string lenght
 * \param x	= number of characters to delete
 * \return	new len
 * */
uint8_t eraseFirstXChars(uint8_t* string, uint16_t len, uint16_t x);
/**
 * \brief get the variable inside the function 
 * \param rx_buffer = the reception buffer (without command)
 * \param len	= string lenght
 * \param x	= number of characters to delete
 * \return 	new len
 * */
uint16_t getVariable(uint8_t* rx_buffer, uint16_t len, uint8_t* variable);
/**
 * \brief	Gives variable numeric equivalent
 * \param	variable = string that contains the variable name
 * \retval	Variable numeric equivalent
 */
uint8_t getVariableInt(uint8_t* variable);
/**
 * \brief get the float value of buffer
 * \note	this is a magic trick
 *  		
 *  	address	      |	uint8_t | uint8_t | uint8_t | uint8_t |
 *  	address	      | 	      float_32		      |
 *
 *  i position a float 32 where the four uint8_t values were, hence converting four ascii
 *  chars to a 32 bit float.
 * \param rx_buffer = rx_buffer pointer (only contains value in ascii)
 * \param len	= string lenght
 * \return 	float value
 * */
float getFloatValueFromChar(uint8_t* rx_buffer, uint16_t len);
/**
 * \brief get the float value of buffer
 * \note	this function converts a string to a float
 * \param rx_buffer = rx_buffer pointer (only contains value in ascii)
 * \param len	= string lenght
 * \return 	float value
 * */
float getFloatValue(uint8_t* rx_buffer, uint16_t len);
uint8_t bt_pVariable(uint8_t variable);
uint8_t bt_sVariable(uint8_t variable, float value);
/** @}*/

uint8_t dma_uart_rx_buffer[32];
uint8_t dma_uart_tx_buffer[8];

/** @addtogroup extern_vars 
  * @{
  */
extern StreamBufferHandle_t bt_rx_streamBuffer;
extern StreamBufferHandle_t bt_tx_streamBuffer;
/**
  * @}
  */

/**
 * \brief 	Aeropendulum communications task.
 * \note 	The goal of this task is to receive commands and send them to the corresponding areas. It also sends data upon request.
 */
void aero_comms(void *pvParameters)
{
	bt_UART5_conf();
	DMA_config();
	uint8_t i;
	uint8_t rx_buffer[32];
	for(i=0;i<8;i++)
	{
		
	}
	while(1)
	{
		if(DMA1->LISR & DMA_LISR_TCIF0)
		{
			for(i=0;i<32;i++)
			{
				if(dma_uart_rx_buffer[i]==0)
				{
					rx_buffer[i]=0;
					break;
				}
				rx_buffer[i]=dma_uart_rx_buffer[i];
			}
			
			/* clear flags */
			DMA1->LIFCR = (3<<4);

			/* reset counter and dma re-enable */
			DMA1_Stream0->NDTR 	 =  (uint8_t)32;
			DMA1_Stream0->CR 	|=  (uint32_t)(1<<0); /* enable dma1 */
			decodeInstruction(rx_buffer, i);
		}
		vTaskDelay(msToTick(100));
	}
}

void decodeInstruction(uint8_t *rx_buffer, uint16_t len)
{
	uint8_t command = -1;
	uint8_t variable[20];
	uint8_t variable_int;
	uint16_t variable_len;
	float	value_float;
	command = getCommand(rx_buffer, len);
	eraseFirstXChars(rx_buffer, len, 2);
	len -= 2;
	variable_int = getVariableInt(variable);
	if(command == 1)
	{
		variable_len = getVariable(rx_buffer, len, variable);
		eraseFirstXChars(rx_buffer, len, variable_len+1);
		len -= variable_len+1;
		value_float = getFloatValue(rx_buffer, len);
	}
	else
	{
		uint16_t i;
		for(i=0;i<len;i++){variable[i] = rx_buffer[i];}
		for(i=len;i<20;i++){variable[i] = 0;}
		variable_len = len;
	}
	switch(command)
	{
		case 0:
			bt_pVariable(variable_int);
			break;
		case 1:
			bt_sVariable(variable_int, value_float);
			break;
		default:
			break;
	}

}

uint8_t getCommand(uint8_t *rx_buffer, uint16_t len)
{
	uint8_t command;
	if(rx_buffer[0]=='p')
	{
		command = 0;
		return command;
	}
	if(rx_buffer[0]=='s')
	{
		command = 1;
		return command;
	}
	return -1;
	
}

uint8_t eraseFirstXChars(uint8_t* string, uint16_t len, uint16_t x)
{
	uint16_t i;
	for(i=x;i<len+x;i++)
	{
		if(i>=len)
		{
			string[i-x] = 0;
		}
		else
		{
			string[i-x] = string[i];
		}
	}
	return len-x;
}

uint16_t getVariable(uint8_t* rx_buffer, uint16_t len, uint8_t* variable)
{
	uint16_t i;
	uint8_t variable_len = len;
	for(i=0;i<len;i++)
	{
		if(rx_buffer[i]==' ')
		{
			variable_len = i;
		}
		if(i<variable_len)
		{
			variable[i] = rx_buffer[i];
		}
		else
		{
			variable[i] = 0;
		}
	}
	return variable_len; 
}

float getFloatValue(uint8_t* rx_buffer, uint16_t len)
{
	uint8_t i, comma_index;
	/* search the  */
	
	for(i=0;i<len;i++)
	{
		if(rx_buffer[i]=='.')
		{
			comma_index = i;
			break;
		}
	}

	float value;
	for(i=0;i<comma_index;i++)
	{
		value += pow(10,((comma_index-1)-i))*(rx_buffer[i]-48);
	}
	for(i=comma_index+1;i<len;i++)
	{
		value += pow(10,(-(i-(comma_index))))*(rx_buffer[i]-48);
	}
	return value;
}

float getFloatValueFromChars(uint8_t* rx_buffer, uint16_t len)
{
	uint8_t* first_address_pointer;
	first_address_pointer = &rx_buffer[0];
	uint8_t i;
	for(i=0;i<4;i++)
	{
		/* if there is no more data */
		if(i>=len)
		{
			*first_address_pointer = (uint8_t)0x00;
		}
		/* increment pointer address */
		first_address_pointer++;
	}

	float* address_pointer =  (float*)rx_buffer;
	float floatValue = *address_pointer;
	return floatValue;
}

uint8_t getVariableInt(uint8_t* variable)
{
	if(variable == (uint8_t*)"onOff") return variables::onOff;
	if(variable == (uint8_t*)"mode") return variables::mode;
	if(variable == (uint8_t*)"angle") return variables::angle;
	if(variable == (uint8_t*)"motorPower") return variables::motorPower;
	if(variable == (uint8_t*)"linearization") return variables::linearization;
	if(variable == (uint8_t*)"Kp") return variables::Kp;
	if(variable == (uint8_t*)"Ki") return variables::Ki;
	if(variable == (uint8_t*)"Kd") return variables::Kd;
	if(variable == (uint8_t*)"Ts") return variables::Ts;
	return -1;
}

/*
 *
 * */
uint8_t bt_pVariable(uint8_t variable)
{
	return 0;
}

/*
 *
 *
 * */
uint8_t bt_sVariable(uint8_t variable, float value)
{
	return 0;
}

uint8_t bt_write(uint8_t* str, uint32_t len)
{
	/* if previous transaction is done, start sending and return 0 */
	/* else return 1 (busy) */
	if(DMA1->HISR & DMA_HISR_TCIF7)
	{
		/* clear flags */
		DMA1->HIFCR = (3<<26);
		/* transmission enable */
		uint32_t i;
		for(i=0;i<len;i++)
		{
			dma_uart_tx_buffer[i] = str[i];
		}
		for(i=len;i<8;i++)
		{
			dma_uart_tx_buffer[i] = 0;
		}
		/* reset counter and dma re-enable */
		DMA1_Stream7->NDTR 	 =  (uint8_t)8;
		/* Enable DMA1 Stream7 */
		DMA1_Stream7->CR 	|=  (uint32_t)(1<<0);
		return 0;
	}
	else
	{
		return 1;
	}
}

void DMA_config()
{
	/* --- Stream0 configuration --- */
	/* Set peripheral address as UART->DR */
	DMA1_Stream0->PAR 	 =  (uint32_t)0x40005004;
	/* Set memory address as array buffer */
	DMA1_Stream0->M0AR 	 =  (uint32_t)&dma_uart_rx_buffer;
	/* Number of data units to end transmission */
	DMA1_Stream0->NDTR 	 =  (uint8_t)32;
	/* Select DMA Stream 0 channel 4 */
	DMA1_Stream0->CR 	|=  (uint32_t)(4<<25);
	/* Configure memory to increment after each transfer */
	DMA1_Stream0->CR 	|=  (uint32_t)(1<<10);
	/* Configure the memory increment to be one byte */
	DMA1_Stream0->CR 	&= ~(uint32_t)(11<<13);
	/* Enable DMA1 Stream0 */
	DMA1_Stream0->CR 	|=  (uint32_t)(1<<0);

	/* --- Stream7 configuration --- */
	/* Set peripheral address as UART->DR */
	DMA1_Stream7->PAR 	 =  (uint32_t)0x40005004;
	/* Set memory address as array buffer */
	DMA1_Stream7->M0AR 	 =  (uint32_t)&dma_uart_tx_buffer;
	/* Number of data units to end transmission */
	DMA1_Stream7->NDTR 	 =  (uint8_t)8;
	/* transfer direction = memory to peripheral */
	DMA1_Stream7->CR 	|=  (uint32_t)(1<<6);
	/* Select DMA Stream 0 channel 4 */
	DMA1_Stream7->CR 	|=  (uint32_t)(4<<25);
	/* Configure memory to increment after each transfer */
	DMA1_Stream7->CR 	|=  (uint32_t)(1<<10);
	/* Configure the memory increment to be one byte */
	DMA1_Stream7->CR 	&= ~(uint32_t)(11<<13);
	/* Enable DMA1 Stream0 */
	DMA1_Stream7->CR 	|=  (uint32_t)(1<<0);

}

void bt_UART5_conf(void)
{
	/* Enable usart */
	UART5->CR1 	|=	 (uint32_t)(1<<13);
	/* 8 data bits */
	UART5->CR1	&=	~(uint32_t)(1<<12);
	/* 1 stop bit */
	UART5->CR2	&=	~(uint32_t)(11<<12);
	/* RXE interrupt enable */
	UART5->CR1	|=	 (uint32_t)(1<<5);
	/* TXE interrupt inhibited */
	UART5->CR1	&=	~(uint32_t)(1<<7);
	/* Parity control disable */
	UART5->CR1	&=	~(uint32_t)(1<<10);
	/* Oversampling 16 bits */
	UART5->CR1	&=	~(uint32_t)(1<<15);
	/* Half duplex mode not selected  */
	UART5->CR3	&=	~(uint32_t)(1<<3);
	/* enable DMA reception */
	UART5->CR3	|=	 (uint32_t)(1<<6);
	/* enable DMA transmission */
	UART5->CR3	|=	 (uint32_t)(1<<7);
	/* clear transmission complete bit */
	UART5->SR	&=	~(uint32_t)(1<<6);	
	/* read enable */
	UART5->CR1	|=  	 (uint32_t)(1<<2);
	/* write enable */
	UART5->CR1	|=  	 (uint32_t)(1<<3);
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
}
