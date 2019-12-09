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
#include "main.h"
#include "comms.h"
#include <math.h>


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
void decodeInstruction(uint8_t *rx_buffer, uint16_t len, void* pvParameters);
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
uint8_t getVariableInt(uint8_t* variable, uint8_t len);
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
uint8_t bt_pVariable(uint8_t instruction, uint8_t variable, void* pvParameters);
uint8_t bt_sVariable(uint8_t instruction, uint8_t variable, float value, void* pvParameters);
/** @}*/

uint8_t dma_uart_rx_buffer[32];
uint8_t dma_uart_tx_buffer[8];

/** @addtogroup extern_vars 
  * @{
  */
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
			decodeInstruction(rx_buffer, i, pvParameters);
		}
		vTaskDelay(msToTick(100));
	}
}

void decodeInstruction(uint8_t *rx_buffer, uint16_t len, void* pvParameters)
{
	uint8_t command = -1;
	uint8_t variable[20];
	uint8_t variable_int;
	uint16_t variable_len;
	float	value_float;
	command = getCommand(rx_buffer, len);
	eraseFirstXChars(rx_buffer, len, 2);
	len -= 2;
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
	variable_int = getVariableInt(variable, variable_len);
	switch(command)
	{
		case 0:
			bt_pVariable(command, variable_int, pvParameters);
			break;
		case 1:
			bt_sVariable(command, variable_int, value_float, pvParameters);
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
	uint8_t i, comma_index, is_negative = 0;
	/* search the  */

	/* detect sign */
	if(rx_buffer[0] == '-')
	{
		is_negative = 1;
		for(i=0;i<len-1;i++)
		{
			rx_buffer[i] = rx_buffer[i+1];
		}
		len--;
	}
		
	/* where is the decimal point */
	for(i=0;i<len;i++)
	{
		if(rx_buffer[i]=='.')
		{
			comma_index = i;
			break;
		}
	}

	/* calculate float */
	float value;
	for(i=0;i<comma_index;i++)
	{
		value += pow(10,((comma_index-1)-i))*(rx_buffer[i]-48);
	}
	for(i=comma_index+1;i<len;i++)
	{
		value += pow(10,(-(i-(comma_index))))*(rx_buffer[i]-48);
	}

	if(is_negative)
	{
		value *= -1;
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

uint8_t stringCompare(uint8_t* str1, uint8_t* str2, uint8_t len1, uint8_t len2)
{
	uint8_t i;
	if(len1+1 != len2) return 0;
	for(i=0;i<len1;i++)
	{
		if(str1[i]!=str2[i]) return 0;
	}
	return 1;
}

uint8_t getVariableInt(uint8_t* variable, uint8_t len)
{
	if(stringCompare(variable, (uint8_t*)"onOff", len, sizeof("onOff"))) return onOff;
	if(stringCompare(variable, (uint8_t*)"mode", len, sizeof("mode"))) return mode;
	if(stringCompare(variable, (uint8_t*)"set_point", len, sizeof("set_point"))) return set_point;
	if(stringCompare(variable, (uint8_t*)"angle", len, sizeof("angle"))) return angle;
	if(stringCompare(variable, (uint8_t*)"motorPower", len, sizeof("motorPower"))) return motorPower;
	if(stringCompare(variable, (uint8_t*)"u0", len, sizeof("u0"))) return u0;
	if(stringCompare(variable, (uint8_t*)"mK", len, sizeof("mK"))) return mK;
	if(stringCompare(variable, (uint8_t*)"Kp", len, sizeof("Kp"))) return Kp;
	if(stringCompare(variable, (uint8_t*)"Ki", len, sizeof("Ki"))) return Ki;
	if(stringCompare(variable, (uint8_t*)"Kd", len, sizeof("Kd"))) return Kd;
	if(stringCompare(variable, (uint8_t*)"Ts", len, sizeof("Ts"))) return Ts;
	uint8_t error[] = "unknown";
	bt_write(error, sizeof(error));
	return -1;
}

/*
 *
 * */
uint8_t bt_pVariable(uint8_t instruction, uint8_t variable, void* pvParameters)
{
	TickType_t wait_max_ms = msToTick(50);

	struct command thisCommand;
	thisCommand.instruction = instruction;
	thisCommand.variable = variable;
	thisCommand.value = 0;

	xQueueSendToBack(*((QueueHandle_t*)pvParameters), (void*) &thisCommand, wait_max_ms);
	return 0;
}

/*
 *
 *
 * */
uint8_t bt_sVariable(uint8_t instruction, uint8_t variable, float value, void* pvParameters)
{
	TickType_t wait_max_ms = msToTick(50);
	
	struct command thisCommand;
	thisCommand.instruction = instruction;
	thisCommand.variable = variable;
	thisCommand.value = value;

	xQueueSendToBack(*((QueueHandle_t*)pvParameters), (void*) &thisCommand, wait_max_ms);
	return 0;
}

void floatWrite(float value)
{
	uint8_t i;
	uint8_t fp_offset;
	uint8_t is_negative = 0;
	float tmp;
	uint8_t cn_exp;
	uint8_t value_str[8] = {0,0,0,0,0,0,0,0};
	
	/* detect sign */
	if(value<0)
	{
		value *= -1;
		is_negative = 1;
	}

	/* detect zero */
	if(value == 0)
	{
		value_str[0]='0';
	}


	else
	{
		/* see if value is in avaliable range */
		if(value>9999999 || value<0.00001)
		{
			uint8_t error[] = "out_ran";
			for(i=0;i<8;i++) value_str[i] = error[i];
		}
		else
		{
			/* see where decimal point is */
			cn_exp = 0;
			for(i=0;i<7;i++)
			{
				tmp = value/pow(10,i);
				if(tmp>=1 && tmp<10)
				{
					cn_exp = i;
					break;
				}
			
			}
			tmp = value;
			fp_offset = 0;
			/* write in some kind of scientific notation */
			tmp = tmp*pow(10,-cn_exp);

			/* write the value */
			for(i=0;i<7;i++)
			{
				value_str[i+fp_offset] = (int)tmp;
				tmp -= (int)tmp;
				tmp = tmp*10;
				if(i == cn_exp)
				{
					fp_offset = 1;
					value_str[i+fp_offset] = '.'-48;
				}
			}
			/* transform to ascii */
			for(i=0;i<7;i++)
			{
				value_str[i] += 48;
			}
		}
	}
	/* apply sign */
	if(is_negative)
	{
		for(i=6;i>0;i--)
		{
			value_str[i]=value_str[i-1];
		}
		value_str[0] = '-';
	}
	else
	{
		value_str[7] = '0';
	}

	/* write value */
	bt_write(value_str,sizeof(value_str));
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
