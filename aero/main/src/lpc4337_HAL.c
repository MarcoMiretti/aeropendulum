/**---------------------------------------------------------------------------/
 * \author 	Marco Miretti <https://github.com/MarcoMiretti>
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 *---------------------------------------------------------------------------*/

#include "main.h"
#include "lpc4337_HAL.h"

/**
 * \addtogroup GPIO
 * @{
 */
/*----------------------------------------------------------------/GPIO/-----*/
/**
 * \brief Initialize specific GPIO PIN.
 * \param PORT: The port of the GPIO to write.
 * \param GPIO: The specific pin of the port.
 * \param RW_BIT: Configures the PIN as input (0) or output(1).
 * \return If success returns a 0, otherwise 1.
 */
int iGPIO_Init(int PORT, int GPIO, int RW_BIT)
{
	if(RW_BIT == 0 | RW_BIT == 1)
	{
		uint32_t GPIO_PORT_DIR_ADDRESS = GPIO_PORT_DIR_BASE + PORT*0x4;
		if(RW_BIT == 1)
		{
			ADDRESS(GPIO_PORT_BASE, GPIO_PORT_DIR_ADDRESS) |=  (uint32_t)(1 << GPIO);
			return 0;
		}
		else
		{
			ADDRESS(GPIO_PORT_BASE, GPIO_PORT_DIR_ADDRESS) &= ~(uint32_t)(1 << GPIO);
			return 0;
		}	
	}
	else
	{
		return 1;
	}
}

/**
 * \brief Write a BIT to a GPIO PIN.
 * \param PORT: The port of the GPIO to write.
 * \param GPIO: The specific pin of the port.
 * \param DATA_BIT: The bit to be written, that is 0 or 1.
 * \return If success returns a 0, otherwise 1.
 */
int iGPIO_Write(int PORT, int GPIO, int DATA_BIT)
{
	if(DATA_BIT == 0 | DATA_BIT == 1)
	{
		volatile unsigned int GPIO_PIN_ADDRESS 	=	 (uint32_t)(0x20)*PORT + GPIO;		/**< PIN address can be calculated */
		if(DATA_BIT == 1)
		{
			ADDRESS(GPIO_PORT_BASE, GPIO_PIN_ADDRESS)	|=	 (uint32_t)(0x01);
			return 0;
		}
		else
		{
			ADDRESS(GPIO_PORT_BASE, GPIO_PIN_ADDRESS)	&=	~(uint32_t)(0x01);
			return 0;
		}	
	}
	else
	{
		return 1;
	}
}
/** @}*/
