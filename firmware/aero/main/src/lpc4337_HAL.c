/**---------------------------------------------------------------------------/
 * \author 	Marco Miretti <https://github.com/MarcoMiretti>
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 *---------------------------------------------------------------------------*/

#include "main.h"
#include "lpc4337_HAL.h"
#include "chip.h"
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

/*-----------------------------------------------------------------/PWM/-----*/
/**
 * \brief Init SCT as PWM for pins 2.10 and 4.4
 * \return 0 if success.
 */
int iPWM_Init(void)
{			
	Chip_SCU_PinMuxSet(0x2, 10, (SCU_MODE_INACT | SCU_MODE_FUNC1));	/**< Pin 2.10 as SCT out2 (LED testigo) */
	Chip_SCU_PinMuxSet(0x4, 4, (SCU_MODE_INACT | SCU_MODE_FUNC1));	/**< Pin 4.4 as SCT out2 (PWM out) */
	
	Chip_SCTPWM_Init(SCT_PWM);					/**< Initializa SCT */
	Chip_SCTPWM_SetRate(SCT_PWM, PWM_FREQ);				/**< Set PWM rate */
	Chip_SCTPWM_SetOutPin(SCT_PWM, 2, 2);				/**< Set Out pin LED */
	Chip_SCTPWM_SetOutPin(SCT_PWM, 4, 4);				/**< Set Out pin PWM out */
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, 2, 0); 			/**< Set duty to zero (init) */
	Chip_SCTPWM_Start(SCT_PWM);					/**< Start PWM */
	
	return 0;
}

/**
 * \brief Set PWM duty percentage.
 * \return 0 if success.
 */
int iPWM_SetDuty(float duty)
{			
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, 2, Chip_SCTPWM_PercentageToTicks(SCT_PWM, duty)); 	/**< Calculate and set duty percentage */
	return 0;
}


