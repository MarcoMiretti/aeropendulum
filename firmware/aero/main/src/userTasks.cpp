/**---------------------------------------------------------------------------/
 * \author 	Marco Miretti <https://github.com/MarcoMiretti>
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 *---------------------------------------------------------------------------*/

#include "userTasks.h"
#include "lpc4337_HAL.h"

/**
 * \addtogroup LED Functions
 * @{
 */
 /**
 * \addtogroup Function prototypes
 * @{
 */
static int RGB_White(void);
static int RGB_Off(void);
/**@}*/
/**@}*/

/**
 * \brief Blinky task
 */
void fancyBlink( void* taskParmPtr )
{
	RGB_White();
	vTaskDelay( 1000 / portTICK_RATE_MS );				/**< Task blocked for 1000ms */
	RGB_Off();

	portTickType xPeriodicity =  200 / portTICK_RATE_MS;		/**< Set task periodicity 200ms */
	portTickType xLastWakeTime = xTaskGetTickCount();
	unsigned volatile int i,j;

	/** Repeat task */
	while(1) {
		for(j=0; j<3; j++)
		{
			for(i=j; i<3; i++)
			{
				iGPIO_Write(5, i-j, 1);
				vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
			}
			for(i=j; i<3; i++)
			{
				iGPIO_Write(5, i-j, 0);
				vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
			}
		}
	}
}

/**
 * \brief 	Turns RGB LED white.
 * \return 	0 if success.
 */
int RGB_White(void)
{
	iGPIO_Write(5, 0, 1);
	iGPIO_Write(5, 1, 1);
	iGPIO_Write(5, 2, 1);
	return 0;
}

/**
 * \brief 	Turns RGB LED off.
 * \return 	0 if success.
 */
int RGB_Off(void)
{
	iGPIO_Write(5, 0, 0);
	iGPIO_Write(5, 1, 0);
	iGPIO_Write(5, 2, 0);
	return 0;
}
