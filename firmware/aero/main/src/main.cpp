/**---------------------------------------------------------------------------/
 * \file	main.cpp
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 * 
 * \brief 	main file, entry point and pin definitions.
 *---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/** \addtogroup Included dependencies
 *  @{
 */
#include "main.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "userTasks.h"

#include "lpc4337_HAL.h"
/** @}"*/
/*---------------------------------------------------------------------------*/
/** \addtogroup Preprocessor defines
 *  @{
 */
/** \addtogroup SCU defines
 *  @{
 */
#define SCU_BASE	(uint32_t)(0x40086000)
#define SFSP2_2		(uint32_t)(0x00000108)
/** @}"*/
/** @}"*/
/*---------------------------------------------------------------------------*/
/** \addtogroup main function definitions 
 *  @{
 */
int iSCUPinConf(void);
/** @}"*/
/*---------------------------------------------------------------------------*/
/**
 * SCU pin configuration.
 * LPC4337 pin electrical an multiplexation settings (functions).
 * Pins are set according to: UM10503, Chapter 17.
 */
int main( void )
{
	iSCUPinConf();			/**< call pin configuration */
	
	/** SET GPIO LED as outputs */
	iGPIO_Init(5, 0, 1);
	iGPIO_Init(5, 1, 1);
	iGPIO_Init(5, 2, 1);
	iGPIO_Init(0, 14, 1);
	iGPIO_Init(1, 11, 1);
	iGPIO_Init(1, 12, 1);
	
	/** Turn the LEDs OFF */
	iGPIO_Write(5, 0, 0);
	iGPIO_Write(5, 1, 0);
	iGPIO_Write(5, 2, 0);
	iGPIO_Write(0, 14, 0);
	iGPIO_Write(1, 11, 0);
	iGPIO_Write(1, 12, 0);
	
	/** Create FreeRTOS task */
	xTaskCreate(fancyBlink, (const char *)"fancyBlink", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);
	
	/** The task scheduler takes control over the programm */
	vTaskStartScheduler();

	while( 1 );
	return 0;
}

/**
 * SCU pin configuration.
 * LPC4337 pin electrical an multiplexation settings (functions).
 * Pins are set according to: UM10503, Chapter 17.
 * \return 0 if success, 1 if Error.
 */
int iSCUPinConf(void)
{
	ADDRESS(SCU_BASE, SFSP2_2) &= ~(uint32_t)(0x07); //!< Reset P2_2 Function
	ADDRESS(SCU_BASE, SFSP2_2) |=  (uint32_t)(0x04); //!< P2_2 Function set to function 4 (GPIO5[2])
	ADDRESS(SCU_BASE, SFSP2_2) &= ~(uint32_t)(0x18); //!< P2_2 PULLR = NOPULL
	return 0;	
}
