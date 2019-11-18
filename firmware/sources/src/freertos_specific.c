/**
 * \file	freertos_specific.c
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 * 
 * \brief 	main file, entry point and pin definitions.
 */

#include "FreeRTOS.h"
#include "task.h"

/**
 * \brief Executes when there are memory allocation problems
 *
 * \note This function will only be called if an API call to create a task, queue or semaphore fails because there is too little heap RAM remaining.
 */
void vApplicationMallocFailedHook( void )
{
	for(;;);
}
/**
 * \brief Executes when there is stack overflow
 *
 * \note This function will only be called if a task overflows its stack.  Note that stack overflow checking does slow down the context switch implementation.
 */
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	for( ;; );
}

void vApplicationIdleHook( void )
{
}

void vApplicationTickHook( void )
{
}


