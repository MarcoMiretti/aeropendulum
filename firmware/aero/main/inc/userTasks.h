/**---------------------------------------------------------------------------/
 * \author 	Marco Miretti <https://github.com/MarcoMiretti>
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 *---------------------------------------------------------------------------*/

#ifndef __USER_TASKS_H__
#define __USER_TASKS_H__

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup Task prototypes
 * @{
 */
void fancyBlink( void* taskParmPtr );
/**@}*/

/**
 * \addtogroup Function prototypes
 * @{
 */
 
/**@}*/
#ifdef __cplusplus
}
#endif

#endif /* __USER_TASKS_H__ */
