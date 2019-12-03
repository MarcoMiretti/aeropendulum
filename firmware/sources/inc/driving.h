/**
 * \file	driving.h
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/18
 * 
 * \brief 	Driving header.
 */
/*---------------------------------------------------------------------------*/

/* Prevent recursive inclusion */
#ifndef __DRIVING_H
#define __DRIVING_H

#ifndef PI
#define PI			(float)(3.1415926535)
#endif /* PI */

#define DEGREES_PER_PULSE 	(float)(0.0625)
#define AERO_BASE_ANGLE		(float)(20.625)
#define AERO_MG_K 		(float)(0.227)

#define PROP_MAX_POWER_MS	(float)(2.30) //TODO: define this values propperly
#define PROP_MIN_POWER_MS	(float)(2.10)
#define PROP_POWERON_MS		(float)(1.10)
#define PROP_POWER_RANGE	(float)(PROP_MAX_POWER_MS-PROP_MIN_POWER_MS)


/**
 * \addtogroup task_defs Task definitions
 * @{
 */
void aero_driving(void *pvParameters);
/** @} */

#endif /* __DRIVING_H */
