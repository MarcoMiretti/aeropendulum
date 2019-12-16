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

/**
 * \addtogroup math_const Math Constants 
 * @{
 */
#ifndef PI
#define PI			(float)(3.1415926535) /**< Math constant PI */
#endif /* PI */
/** @} */


/**
 * \addtogroup aero_const Aero physical constants 
 * @{
 */
#define DEGREES_PER_PULSE 	(float)(0.0625) /**< Degrees per encoder pulse */
#define AERO_BASE_ANGLE		(float)(20.625) /**< Aero start angle */

#define PROP_MAX_POWER_MS	(float)(2.35) /**< Max motor power (in duty ms) */ 
#define PROP_MIN_POWER_MS	(float)(2.10) /**< Min motor power (in duty ms) */
#define PROP_POWERON_MS		(float)(1.10) /**< Propeller power-on duty value */
#define PROP_POWER_RANGE	(float)(PROP_MAX_POWER_MS-PROP_MIN_POWER_MS) /**< Propeller power range (in duty ms) */
/** @} */

/**
 * \addtogroup task_defs Task definitions
 * @{
 */
void aero_driving(void *pvParameters);
/** @} */

#endif /* __DRIVING_H */
