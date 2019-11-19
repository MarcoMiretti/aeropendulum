/**
 * \file	driving.cpp
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 * 
 * \brief 	Hardware functions and tasks source.
 * \note 	This file handles everything related to the hardware driving, that is propulsion and measurement.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup deps
 *  @{
 */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "driving.h"
/** @}*/

/** \addtogroup defs 
 *  @{
 */
uint8_t PWM_setDuty(float duty);
uint8_t	PWM_setDuty_milli(float duty);
uint8_t motorTest(void);
uint8_t bangBangControl(float desiredAngle);
//uint8_t aero_motorTest(void); 
/** @} */

class aeropendulum {
	public:
		uint8_t motorInit(void);
		uint8_t motorOff(void);
		uint8_t setMotorPower(float);
		float	getMotorPower(void);
		float	getEncoderAngle(void);
	private:
		float	angle;
		float	motorPower;
};


/**
 * \brief 	Aeropendulum driving task.
 * \note 	The goal of this task is to update the position, calculate the propeller input voltage and drive the motor.
 */
void aero_driving(void *pvParameters)
{
	bangBangControl(90);
	while(1)
	{
	}
}


/**
  * @brief  Test the aeropendulum motor with pwm.
  * @retval 0 if success
  */
uint8_t motorTest(void)
{
	aeropendulum aero;
	aero.motorInit();
	aero.setMotorPower(3);
	GPIO_OutData(3,12,0);
	vTaskDelay(msToTick(5000));
	aero.setMotorPower(10);
	GPIO_OutData(3,12,1);
	vTaskDelay(msToTick(5000));
	aero.motorOff();
	return 0;
}

/**
  * @brief	Simple On Off control 
  * @retval 	0 if success
  */
uint8_t bangBangControl(float desiredAngle)
{
	float encoderAngle;
	aeropendulum aero;
	aero.motorInit();
	while(1)
	{
		encoderAngle = aero.getEncoderAngle();
		if(encoderAngle < desiredAngle)
		{
			aero.setMotorPower(140);
		}
		else
		{
			aero.setMotorPower(130);
		}
		vTaskDelay(msToTick(1));
	}	
	return 0;
}



/**
 * \addtogroup aero_member_functions Aero members
 * @{
 */
/**
 * \brief 	Get aeropendulums encoder current angle.
 * \note 	Since the encoder has 1440 slits, and the quadrature mode detects 4 pules for each slit, it will detect 5760 pulses each full rotation. That is, 1 pulse = 0,0625º or (1/16)º.
 * \retval 	float Aeropendulum angle.
 */
float aeropendulum::getEncoderAngle(void)
{
	aeropendulum::angle = (((TIM3->CNT)*DEGREES_PER_PULSE)+(AERO_BASE_ANGLE));
	return aeropendulum::angle;
}

/**
 * \brief	Set propeller power (percentual).
 * \note 	Use PWM Duty, and propeller limits to set the power.
 * \param 	power Float between 0 and 100.
 * \retval	0 if success.
 * */
uint8_t aeropendulum::setMotorPower(float power)
{
	PWM_setDuty_milli((PROP_POWER_RANGE * power / 100)+PROP_MIN_POWER_MS);
	aeropendulum::motorPower = power;
	return 0;
}

/**
 * \brief	Get propeller power (percentual).
 * \retval 	power Float between 0 and 100.
 * */
float aeropendulum::getMotorPower()
{
	return aeropendulum::motorPower;
}

/**
 * \brief 	Propeller init sequence.
 * \note 	The motor driver needs to see a pwm signal with 1 ms duty for some time in order to start.
 * TODO: ver que tiempo es el que se necesita
 * \retval 	0 if success.
 */
uint8_t aeropendulum::motorInit(void)
{
	PWM_setDuty_milli(PROP_POWERON_MS);
	vTaskDelay(1000/portTICK_PERIOD_MS);	
	PWM_setDuty_milli(PROP_MIN_POWER_MS);
	return 0;
}

/**
 * \brief 	Propeller off.
 * \note 	Send an 1.1ms pwm signal to the motor, this means no power.
 * TODO: ver que tiempo es el que se necesita
 * \retval 	0 if success.
 */
uint8_t aeropendulum::motorOff(void)
{
	PWM_setDuty_milli(PROP_POWERON_MS);
	return 0;
}
/** @} */

/**
 * \addtogroup pwm_functions PWM Functions
 * @{
 */
/**
  * @brief  Change pwm duty value.
  * @param  duty: integer between 0 and 100 corresponding to the duty 
  * 		percentage.
  * @retval 0 if success
  */
uint8_t PWM_setDuty(float duty)
{
	uint16_t tempccr3 = 0;
	tempccr3 = (TIM4_ARR*duty)/100;
	TIM4->CCR3 = tempccr3;
	return 0;
}

/**
  * @brief  Set PWM duty value in miliseconds.
  * @param  duty: float corresponding to the duty value in miliseconds.
  * @retval 0 if success
  */
uint8_t PWM_setDuty_milli(float duty)
{
	PWM_setDuty(duty*PWM_FREQ/10);
	return 0;
}
/** @} */
