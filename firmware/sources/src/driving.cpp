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
#include "arm_math.h"
/** @}*/

/** \addtogroup defs 
 *  @{
 */
uint8_t PWM_setDuty(float duty);
uint8_t	PWM_setDuty_milli(float duty);
uint8_t motorTest(void);
uint8_t bangBangControl(float desiredAngle);
uint8_t caracterize(void);
float 	feedbackLinearization(float angle, float u0, float mgoverK);
uint8_t pidControl(float set_point);
void linearTest(void);
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
	//pidControl(90);
	//caracterize();
	linearTest();
}

/**
 * \brief Test the linearization
 * \return Does not return
 */
void linearTest(void)
{
	aeropendulum aero;
	aero.motorInit();
	float angle;
	float mgoverK = 0.21515971;
	float u0 = 2.12842924;
	float feedback = 0;

	while(1)
	{
		angle = aero.getEncoderAngle();
		feedback = feedbackLinearization(angle, u0, mgoverK);
		PWM_setDuty_milli(feedback);
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
	float motorPowerH= 140;
	float motorPowerL= 130;
	aeropendulum aero;
	aero.motorInit();
	while(1)
	{
		encoderAngle = aero.getEncoderAngle();
		if(encoderAngle < desiredAngle)
		{
			aero.setMotorPower(motorPowerH);
		}
		else
		{
			aero.setMotorPower(motorPowerL);
		}
		vTaskDelay(msToTick(1));
	}	
	return 0;
}

/*
 * \brief 	Function to carecterize the propellerPower and angle relation.
 * \return 	Does not return.
 * \note 	To avoid the monotone work of printing at each step, break commands can be used in gdb, for example:
 * 			b driving.cpp:148
 * 			p encoderAngle
 * 			p motorPower
 * 			set motorPower += 0.001
 * 			c
 */
uint8_t caracterize(void)
{
	aeropendulum aero;
	float motorPower = 2.2; 
	float encoderAngle = 0;
	float tmpEncoderAngle;
	aero.motorInit();
	PWM_setDuty_milli(motorPower);
	while(1)
	{
		tmpEncoderAngle = aero.getEncoderAngle();
		if((encoderAngle < 1.005*tmpEncoderAngle) && (encoderAngle > 0.995*tmpEncoderAngle))
		{
			encoderAngle = 0;
			PWM_setDuty_milli(motorPower);
		}
		else
		{
		encoderAngle = tmpEncoderAngle;
		vTaskDelay(msToTick(5000));
		}
	}
	return 0;
}

/**
 * \brief 	Feedback Linearization
 * \note 	Given that the system is nonlinear, to avoid the usual linearization between points, we can apply a feedback linearization to cancel the nonlinear term that is to add u0+m*g*sin(angle)/k to the manipulated. The values mg/k were infered with steady state tests.
 * \param	angle	The encoderAngle in radians.
 * \retval 	Feedback linearization constant.
 */
float feedbackLinearization(float angle, float u0, float mgoverK)
{
	//float u0 = 2.116;
	//float mgoverK = AERO_MG_K;
	float sinAngle = arm_sin_f32(angle);
	return u0 + sinAngle*mgoverK;
}

/**
 * \brief 	PID control loop
 * \return 	Does not return
 */
uint8_t pidControl(float set_point)
{
	arm_pid_instance_f32 pid;
	aeropendulum aero;
	float encoderAngle;
	float feedbackTerm;
	float pid_out;
	float duty;
	aero.motorInit();
	pid.Kp = 0.04;
	pid.Ki = 0.0002;
	pid.Kd = 0.01;
	arm_pid_init_f32(&pid, set_point - aero.getEncoderAngle());

	set_point = set_point*3.14159265/180;

	while(1)
	{
		encoderAngle = aero.getEncoderAngle();
		pid_out = arm_pid_f32(&pid, set_point - encoderAngle);
		feedbackTerm = feedbackLinearization(encoderAngle, 0, 0);

		duty = pid_out + feedbackTerm;

		if(duty > 2.5)
		{
			duty = 2.5;
		}
		if(duty < 2.1)
		{
			duty = 2.1;
		}
		
		PWM_setDuty_milli(duty);

		vTaskDelay(msToTick(10));
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
	angle = (((TIM3->CNT)*DEGREES_PER_PULSE)+(AERO_BASE_ANGLE))*PI/180;
	return angle;
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
	PWM_setDuty_milli(2.2);
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
