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
class aeropendulum {
	public:
		/**
		 * \brief 	Propeller init sequence.
		 * \note 	The motor driver needs to see a pwm signal with 1 ms duty for some time in order to start.
		 * TODO: ver que tiempo es el que se necesita
		 * \retval 	0 if success.
		 */
		uint8_t motorInit(void);
		/**
		 * \brief 	Propeller off.
		 * \note 	Send an 1.1ms pwm signal to the motor, this means no power.
		 * \retval 	0 if success.
		 */
		uint8_t motorOff(void);
		/**
		 * \brief 	Update aeropendulums encoder current angle.
		 * \note 	Since the encoder has 1440 slits, and the quadrature mode detects 4 pules for each slit, it will detect 5760 pulses each full rotation. That is, 1 pulse = 0,0625º or (1/16)º.
		 * \retval 	0 if success.
		 */
		uint8_t	updateAngle();
		
		/**
		 * \brief	Set propeller power (percentual).
		 * \note 	Use PWM Duty, and propeller limits to set the power.
		 * \param 	power in ms
		 * \retval	0 if success
		 * */
		uint8_t	set_motorPower(float);
		
		/**
		 * \brief	Set PID Kd constant.
		 * */
		void	set_PID_Kd(float);
		/**
		 * \brief	Set PID Ki constant.
		 * */
		void	set_PID_Ki(float);
		/**
		 * \brief	Set PID Kp constant.
		 * */
		void	set_PID_Kp(float);
		/**
		 * \brief	Set propeller MK constant.
		 * */
		void	set_propellerConst_MK(float);
		/**
		 * \brief	Set propeller u0 constant.
		 * */
		void	set_propellerConst_u0(float);
		/**
		 * \brief	Set 90 constant.
		 * */
		void	set_over90_compensation_cohef(float);

		/**
		 * \brief	Get propeller power (ms).
		 * \retval 	power in ms
		 * */
		float	get_motorPower(void);
		/**
		 * \brief	Get pendulum angle.
		 * \retval 	angle in radians
		 * */
		float	get_angle(void);
		/**
		 * \brief	Get propeller MK constant.
		 * \retval 	MK constant
		 * */
		float	get_propellerConst_MK(void);
		/**
		 * \brief	Get propeller u0 constant.
		 * \retval 	u0 constant
		 * */
		float	get_propellerConst_u0(void);
		/**
		 * \brief	Get 90 constant.
		 * \retval 	90 constant
		 * */
		float	get_over90_compensation_cohef(void);
		/**
		 * \brief	Get PID kd constant.
		 * \retval 	kd constant
		 * */
		float 	get_PID_Kd(void);
		/**
		 * \brief	Get PID ki constant.
		 * \retval 	ki constant
		 * */
		float 	get_PID_Ki(void);
		/**
		 * \brief	Get PID kp constant.
		 * \retval 	kp constant
		 * */
		float 	get_PID_Kp(void);

	private:
		float	angle;
		float	motorPower;
		float	PID_Kd;
		float	PID_Ki;
		float	PID_Kp;
		float	propellerConst_MK;
		float	propellerConst_u0;
		float	over90_compensation_cohef;
};
/**
  * @brief  Change pwm duty value.
  * @param  duty: integer between 0 and 100 corresponding to the duty 
  * 		percentage.
  * @retval 0 if success TODO: hacer que evalue realmente si la escritura fue exitosa
  */
uint8_t PWM_setDuty(float duty);
/**
  * @brief  Set PWM duty value in miliseconds.
  * @param  duty: float corresponding to the duty value in miliseconds.
  * @retval 0 if success
  */
uint8_t	PWM_setDuty_milli(float duty);
/**
 * \brief Test the linearization
 * \param aero = aeropendulum instance
 * \return Does not return
 */
void 	linearTest(aeropendulum& aero);
/*
 * \brief 	Function to carecterize the propellerPower and angle relation.
 * \return 	Does not return.
 * \note 	To avoid the monotone work of printing at each step, break commands can be used in gdb, for example:
 * 			b driving.cpp:151
 * 			commands 1
 * 				>p encoderAngle
 * 				>p motorPower
 * 				>set motorPower += 0.001
 * 				>c
 * 			set logging on
 * 			c
 */
uint8_t caracterize(aeropendulum& aero);
/**
 * \brief 	Feedback Linearization
 * \note 	Given that the system is nonlinear, to avoid the usual linearization between points, we can apply a feedback linearization to cancel the nonlinear term that is to add u0+m*g*sin(angle)/k to the manipulated. The values mg/k were infered with steady state tests.
 * \param	aero	aeropendulum instance
 * \retval 	Feedback linearization constant.
 */
float 	feedbackLinearization(aeropendulum& aero);
/**
  * \brief	Simple On Off control 
  * \param	aero = aeropendulum instance
  * \param	set_point = desired angle
  * \retval 	0 if success
  */
void bangBangControl(aeropendulum& aero ,float set_point);
/**
 * \brief 	PID control loop
 * \param	aero = aeropendulum instance
 * \param	set_point = desired setpoint in radians
 * \param 	refresh_period = refresh period of the control loop
 * \return 	Does not return
 */
void pidControl(aeropendulum& aero, float set_point, float refresh_period);
/** @} */

void impulse(aeropendulum& aero, float amplitude, float duration_ms);


/**
 * \brief 	Aeropendulum driving task.
 * \note 	The goal of this task is to update the position, calculate the propeller input voltage and drive the motor.
 */
void aero_driving(void *pvParameters)
{
	aeropendulum aero;
	/* aerodynamic constants */
	float mk = 0.21970103*0.93;
	float u0 = 2.12768394;
	float ninetyConst = 0.08;
	aero.set_propellerConst_MK(mk);
	aero.set_propellerConst_u0(u0);
	aero.set_over90_compensation_cohef(ninetyConst);

	/* pid constants */
	float refresh_period = 10;
	float Kp = 0.03;
	float Ki = 0.0001;
	float Kd = 0.003;
	aero.set_PID_Kp(Kp);
	aero.set_PID_Ki(Ki);
	aero.set_PID_Kd(Kd);

	/* set point */
	float set_point = PI/2;

	//impulse(aero, 2.21, refresh_period);
	pidControl(aero, set_point, refresh_period);
	//caracterize(aero);
	//linearTest(aero);
}

void impulse(aeropendulum& aero, float amplitude, float duration_ms)
{
	aero.motorInit();
	vTaskDelay(msToTick(duration_ms));
	aero.motorOff();
	while(1)
	{
	
	}
}

/**
 * \addtogroup aero_member_functions Aero members
 * @{
 */
uint8_t aeropendulum::motorInit(void)
{
	PWM_setDuty_milli(PROP_POWERON_MS);
	vTaskDelay(msToTick(1000));
	return PWM_setDuty_milli(2.2);
}

uint8_t aeropendulum::motorOff(void)
{
	return PWM_setDuty_milli(PROP_POWERON_MS);

}

uint8_t aeropendulum::updateAngle(void)
{
	uint16_t COUNT;
	if(TIM3->CNT == 0xFFFF) COUNT = 1;
	else 		COUNT = TIM3->CNT;
	aeropendulum::angle = (((COUNT)*DEGREES_PER_PULSE)+(AERO_BASE_ANGLE))*PI/180;
	return 0;
}

/**
 * \addtogroup aero_setters Aero setters
 * @{
 */
uint8_t aeropendulum::set_motorPower(float power)
{
	aeropendulum::motorPower = power;
	return PWM_setDuty_milli(power);
}

void aeropendulum::set_PID_Kd(float Kd)
{
	aeropendulum::PID_Kd = Kd;
}

void aeropendulum::set_PID_Ki(float Ki)
{
	aeropendulum::PID_Ki = Ki;
}

void aeropendulum::set_PID_Kp(float Kp)
{
	aeropendulum::PID_Kp = Kp;
}

void aeropendulum::set_propellerConst_MK(float MK)
{
	aeropendulum::propellerConst_MK = MK;
}

void aeropendulum::set_propellerConst_u0(float u0)
{
	aeropendulum::propellerConst_u0 = u0;
}

void aeropendulum::set_over90_compensation_cohef(float ninetyConst)
{
	aeropendulum::over90_compensation_cohef = ninetyConst;
}
/** @} */

/**
 * \addtogroup aero_getters Aero getters
 * @{
 */
float aeropendulum::get_angle()
{
	return aeropendulum::angle;
}

float aeropendulum::get_motorPower()
{
	return aeropendulum::motorPower;
}

float aeropendulum::get_PID_Kd()
{
	return aeropendulum::PID_Kd;
}

float aeropendulum::get_PID_Ki()
{
	return aeropendulum::PID_Ki;
}

float aeropendulum::get_PID_Kp()
{
	return aeropendulum::PID_Kp;
}

float aeropendulum::get_propellerConst_MK()
{
	return aeropendulum::propellerConst_MK;
}

float aeropendulum::get_propellerConst_u0()
{
	return aeropendulum::propellerConst_u0;
}

float aeropendulum::get_over90_compensation_cohef()
{
	return aeropendulum::over90_compensation_cohef;
}

/** @} */ //Getters
/** @} */ //Aero members

/**
 * \addtogroup pwm_functions PWM Functions
 * @{
 */
uint8_t PWM_setDuty(float duty)
{
	uint16_t tempccr3 = 0;
	tempccr3 = (TIM4_ARR*duty)/100;
	TIM4->CCR3 = tempccr3;
	return 0;
}

uint8_t PWM_setDuty_milli(float duty)
{
	return PWM_setDuty(duty*PWM_FREQ/10);
}
/** @} */


/**
 * \addtogroup linearization Linearization
 * @{
 */
void linearTest(aeropendulum& aero)
{
	aero.motorInit();
	while(1)
	{
		aero.updateAngle();
		aero.set_motorPower(feedbackLinearization(aero));
		vTaskDelay(msToTick(10));
	}
}



uint8_t caracterize(aeropendulum& aero)
{
	float motorPower = 2.2; 
	float encoderAngle = 0;
	float tmpEncoderAngle;
	aero.motorInit();
	aero.set_motorPower(motorPower);
	while(1)
	{
		aero.updateAngle();
		tmpEncoderAngle = aero.get_angle();
		if((encoderAngle < 1.005*tmpEncoderAngle) && (encoderAngle > 0.995*tmpEncoderAngle))
		{
			encoderAngle = 0;
			aero.set_motorPower(motorPower);
		}
		else
		{
		encoderAngle = tmpEncoderAngle;
		vTaskDelay(msToTick(1000));
		}
	}
	return 0;
}

float feedbackLinearization(aeropendulum& aero)
{
	float MK = aero.get_propellerConst_MK();
	float u0 = aero.get_propellerConst_u0();
	float sinAngle = arm_sin_f32(aero.get_angle());
	if(aero.get_angle() < PI/2) return u0 + sinAngle*MK;
	else return u0 + sinAngle*MK - (aero.get_angle()-(PI/2))*aero.get_over90_compensation_cohef();
}
/** @} */

/**
 * \addtogroup control_loops Control Loops
 * @{
 */
void bangBangControl(aeropendulum& aero, float set_point)
{
	aero.motorInit();
	while(1)
	{
		aero.updateAngle();
		
		if(aero.get_angle() < set_point)
		{
			aero.set_motorPower(feedbackLinearization(aero)+0.02);
		}
		else
		{
			aero.set_motorPower(feedbackLinearization(aero)-0.02);
		}
		vTaskDelay(msToTick(10));
	}	
}

void pidControl(aeropendulum& aero , float set_point, float refresh_period)
{
	arm_pid_instance_f32 pid;
	float feedbackTerm;
	float pid_out;
	float duty;

	aero.motorInit();
	pid.Kp = aero.get_PID_Kp();
	pid.Ki = aero.get_PID_Ki();
	pid.Kd = aero.get_PID_Kd();
	
	arm_pid_init_f32(&pid, 1);


	while(1)
	{
		aero.updateAngle();

		pid_out = arm_pid_f32(&pid, set_point - aero.get_angle());
		feedbackTerm = feedbackLinearization(aero);

		duty = pid_out + feedbackTerm;

		/* Limit duty TODO: programm anti windup */
		if(duty > 2.35)
		{
			duty = 2.35;
		}
		if(duty < 2.1)
		{
			duty = 2.1;
		}
		
		aero.set_motorPower(duty);
		vTaskDelay(msToTick(refresh_period));
	}
}
/** @} */
