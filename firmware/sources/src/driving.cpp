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
#include "main.h"
#include "driving.h"
#include "math.h"
/** @}*/

/** \addtogroup defs 
 *  @{
 */

/**
 * \brief The aeropendulum class is in charge of creating an aeropendulum instance. Said instance has all the aeropendulum mechanical caracteristics, its options, and the sensor/actuator drivin methods.
 **/
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
		 * \brief	Set onOff status.
		 * */
		void set_onOff(uint8_t);
		/**
		 * \brief	Set operation mode.
		 * */
		void set_mode(uint8_t);
		/**
		 * \brief	Set tracking mode.
		 * */
		void set_tracking(uint8_t);
		/**
		 * \brief	Set PRBS amplitude.
		 * */
		void	set_prbs_amplitude(float);
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
		 * \brief	Set setPoint.
		 * */
		void	set_setPoint(float);

		/**
		 * \brief	Get onOff status.
		 * \retval 	onOff status
		 * */
		uint8_t 	get_onOff(void);
		/**
		 * \brief	Get operation mode.
		 * \retval 	operation mode
		 * */
		uint8_t 	get_mode(void);
		/**
		 * \brief	Get tracking.
		 * \retval 	tracking	
		 * */
		uint8_t 	get_tracking(void);
		/**
		 * \brief	get PRBS amplitude.
		 * \retval 	PRBS amplitude
		 * */
		float	get_prbs_amplitude(void);
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

	private:
		uint8_t onOff;
		uint8_t mode;
		uint8_t tracking;
		float	prbs_amplitude;
		float	setPoint;
		float	angle;
		float	motorPower;
		float	propellerConst_MK;
		float	propellerConst_u0;
		float	over90_compensation_cohef;
};

/**
 * \brief This class can create an pid_controller object. Said object has all the necessary variables an methods to give a manipulated variable given an error.
 *
 **/
class pid_controller
{
	public:
		/**
		 * \brief	Calculate a and b coeficients.
		 * */
		void	init(void);		
		/**
		 * \brief	Detects saturation
		 * \note	If the motors are saturating (u0 is under or over limits) then u0 is set to that limit. Furthermore a saturating flag is risen, depending on the direction of the saturation. 
		 */
		float saturationDetector(void);
		/**
		 * \brief	Detects possible integrator windup and clamps or unclamps integration
		 * \param	error = the error used to see the direction of integration
		 */
		void integratorClamping(float);
		/**
		 * \brief Update parameters according to gains
		 */
		void update_parameters(void);
		/**
		 * \brief 	Update manipulated variable according to PID params and previous states
		 * \param	error = difference between set_point and actual value
		 * \return	the manipulated variable
		 */
		float update_manipulated(float);
		/**
		 * \brief	Set PID Kd constant.
		 * */
		void	set_Kd(float);
		/**
		 * \brief	Set PID Ki constant.
		 * */
		void	set_Ki(float);
		/**
		 * \brief	Set PID Kp constant.
		 * */
		void	set_Kp(float);
		/**
		 * \brief	Set PID Kd command constant.
		 * */
		void	set_Kd_c(float);
		/**
		 * \brief	Set PID Ki command constant.
		 * */
		void	set_Ki_c(float);
		/**
		 * \brief	Set PID Kp command constant.
		 * */
		void	set_Kp_c(float);
		/**
		 * \brief	Set PID Kd reject constant.
		 * */
		void	set_Kd_r(float);
		/**
		 * \brief	Set PID Ki reject constant.
		 * */
		void	set_Ki_r(float);
		/**
		 * \brief	Set PID Kp reject constant.
		 * */
		void	set_Kp_r(float);
		/**
		 * \brief	Set PID Ts constant.
		 * */
		void	set_Ts(float);
		/**
		 * \brief	Set lowpass filter constant.
		 * */
		void	set_N(float);
		/**
		 * \brief	Set windup high limit.
		 * */
		void	set_wH(float);
		/**
		 * \brief	Set windup low limit.
		 * */
		void	set_wL(float);
		/**
		 * \brief	Set feedback linearization constant.
		 * */
		void	set_feedback_linearization(float);
		/**
		 * \brief	Set command mode vs reject mode threshold.
		 * */
		void	set_command_vs_reject_thres(float);
		
		
		/**
		 * \brief	Get PID kd constant.
		 * \retval 	kd constant
		 * */
		float 	get_Kd(void);
		/**
		 * \brief	Get PID ki constant.
		 * \retval 	ki constant
		 * */
		float 	get_Ki(void);
		/**
		 * \brief	Get PID kp constant.
		 * \retval 	kp constant
		 * */
		float 	get_Kp(void);
		/**
		 * \brief	Get PID kd command constant.
		 * \retval 	kd constant
		 * */
		float 	get_Kd_c(void);
		/**
		 * \brief	Get PID ki command constant.
		 * \retval 	ki constant
		 * */
		float 	get_Ki_c(void);
		/**
		 * \brief	Get PID kp command constant.
		 * \retval 	kp constant
		 * */
		float 	get_Kp_c(void);
		/**
		 * \brief	Get PID kd reject constant.
		 * \retval 	kd constant
		 * */
		float 	get_Kd_r(void);
		/**
		 * \brief	Get PID ki reject constant.
		 * \retval 	ki constant
		 * */
		float 	get_Ki_r(void);
		/**
		 * \brief	Get PID kp reject constant.
		 * \retval 	kp constant
		 * */
		float 	get_Kp_r(void);
		/**
		 * \brief	Get PID Ts constant.
		 * \retval 	Ts constant
		 * */
		float 	get_Ts(void);
		/**
		 * \brief	Get lowpass filter constant.
		 * \retval 	N constant
		 * */
		float 	get_N(void);
		/**
		 * \brief	Get windup high limit.
		 * \retval 	wH constant
		 * */
		float 	get_wH(void);
		/**
		 * \brief	Get windup low limit.
		 * \retval 	wL constant
		 * */
		float 	get_wL(void);
		/**
		 * \brief	Get feedback linearization.
		 * \retval 	feedback linearization constant
		 * */
		float 	get_feedback_linearization(void);
		/**
		 * \brief	Get command mode vs reject mode threshold angle.
		 * \retval 	feedback linearization constant
		 * */
		float 	get_command_vs_reject_thres(void);
	private:

		float	Kd;
		float	Ki;
		float	Kp;
	
		float	Kd_c;
		float	Ki_c;
		float	Kp_c;

		float	Kd_r;
		float	Ki_r;
		float	Kp_r;
	

		float	command_vs_reject_thres;

		float	Ts;	/**< Sampling time */
		float	N;	/**< Derivative term lowpass filter constant */
		float	wH;	/**< Windup High limit */
		float	wL;	/**< Windup Low limit */
		
		float	is_saturating_H;
		float	is_saturating_L;
		float	clamping_flag;
		float	tmpKi;
		float	feedback_linearization;

		// Terms of the difference equation
		float	a[3];
		float	b[3];

		// Controller output
		float	u[3];

		// Error signal
		float	e[3];
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
 * \brief 	PID control update
 * \param	aero = aeropendulum instance
 * \param	pid = pid controller instance
 * \param	set_point = desired setpoint in radians
 * \return 	
 */
void pidControl(aeropendulum& aero, pid_controller& pid, float set_point);
/**
 * \brief 	PID control initialization 
 * \param	aero = aeropendulum instance
 * \param	pid = pid controller instance
 * \param	set_point = desired setpoint in radians
 * \return 	
 */
void pidControlInit(aeropendulum& aero, pid_controller& pid, float set_point);
/** @} */

/**
 * \brief handle a received command, writes to bluetooth or reads from it
 * \param aero aeropendulum instance
 * \param pid pid_controller instance
 * \param receivedCommand command struct with requested command
 * */
void handleRequest(aeropendulum& aero, pid_controller& pid, struct command receivedCommand);

/**
 * \brief handle a queue request and generate a received Command then call handleRequest
 * \param pvParameters queue received from aero_comms task
 * \param aero aeropendulum instance
 * \param pid pid_controller instance
 * */
void commsHandler(void *pvParameters, aeropendulum& aero, pid_controller& pid);
/**
 * \brief 	Aeropendulum driving task.
 * \note 	The goal of this task is to update the position, calculate the propeller input voltage and drive the motor.
 */
void aero_driving(void *pvParameters)
{
	aeropendulum aero;
	pid_controller pid;
	/* aerodynamic constants */
	float mk = 0.14865924;
	float u0 = 2.12410885;
	float prbs_amplitude = 0.015;
	float mode = mode_pidControl;
	float ninetyConst = 0.02;//0.02;
	aero.set_propellerConst_MK(mk);
	aero.set_propellerConst_u0(u0);
	aero.set_over90_compensation_cohef(ninetyConst);
	aero.set_prbs_amplitude(prbs_amplitude);
	aero.set_mode(mode);
/*
	float Kp = 0.0318;
	float Ki = 0.02474;
	float Kd = 0.00613;
*/
	float Kp = 0.03;
	float Ki = 0.00001;
	float Kd = 0.000015;

	float sampling_time = 10;//millisecond
	float derivative_filter_cutoff = 10000;
	float wH = 2.35;
	float wL = 2.1;
	pid.set_Kp(Kp);
	pid.set_Ki(Ki);
	pid.set_Kd(Kd);

	pid.set_Ts(sampling_time);
	pid.set_N(derivative_filter_cutoff);
	pid.set_wH(wH);
	pid.set_wL(wL);
	aero.set_onOff(0);
	/* set point */
	float set_point = PI/2;
	aero.set_setPoint(set_point);
	while(1)
	{	
		/* detect operation modes */
		commsHandler(pvParameters, aero, pid);
		uint16_t track_delay = (uint32_t)(100/Ts);
		uint16_t track_count = 0;
		switch((uint32_t)aero.get_mode())
		{
			case mode_pcOperation:
				/*-- PC operation mode --*/
				/* the aeropendulum is operated only by pc set/get commands */
				aero.motorInit();
				while((uint32_t)aero.get_onOff())
				{
					commsHandler(pvParameters, aero, pid);
					aero.updateAngle();
					if((aero.get_tracking()) && (aero.get_angle()>0.365)) floatWrite(aero.get_angle());
					uint32_t start = xTaskGetTickCount();
				
					vTaskDelay(msToTick(pid.get_Ts())-(xTaskGetTickCount()-start));
				}
				break;
			
			case mode_identification:
				{
					/*-- PC operation mode --*/
					/* the aeropendulum is operated only by pc set/get commands */
					aero.motorInit();
					uint16_t i;
					uint8_t j = 0;
					int prbs[] = {1,-1,1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,1     ,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,1     ,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,1     ,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,1,1,-1,1};
					while(aero.get_onOff())
					{
						if(aero.get_angle()>PI/2)
						{
							for(i=0;i<4*630;i++)
							{
								aero.updateAngle();
								floatWrite(aero.get_angle());
								uint32_t start = xTaskGetTickCount();
								if(j == 0)
								{
									aero.set_motorPower(feedbackLinearization(aero)+prbs[(uint16_t)(i/10)]*aero.get_prbs_amplitude());
									j = 10;
								}
								j--;
								vTaskDelay(msToTick(pid.get_Ts())-(xTaskGetTickCount()-start));
							}
							aero.set_onOff(0);						
						}
						else
						{
							aero.updateAngle();
							commsHandler(pvParameters, aero, pid);
							aero.set_motorPower(feedbackLinearization(aero));
							vTaskDelay(msToTick(pid.get_Ts()));
						}	
					}
				break;
				}
			case mode_pidControl:
				/*-- PID CONTROL --*/
				/* starts motors and sets default values */
				pidControlInit(aero, pid, set_point);
				while((uint32_t)aero.get_onOff())
				{
					uint32_t start = xTaskGetTickCount();
					aero.updateAngle();
					if(aero.get_tracking() && track_count == track_delay)
					{
						floatWrite(aero.get_angle());
						track_count = 0;
					}
					else
					{
						track_count++;
					}
					commsHandler(pvParameters, aero, pid);
					pidControl(aero, pid, set_point);
					vTaskDelay(msToTick(pid.get_Ts())-(xTaskGetTickCount()-start));
				}
				break;
		}
		vTaskDelay(msToTick(10));
	}
}

void commsHandler(void *pvParameters, aeropendulum& aero, pid_controller& pid)
{
	struct command receivedCommand;
	receivedCommand.instruction = 0xFF;	/* reset command */
	while(uxQueueMessagesWaiting(*((QueueHandle_t*)pvParameters)))
	{
		xQueueReceive(*((QueueHandle_t*)pvParameters), &(receivedCommand), (TickType_t)0);
	}
	if(receivedCommand.instruction != 0xFF)
	{
		handleRequest(aero, pid, receivedCommand);
	}
}

void handleRequest(aeropendulum& aero, pid_controller& pid, struct command receivedCommand)
{
	float value;
	switch(receivedCommand.instruction)
	{
	case print:
		switch(receivedCommand.variable)
		{
			case onOff:
				value = aero.get_onOff();
				break;
			case mode:
				value = aero.get_mode();	
				break;
			case tracking:
				value = aero.get_tracking();	
				break;
			case prbs_amplitude:
				value = aero.get_prbs_amplitude();
				break;
			case angle:
				aero.updateAngle();
				value = aero.get_angle();
				break;
			case motorPower:
				value = aero.get_motorPower();
				break;
			case u0:
				value = aero.get_propellerConst_u0();
				break;
			case mK:
				value = aero.get_propellerConst_MK();
				break;
			case Kp:
				value = pid.get_Kp();
				break;
			case Ki:
				value = pid.get_Ki();
				break;
			case Kd:
				value = pid.get_Kd();
				break;
			case Ts:	
				value = pid.get_Ts();
				break;
		}
		/* write the selected value to UART */
		floatWrite(value);
		break;
	case set:
		value = receivedCommand.value;
		switch(receivedCommand.variable)
		{
			case onOff:
				aero.set_onOff((uint8_t)value);
				break;
			case mode:	
				aero.set_mode((uint8_t)value);
				break;
			case tracking:	
				aero.set_tracking((uint8_t)value);
				break;
			case prbs_amplitude:
				aero.set_prbs_amplitude(value);
				break;
			case set_point:
				aero.set_setPoint(value);
				break;
			case motorPower:
				aero.set_motorPower(value);
				break;
			case u0:
				aero.set_propellerConst_u0(value);
				break;
			case mK:
				aero.set_propellerConst_MK(value);
				break;
			case Kp:
				pid.set_Kp(value);
				break;
			case Ki:
				pid.set_Ki(value);
				break;
			case Kd:
				pid.set_Kd(value);
				break;
			case Ts:	
				pid.set_Ts(value);
				break;
		}
		break;
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
	return aeropendulum::set_motorPower(2.2);
}

uint8_t aeropendulum::motorOff(void)
{
	return PWM_setDuty_milli(PROP_POWERON_MS);

}

uint8_t aeropendulum::updateAngle(void)
{
	uint16_t COUNT;
	if(TIM3->CNT & 0x8000)
	{
		COUNT = 1;
	}
	else
	{
		COUNT = TIM3->CNT;
	}
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

void aeropendulum::set_onOff(uint8_t onOff)
{
	aeropendulum::onOff = onOff;
}

void aeropendulum::set_mode(uint8_t mode)
{
	aeropendulum::mode = mode;
}

void aeropendulum::set_tracking(uint8_t tracking)
{
	aeropendulum::tracking = tracking;
}

void aeropendulum::set_prbs_amplitude(float prbs_amplitude)
{
	aeropendulum::prbs_amplitude = prbs_amplitude;
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

void aeropendulum::set_setPoint(float setPoint)
{
	aeropendulum::setPoint = setPoint;
}

/** @} */

/**
 * \addtogroup aero_getters Aero getters
 * @{
 */
uint8_t aeropendulum::get_onOff()
{
	return aeropendulum::onOff;
}

uint8_t aeropendulum::get_mode()
{
	return aeropendulum::mode;
}

uint8_t aeropendulum::get_tracking()
{
	return aeropendulum::tracking;
}

float aeropendulum::get_prbs_amplitude()
{
	return aeropendulum::prbs_amplitude;
}

float aeropendulum::get_angle()
{
	return aeropendulum::angle;
}

float aeropendulum::get_motorPower()
{
	return aeropendulum::motorPower;
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
 * \addtogroup pid_member_functions PID members
 * @{
 */

void pid_controller::init()
{
	pid_controller::update_parameters();
	pid_controller::e[0] = 0;
	pid_controller::e[1] = 0;
	pid_controller::e[2] = 0;
	pid_controller::u[0] = 0;
	pid_controller::u[1] = 0;
	pid_controller::u[2] = 0;
}

void pid_controller::update_parameters()
{
	pid_controller::a[0] = (1+pid_controller::N*pid_controller::Ts);
	pid_controller::a[1] = -(2+pid_controller::N*pid_controller::Ts);
	pid_controller::a[2] = 1;

	pid_controller::b[0] = pid_controller::Kp*pid_controller::a[0] + pid_controller::Ki*pid_controller::Ts*pid_controller::a[0] + pid_controller::Kd*pid_controller::N;
	pid_controller::b[1] = pid_controller::Kp*pid_controller::a[1] - pid_controller::Ki*pid_controller::Ts - 2*pid_controller::Kd*pid_controller::Ts;
	pid_controller::b[2] = pid_controller::Kp + pid_controller::Kd*pid_controller::N;
}

float pid_controller::update_manipulated(float error)
{
	//pid_controller::integratorClamping(error);
	pid_controller::e[2] = pid_controller::e[1];
	pid_controller::e[1] = pid_controller::e[0];
	pid_controller::e[0] = error;

	pid_controller::u[2] = pid_controller::u[1];
	pid_controller::u[1] = pid_controller::u[0];

	pid_controller::u[0] =(-pid_controller::a[1]*pid_controller::u[1] + \
			       -pid_controller::a[2]*pid_controller::u[2] + \
			        pid_controller::b[0]*pid_controller::e[0] + \
			        pid_controller::b[1]*pid_controller::e[1] + \
			        pid_controller::b[2]*pid_controller::e[2])/pid_controller::a[0]; 

	return pid_controller::saturationDetector();
}

float pid_controller::saturationDetector()
{
	/* motor saturation detection */
	if(pid_controller::u[0]+pid_controller::feedback_linearization > pid_controller::wH)
	{
		pid_controller::is_saturating_H = 1;
		pid_controller::u[0] = pid_controller::wH-pid_controller::feedback_linearization;
	}
	else
	{
		pid_controller::is_saturating_H = 0;
	}
	if(pid_controller::u[0]+pid_controller::feedback_linearization < pid_controller::wL)
	{
		pid_controller::is_saturating_L = 1;
		pid_controller::u[0] = pid_controller::wL-pid_controller::feedback_linearization;
	}
	else
	{
		pid_controller::is_saturating_L = 0;
	}
	return pid_controller::u[0];
}


void pid_controller::integratorClamping(float error)
{
	/* if controller saturates and the error tends to saturate even more, then clamp integrator */
	if((error > 0 && pid_controller::is_saturating_H) || (error < 0 && pid_controller::is_saturating_L))
	{
		pid_controller::clamping_flag = 1;
		pid_controller::tmpKi = pid_controller::Ki;
		pid_controller::Ki = 0;
		pid_controller::update_parameters();
	}
	/* when stops saturating, or the error sign changes, unclamp integrator */
	else
	{
		if(pid_controller::clamping_flag)
		{
			pid_controller::Ki = pid_controller::tmpKi;
			pid_controller::update_parameters();
			pid_controller::clamping_flag = 0;
		}
	}
}

/**
 * \addtogroup pid_setters PID setters
 * @{
 */
void pid_controller::set_Kd(float Kd)
{
	pid_controller::Kd = Kd;
}

void pid_controller::set_Ki(float Ki)
{
	pid_controller::Ki = Ki;
}

void pid_controller::set_Kp(float Kp)
{
	pid_controller::Kp = Kp;
}

void pid_controller::set_Kd_c(float Kd)
{
	pid_controller::Kd_c = Kd;
}

void pid_controller::set_Ki_c(float Ki)
{
	pid_controller::Ki_c = Ki;
}

void pid_controller::set_Kp_c(float Kp)
{
	pid_controller::Kp_c = Kp;
}

void pid_controller::set_Kd_r(float Kd)
{
	pid_controller::Kd_r = Kd;
}

void pid_controller::set_Ki_r(float Ki)
{
	pid_controller::Ki_r = Ki;
}

void pid_controller::set_Kp_r(float Kp)
{
	pid_controller::Kp_r = Kp;
}

void pid_controller::set_Ts(float Ts)
{
	pid_controller::Ts = Ts;
}

void pid_controller::set_wH(float wH)
{
	pid_controller::wH = wH;
}

void pid_controller::set_wL(float wL)
{
	pid_controller::wL = wL;
}

void pid_controller::set_N(float N)
{
	pid_controller::N = N;
}

void pid_controller::set_feedback_linearization(float feedback_linearization)
{
	pid_controller::feedback_linearization = feedback_linearization;
}

void pid_controller::set_command_vs_reject_thres(float command_vs_reject_thres)
{
	pid_controller::command_vs_reject_thres = command_vs_reject_thres;
}
/** @} */

/**
 * \addtogroup pid_getters PID getters
 * @{
 */
float pid_controller::get_Kd()
{
	return pid_controller::Kd;
}

float pid_controller::get_Ki()
{
	return pid_controller::Ki;
}

float pid_controller::get_Kp()
{
	return pid_controller::Kp;
}

float pid_controller::get_Kd_c()
{
	return pid_controller::Kd_c;
}

float pid_controller::get_Ki_c()
{
	return pid_controller::Ki_c;
}

float pid_controller::get_Kp_c()
{
	return pid_controller::Kp_c;
}

float pid_controller::get_Kd_r()
{
	return pid_controller::Kd_r;
}

float pid_controller::get_Ki_r()
{
	return pid_controller::Ki_r;
}

float pid_controller::get_Kp_r()
{
	return pid_controller::Kp_r;
}

float pid_controller::get_Ts()
{
	return pid_controller::Ts;
}

float pid_controller::get_N()
{
	return pid_controller::N;
}

float pid_controller::get_wH()
{
	return pid_controller::wH;
}

float pid_controller::get_wL()
{
	return pid_controller::wL;
}

float pid_controller::get_feedback_linearization()
{
	return pid_controller::feedback_linearization;
}

float pid_controller::get_command_vs_reject_thres()
{
	return pid_controller::command_vs_reject_thres;
}
/** @} */
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

float feedbackLinearization(aeropendulum& aero)
{
	float MK = aero.get_propellerConst_MK();
	float u0 = aero.get_propellerConst_u0();
	float sinAngle = sin(aero.get_angle()); // arm_sin_f32(aero.get_angle());
	if(aero.get_angle() < PI/2) return u0 + sinAngle*MK;
	else return u0 + sinAngle*MK - (aero.get_angle()-0.85*(PI/2))*aero.get_over90_compensation_cohef();
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

void pidControlInit(aeropendulum& aero ,pid_controller& pid , float set_point)
{
	aero.motorInit();
	pid.init();
}


void pidControl(aeropendulum& aero ,pid_controller& pid , float set_point)
{
	float feedbackTerm;
	float pid_out;
	float duty;
	aero.updateAngle();
	/*
	if(aero.get_angle()-pid.get_command_vs_reject_thres()>set_point || aero.get_angle()+pid.get_command_vs_reject_thres()<set_point)
	{
		if(!is_command_mode)
		{
			pid.set_Kp(pid.get_Kp_c());	
			pid.set_Ki(pid.get_Ki_c());	
			pid.set_Kd(pid.get_Kd_c());
			pid.update_parameters();	
			is_command_mode = 1;
		}
	}
	else
	{
		if(is_command_mode)
		{
			pid.set_Kp(pid.get_Kp_r());	
			pid.set_Ki(pid.get_Ki_r());	
			pid.set_Kd(pid.get_Kd_r());
			pid.update_parameters();	
			is_command_mode = 0;
		}
	}
	*/
	feedbackTerm = feedbackLinearization(aero);
	pid.set_feedback_linearization(feedbackTerm);

	pid_out = pid.update_manipulated(set_point - aero.get_angle());

	duty = pid_out + feedbackTerm;

	aero.set_motorPower(duty);
}
/** @} */
