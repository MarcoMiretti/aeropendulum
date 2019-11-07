/**---------------------------------------------------------------------------/
 * \author 	Marco Miretti <https://github.com/MarcoMiretti>
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/10/30
 *---------------------------------------------------------------------------*/

#ifndef __LPC4337HAL_H__
#define __LPC4337HAL_H__

#ifdef __cplusplus
extern "C" {
#endif
/** \addtogroup SCU
 *  @{
 */
#define SCU_BASE	(uint32_t)(0x40086000)
#define SFSP2_2		(uint32_t)(0x00000108)
#define SFSP2_10	(uint32_t)(0x00000128)
/** @}"*/

/**
 * \addtogroup GPIO
 * @{
 */
#define GPIO_PORT_BASE		(uint32_t)(0x400F4000)
#define GPIO_PORT_DIR_BASE	(uint32_t)(0x00002000)
/**@}*/

/**
 * \addtogroup PWM
 * @{
 */
#define PWM_FREQ		(10000)
#define PWM_PERIOD		(SystemCoreClock / (PWM_FREQ * 2))
#define SCT_PWM            	LPC_SCT
/**@}*/

/**
 * \addtogroup Function-like macros
 * @{
 */
#define ADDRESS(base, offset) (*(volatile int *)(volatile char *) ((base)+(offset)))
/**@}*/

/**
 * \addtogroup Function prototypes
 * @{
 */
int iGPIO_Init(int PORT, int GPIO, int RW_BIT);
int iGPIO_Write(int PORT, int GPIO, int DATA_BIT);
int iPWM_Init(void);
int iPWM_SetDuty(float duty);
/**@}*/

#ifdef __cplusplus
}
#endif

#endif /* __LPC4337HAL_H__ */
