/**
 * \file	comms.h
 * \author 	Marco Miretti \see https://github.com/MarcoMiretti
 * \copyright 	License: gpl-3.0 (see LICENSE.txt)
 * \date 	2019/11/20
 * 
 * \brief 	Comms header.
 */
/*---------------------------------------------------------------------------*/

/* Prevent recursive inclusion */
#ifndef __COMMS_H
#define __COMMS_H

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \addtogroup comm_defs Communication Functions Definitions
 * @{
 */
void USB_VCP_Init(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __COMMS_H */
