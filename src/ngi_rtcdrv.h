// Modified by NGI 20140729 from original rtcdrv.h.
// Does not clash with the original, you may use both.
/***************************************************************************//**
 * @file
 * @brief Real Time Counter (RTC) driver prototypes and definitions
 * @version 3.20.5
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/


#ifndef __NGI_RTCDRV_H
#define __NGI_RTCDRV_H

#include "em_device.h"
#include "em_cmu.h"

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup RtcDrv
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

uint32_t RTCDRV_DelayInEM(const uint32_t msec_min, const uint32_t msec_max, const unsigned int em);
//void RTCDRV_Trigger(uint32_t msec, void (*cb)(void));
//void RTCDRV_Setup(CMU_Select_TypeDef lfaClockSrc, CMU_ClkDiv_TypeDef rtcPrescale);

void wait(uint32_t msec);


uint32_t wait_from_to(const uint32_t msec_min, const uint32_t msec_max);


#ifdef __cplusplus
}
#endif

/** @} (end group RtcDrv) */
/** @} (end group Drivers) */

#endif
