/**
 * @file pmsm_foc_debug.h
 * @Firmware PMSM_FOC_SL_XMC13_XMC14_V1_5
 * @Modified date: 2019-01-10
 *
 * @cond
 ****************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015-2019, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ******************************************
 *
 * @file pmsm_foc_debug.h
 * @date 09 May, 2017
 * @version 1.0.1
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * @endcond
 *
 */

#ifndef PMSM_FOC_MIDSYS_PMSM_FOC_DEBUG_H_
#define PMSM_FOC_MIDSYS_PMSM_FOC_DEBUG_H_

/*********************************************************************************************************************
 * HEADER FILES
 ***************************************/
#include "../MCUInit/adc.h"
#include "../MCUInit/ccu4.h"
#include "../MCUInit/ccu8.h"
#include "../MCUInit/clock.h"
#include "../MCUInit/gpio.h"
#include "../MCUInit/math_init.h"
#include "../MCUInit/mcuinit.h"
#include "../MCUInit/uart.h"
#include "../MCUInit/wdt.h"
#include "xmc1_flash.h"

/**
 * @addtogroup
 * @{
 */

/**
 * @addtogroup
 * @{
 */
/*********************************************************************************************************************
 * API Prototypes
 ***************************************/
/**
 * @param In04      variable to be debug on P0.4     \n
 * @param In04_Flag 0: 0 < In04 < 2^In04_N           \n
 *                  1: -2^In04_N < In04 < 2^In04_N   \n
 * @param In04_N    resolution of the debug variable \n
 * @param In10      variable to be debug on P1.0     \n
 * @param In10_Flag 0: 0 < In10 < 2^In10_N           \n
 *                  1: -2^In10_N < In04 < 2^In10_N   \n
 * @param In10_N    resolution of the debug variable \n
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * To use CCU4 Debug with 2 Outputs, P0.4 and P1.0 <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void pmsm_foc_ccu4_debug3output(int32_t In04, uint16_t In04_Flag, uint16_t In04_N, int32_t In10, uint16_t In10_Flag,
                       uint16_t In10_N);

/*------------------------------------ Speed Profile (Used when SPEED_CONTROLLED_VF_ONLY|SPEED_CONTROLLED_VF_MET_FOC|SPEED_CONTROLLED_DIRECT_FOC is enabled) ----------------------------------- */
#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
#define MOTOR_SPEED_0                               (0U)
#define MOTOR_SPEED_1                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.1f)
#define MOTOR_SPEED_2                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.2f)
#define MOTOR_SPEED_3                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.3f)
#define MOTOR_SPEED_4                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.4f)
#define MOTOR_SPEED_5                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.5f)
#define MOTOR_SPEED_6                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.6f)
#define MOTOR_SPEED_7                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.7f)
#define MOTOR_SPEED_8                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.8f)
#define MOTOR_SPEED_9                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.9f)
#define MOTOR_SPEED_A                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 1.0f)

#define FOR_MOTOR_SPEED_0                           (0U)
#define FOR_MOTOR_SPEED_1                           (uint32_t)(((MOTOR_SPEED_1 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_2                           (uint32_t)(((MOTOR_SPEED_2 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_3                           (uint32_t)(((MOTOR_SPEED_3 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_4                           (uint32_t)(((MOTOR_SPEED_4 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_5                           (uint32_t)(((MOTOR_SPEED_5 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_6                           (uint32_t)(((MOTOR_SPEED_6 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_7                           (uint32_t)(((MOTOR_SPEED_7 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_8                           (uint32_t)(((MOTOR_SPEED_8 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_9                           (uint32_t)(((MOTOR_SPEED_9 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_A                           (uint32_t)(((MOTOR_SPEED_A - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))

#define ADC_STEP_INC_EACHRPM                        (uint32_t)(50/(USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM) * (1<<12))

/*------------------------------------ Speed Profile (Used when TORQUE_CONTROLLED_DIRECT_FOC is enabled) ------------------------------------------------------------------------------------ */
#elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
#define MOTOR_SPEED_0                               (0U)
#define MOTOR_SPEED_1                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.1f)
#define MOTOR_SPEED_2                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.2f)
#define MOTOR_SPEED_3                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.3f)
#define MOTOR_SPEED_4                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.4f)
#define MOTOR_SPEED_5                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.5f)
#define MOTOR_SPEED_6                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.6f)
#define MOTOR_SPEED_7                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.7f)
#define MOTOR_SPEED_8                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.8f)
#define MOTOR_SPEED_9                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.9f)
#define MOTOR_SPEED_A                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 1.0f)
#define FOR_MOTOR_SPEED_0                           (0U)
#define FOR_MOTOR_SPEED_1                           (uint32_t)(((MOTOR_SPEED_1 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_2                           (uint32_t)(((MOTOR_SPEED_2 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_3                           (uint32_t)(((MOTOR_SPEED_3 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_4                           (uint32_t)(((MOTOR_SPEED_4 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_5                           (uint32_t)(((MOTOR_SPEED_5 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_6                           (uint32_t)(((MOTOR_SPEED_6 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_7                           (uint32_t)(((MOTOR_SPEED_7 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_8                           (uint32_t)(((MOTOR_SPEED_8 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_9                           (uint32_t)(((MOTOR_SPEED_9 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_A                           (uint32_t)(((MOTOR_SPEED_A - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))

#define ADC_STEP_INC_EACHRPM                        (uint32_t)(50/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT) * (1<<12))

/*------------------------------------ Speed Profile (Used when VQ_CONTROLLED_DIRECT_FOC is enabled) ------------------------------------------------------------------------------------ */
#elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
#define MOTOR_SPEED_0                               (0U)
#define MOTOR_SPEED_1                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.1f)
#define MOTOR_SPEED_2                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.2f)
#define MOTOR_SPEED_3                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.3f)
#define MOTOR_SPEED_4                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.4f)
#define MOTOR_SPEED_5                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.5f)
#define MOTOR_SPEED_6                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.6f)
#define MOTOR_SPEED_7                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.7f)
#define MOTOR_SPEED_8                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.8f)
#define MOTOR_SPEED_9                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.9f)
#define MOTOR_SPEED_A                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 1.0f)
#define FOR_MOTOR_SPEED_0                           (0U)
#define FOR_MOTOR_SPEED_1                           (uint32_t)(((MOTOR_SPEED_1 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_2                           (uint32_t)(((MOTOR_SPEED_2 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_3                           (uint32_t)(((MOTOR_SPEED_3 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_4                           (uint32_t)(((MOTOR_SPEED_4 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_5                           (uint32_t)(((MOTOR_SPEED_5 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_6                           (uint32_t)(((MOTOR_SPEED_6 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_7                           (uint32_t)(((MOTOR_SPEED_7 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_8                           (uint32_t)(((MOTOR_SPEED_8 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_9                           (uint32_t)(((MOTOR_SPEED_9 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_A                           (uint32_t)(((MOTOR_SPEED_A - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define ADC_STEP_INC_EACHRPM                        (uint32_t)(50/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT) * (1<<12))
#endif


#endif /* PMSM_FOC_MIDSYS_PMSM_FOC_DEBUG_H_ */

/**
 * @}
 */

/**
 * @}
 */
