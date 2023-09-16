/**
 * @file mcuinit.h
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
 * @file mcuinit.h
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

#ifndef PMSM_FOC_MCUINIT_MCUINIT_H_
#define PMSM_FOC_MCUINIT_MCUINIT_H_

#include "adc.h"
#include "ccu4.h"
#include "ccu8.h"
#include "gpio.h"
#include "wdt.h"
#include "clock.h"
#include "math_init.h"
#include "uart.h"
#include "../Configuration/pmsm_foc_variables_scaling.h"


/**
 * @addtogroup
 * @{
 */

/**
 * @addtogroup
 * @{
 */
#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************************************************************
 * API Prototypes
 ***************************************/
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes XMC1302 MCU and its peripherals for PMSM FOC motor control. <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void pmsm_foc_init(void);
void pmsm_motorcontrol_init (void);
void pmsm_foc_init_variables_cfr_motor(void);
void pmsm_foc_variables_init (void);
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Synchronous start of CAPCOM modules <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void pmsm_foc_CCUx_SynStart(void);

#ifdef __cplusplus
}
#endif

#endif /* PMSM_FOC_MCUINIT_MCUINIT_H_ */

/**
 * @}
 */

/**
 * @}
 */
