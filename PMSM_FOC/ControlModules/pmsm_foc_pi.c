/**
 * @file pmsm_foc_pi.c
 * @Firmware PMSM_FOC_SL_XMC13_XMC14_V1_5
 * @Modified date: 2019-01-10
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
 * @file pmsm_foc_pi.c
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
/*********************************************************************************************************************
 * HEADER FILES
 ***************************************/

#include "pmsm_foc_pi.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ***************************************/

PI_Coefs_Type PI_Speed; /* Speed PI controller. */
PI_Coefs_Type PI_Torque; /* Torque / Iq PI controller. */
PI_Coefs_Type PI_Flux; /* Flux /Id PI controller. */
PI_Coefs_Type PI_PLL; /* PLL rotor speed PI controller. */

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ***************************************/

/* API to initialize PI Controller parameters */
void pmsm_foc_pi_controller_init(void)
{
  /*################### For Speed PI controller ######################*/
	PI_Speed.Kp = (uint16_t)PI_SPEED_KP;
	PI_Speed.Ki = PI_SPEED_KI;
	PI_Speed.Scale_KpKi = PI_SPEED_SCALE_KPKI;

	PI_Speed.Ik_limit_min = PI_SPEED_IK_LIMIT_MIN;
	PI_Speed.Ik_limit_max = PI_SPEED_IK_LIMIT_MAX;
	PI_Speed.Ik_limit_min <<= PI_Speed.Scale_KpKi;		/* Ik (32-bit) limits shift, due to PI Controller design. */
	PI_Speed.Ik_limit_max <<= PI_Speed.Scale_KpKi;
  PI_Speed.Uk_limit_status = 0;

	PI_Speed.Ik = 0;

	PI_Speed.Uk_limit_min = PI_SPEED_UK_LIMIT_MIN;
	PI_Speed.Uk_limit_max = PI_SPEED_UK_LIMIT_MAX;

  /*################### For Torque / Iq PI controller ######################*/
	PI_Torque.Kp = PI_TORQUE_KP;
	PI_Torque.Ki = PI_TORQUE_KI;
	PI_Torque.Scale_KpKi = PI_TORQUE_SCALE_KPKI;

	PI_Torque.Ik_limit_min = PI_TORQUE_IK_LIMIT_MIN;
	PI_Torque.Ik_limit_max = PI_TORQUE_IK_LIMIT_MAX;
	PI_Torque.Ik_limit_min <<= PI_Torque.Scale_KpKi;	/* Ik limits shift. */
	PI_Torque.Ik_limit_max <<= PI_Torque.Scale_KpKi;

	PI_Torque.Ik = 0;

	PI_Torque.Uk_limit_min = PI_TORQUE_UK_LIMIT_MIN;
	PI_Torque.Uk_limit_max = PI_TORQUE_UK_LIMIT_MAX;

  /*################### For Flux / Id PI controller ######################*/
	PI_Flux.Kp = PI_FLUX_KP;
	PI_Flux.Ki = PI_FLUX_KI;
	PI_Flux.Scale_KpKi = PI_FLUX_SCALE_KPKI;

	PI_Flux.Ik_limit_min = PI_FLUX_IK_LIMIT_MIN;
	PI_Flux.Ik_limit_max = PI_FLUX_IK_LIMIT_MAX;
	PI_Flux.Ik_limit_min <<= PI_Flux.Scale_KpKi;		/* Ik limits shift. */
	PI_Flux.Ik_limit_max <<= PI_Flux.Scale_KpKi;

	PI_Flux.Ik = 0;

	PI_Flux.Uk_limit_min = PI_FLUX_UK_LIMIT_MIN;
	PI_Flux.Uk_limit_max = PI_FLUX_UK_LIMIT_MAX;

  /*################### For PLL rotor speed PI controller ######################*/
	PI_PLL.Kp = (uint16_t)PI_PLL_KP;
	PI_PLL.Ki = (uint16_t)PI_PLL_KI;
	PI_PLL.Scale_KpKi = PI_PLL_SCALE_KPKI;

	PI_PLL.Ik_limit_min = (int32_t)PI_PLL_IK_LIMIT_MIN;
	PI_PLL.Ik_limit_max = (int32_t)PI_PLL_IK_LIMIT_MAX;
	PI_PLL.Ik_limit_min <<= PI_PLL.Scale_KpKi;			/* Ik limits shift. */
	PI_PLL.Ik_limit_max <<= PI_PLL.Scale_KpKi;

	PI_PLL.Ik = 0;

	PI_PLL.Uk_limit_min = (int32_t)PI_PLL_UK_LIMIT_MIN;
	PI_PLL.Uk_limit_max = (int32_t)PI_PLL_UK_LIMIT_MAX;


}	/* End of pmsm_foc_pi_controller_init () */



