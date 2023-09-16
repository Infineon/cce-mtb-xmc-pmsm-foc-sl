/**
 * @file pmsm_foc_directfoc_startup.h
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
 * @file pmsm_foc_directfoc_startup.h
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
#ifndef PMSM_FOC_CONTROLMODULES_PMSM_FOC_DIRECTFOC_STARTUP_H_
#define PMSM_FOC_CONTROLMODULES_PMSM_FOC_DIRECTFOC_STARTUP_H_

/*********************************************************************************************************************
 * HEADER FILES
 ***************************************/
#include "../MIDSys/pmsm_foc_current_threeshunt.h"
#include "../MIDSys/pmsm_foc_current_singleshunt.h"
#if(CATCH_FREE_RUNNING == ENABLED)
#include "../MIDSys/pmsm_foc_catch_free_running.h"
#endif
#include "../MIDSys/pmsm_foc_transitions.h"
#include "../MIDSys/pmsm_foc_debug.h"
#include "../MIDSys/pmsm_foc_pwmsvm.h"
/**
 * @addtogroup
 * @{
 */

/**
 * @addtogroup
 * @{
 */

/*********************************************************************************************************************
 * API PROTOTYPES
 ***************************************/

/**
 * @param none \n
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 *  Rotor initial preposition/alignment <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void pmsm_foc_directfocrotor_pre_positioning(void);

#endif /* PMSM_FOC_CONTROLMODULES_PMSM_FOC_DIRECTFOC_STARTUP_H_ */

/**
 * @}
 */

/**
 * @}
 */
