/**
 * @file pmsm_foc_interface.h
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
 * @file pmsm_foc_interface.h
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

#ifndef PMSM_FOC_CONTROLMODULES_PMSM_FOC_INTERFACE_H_
#define PMSM_FOC_CONTROLMODULES_PMSM_FOC_INTERFACE_H_

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
#ifdef __cplusplus
extern "C" {
#endif
/*********************************************************************************************************************
 * API PROTOTYPES
 ***************************************/
void pmsm_foc_motor_hold (void);
//extern void pmsm_foc_get_adcphasecurrent(uint16_t Previous_SVM_SectorNo, uint16_t New_SVM_SectorNo, ADCType* const HandlePtr);
void pmsm_foc_bootstrap_charge(void);
void pmsm_foc_vf_openloop_rampup(void);
void pmsm_foc_motor_start(void);
void pmsm_foc_motor_stop(void);
void pmsm_foc_motor_brake(void);

void pmsm_foc_pull_1phase_low(uint16_t phase_no);
void pmsm_foc_motor_coasting(void);
void pmsm_foc_transition_normalstart(void);
//extern void pmsm_phasecurrent_init(void);
int32_t pmsm_foc_get_motor_speed(void);
float pmsm_foc_get_Vdc_link(void);
void pmsm_foc_disable_inverter(void);
void pmsm_foc_enable_inverter(void);
void pmsm_foc_Clear_trap(void);


#if(SETTING_TARGET_SPEED == SET_TARGET_SPEED)
#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
  void pmsm_foc_set_motor_target_speed(int32_t motor_target_speed);
#elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
  void pmsm_foc_set_motor_target_torque(int32_t motor_target_torque);
#elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
  void pmsm_foc_set_motor_target_voltage(int32_t motor_target_voltage);
#endif
#endif

extern void  pmsm_foc_restore_ccu8_adc_passive_level(void);

#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
__RAM_FUNC void pmsm_foc_scurve_ramp_generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val);
__RAM_FUNC void pmsm_foc_linear_ramp_generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val);
#elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
__RAM_FUNC void pmsm_foc_linear_torque_ramp_generator(int32_t current_set, int32_t inc_step,
                                                            int32_t dec_step,
                                                             FOCInputType* const HandlePtr);
#elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
__RAM_FUNC void pmsm_foc_linear_vq_ramp_generator(int32_t current_set, int32_t inc_step,
                                                                           int32_t dec_step,
                                                                           FOCInputType* const HandlePtr);
#endif

#if(CATCH_FREE_RUNNING == ENABLED)
void pmsm_foc_cfr_precharge_bootstrap (void);
void pmsm_foc_cfr_transition_closedloop(CFR_type* const HandlePtr);
#endif
	
#ifdef __cplusplus
}
#endif

#endif /* PMSM_FOC_CONTROLMODULES_PMSM_FOC_INTERFACE_H_ */

/**
 * @}
 */

/**
 * @}
 */
