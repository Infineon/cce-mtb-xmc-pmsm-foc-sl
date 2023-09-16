/**
 * @file pmsm_foc_pi.h
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
 * @file pmsm_foc_pi.h
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
 *
 *
 *
 *
 *     1. KIT_XMC1X_AK_MOTOR_001                 |  KIT_XMC13_BOOT_001    | PMSM_LV15W                 | MAXON_MOTOR_267121                      |* Support 1&3 shunts current sensing, External Op-Amp, BEMF Voltage Sensing Circuit
 *
 *
 *
 *
 *
 *
 */


#ifndef PMSM_FOC_CONTROLMODULES_PMSM_FOC_PI_H_
#define PMSM_FOC_CONTROLMODULES_PMSM_FOC_PI_H_

/*********************************************************************************************************************
 * HEADER FILES
 ***************************************/
#include "../MIDSys/pmsm_foc_current_threeshunt.h"
#include "../MIDSys/pmsm_foc_current_singleshunt.h"
#if(CATCH_FREE_RUNNING == ENABLED)
#include "../MIDSys/pmsm_foc_catch_free_running.h"
#endif
#include "../MIDSys/pmsm_foc_debug.h"
#include "../MIDSys/pmsm_foc_pwmsvm.h"
#include "../MIDSys/pmsm_foc_transitions.h"

/*********************************************************************************************************************
 * MACROS
 ***************************************/
 /*%%%%%%%% Update / Modify PI Parameters Here %%%%%%%%*/


/*********************************************************************************************************************
 * DATA STRUCTURES
 ***************************************/
typedef struct PI_Coefs_Type
{
  int32_t error;										/* PI error signal (reference value � feedback value), error[k] */

  int32_t Uk;											  /* PI output U[k] */
  int32_t Ik;											  /* Integral result I[k] */

  uint16_t Kp;										  /* Proportional gain Kp */
  uint16_t Ki;										  /* Integral gain Ki */
  int16_t Scale_KpKi;								/* Scale-up Kp and Ki by 2^Scale_KpKi */

  int32_t Ik_limit_min;
  int32_t Ik_limit_max;

  int32_t Uk_limit_min;
  int32_t Uk_limit_max;
  uint8_t Uk_limit_status;
} PI_Coefs_Type;



/*********************************************************************************************************************
 * API Prototypes
 ***************************************/
/**
 * @brief Fixed point implementation for filter saturation logic.
 * @param input_val    Value that need to be limited
 * @param higher_limit Maximum value for <i>input_val</i>
 * @param lower_limit  Minimum value for <i>input_val</i>
 * @return int32_t
 * <i>input_val</i>, if <i>lower_limit</i> < <i>input_val</i> < <i>higher_limit</i>
 * <i>higher_limit</i>, if <i>input_val</i> > <i>higher_limit</i>
 * <i>lower_limit</i>, if <i>input_val</i> < <i>lower_limit</i>
 *
 * \par<b>Description: </b><br>
 * This function is used by PI Controller to limit the <i>input_val</i> within its minimum and maximum range.
 *
 * @endcode
 */
__STATIC_INLINE int32_t MIN_MAX_LIMIT(int32_t input_val,int32_t higher_limit,int32_t lower_limit);

/**
 * @brief PI controller
 *      U(t)=Kp x e(t) + (Ki/Ts) x ∫e(t)dt, where Ts is sampling period, e.g.: Ts = 50us.
 *      I[k] = I[k-1] + Ki * error[k]
 *      U[k] = Kp * error[k] + I[k]
 *
 * @param *PI_data
 *      error
 *
 *@retval *PI_data
 */
__STATIC_INLINE void pmsm_foc_pi_controller(int32_t reference, int32_t feedback, PI_Coefs_Type *PI_data);

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ***************************************/
__STATIC_INLINE int32_t MIN_MAX_LIMIT(int32_t input_val,int32_t higher_limit,int32_t lower_limit)
{
  int32_t return_val;
  if ( input_val > higher_limit )
  {
    return_val = higher_limit;
  }
  else if ( input_val < lower_limit )
  {
    return_val = lower_limit;
  }
  else
  {
    return_val = input_val;
  }
  return return_val;
}

__STATIC_INLINE __RAM_FUNC void pmsm_foc_pi_controller(int32_t reference, int32_t feedback, PI_Coefs_Type *PI_data)
{
  int32_t Tmp_Ik_Uk;

  PI_data->error = reference - feedback;

  /* Integral output I[k] = I[k-1] + Ki * error[k] */
  Tmp_Ik_Uk = ((int32_t)PI_data->Ki * PI_data->error) + PI_data->Ik;

  /* Check I[k] limit */
  PI_data->Ik = MIN_MAX_LIMIT(Tmp_Ik_Uk, PI_data->Ik_limit_max, PI_data->Ik_limit_min);

  /* PI output U[k] = Kp * error[k] + I[k] */
  Tmp_Ik_Uk = ((int32_t)PI_data->Kp * PI_data->error) + PI_data->Ik;

  /* Check U[k] output limit */
  PI_data->Uk = MIN_MAX_LIMIT((Tmp_Ik_Uk >> PI_data->Scale_KpKi), PI_data->Uk_limit_max, PI_data->Uk_limit_min);

}

void pmsm_foc_pi_controller_init(void);
__STATIC_INLINE __RAM_FUNC void pmsm_foc_pi_controller_anti_windup(int32_t reference, int32_t feedback,
                                                             PI_Coefs_Type *PI_data);

__STATIC_INLINE __RAM_FUNC void pmsm_foc_pi_controller_anti_windup(int32_t reference, int32_t feedback,
                                                             PI_Coefs_Type *PI_data)
{
  static int32_t Tmp_Ik_Uk;

  PI_data->error = reference - feedback;

  if(PI_data->Uk_limit_status == 0)
  {
    /* Integral output I[k] = I[k-1] + Ki * error[k] */
    Tmp_Ik_Uk = ((int32_t)PI_data->Ki * PI_data->error) + PI_data->Ik;
    PI_data->Ik = MIN_MAX_LIMIT(Tmp_Ik_Uk, PI_data->Ik_limit_max, PI_data->Ik_limit_min);
  }

  /* PI output U[k] = Kp * error[k] + I[k] */
  Tmp_Ik_Uk = ((int32_t)PI_data->Kp * PI_data->error) + PI_data->Ik;
  Tmp_Ik_Uk = Tmp_Ik_Uk >> PI_data->Scale_KpKi;
  /* Check U[k] output limit */
  PI_data->Uk = MIN_MAX_LIMIT(Tmp_Ik_Uk, PI_data->Uk_limit_max, PI_data->Uk_limit_min);
  if(PI_data->Uk != Tmp_Ik_Uk)
  {
    PI_data->Uk_limit_status = 1;
  }
  else
  {
    PI_data->Uk_limit_status = 0;
  }

}

#endif /* PMSM_FOC_CONTROLMODULES_PMSM_FOC_PI_H_ */

/**
 * @}
 */

/**
 * @}
 */
