/*
 * pmsm_foc_const.h
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
 * @file pmsm_foc_user_config.h
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
 *
 *
 *
 *
 *
 *
 *
 *
 */

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_CONST_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_CONST_H_

#include "pmsm_foc_user_config.h"

/*********************************************************************************************************************
 * VADC constant Resources
 ****************************************/
#if (CURRENT_SENSING ==  USER_THREE_SHUNT_SYNC_CONV)
#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL
#endif
/*
 * end of section
 * */

#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
  #define MOTOR_HOLD_THRESHOLD                                (Motor.Target_Speed <= USER_SPEED_LOW_LIMIT_RPM )
#elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
  #define MOTOR_HOLD_THRESHOLD                                (Motor.Target_Torque <=  USER_IQ_REF_LOW_LIMIT)
#elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
  #define MOTOR_HOLD_THRESHOLD                                (Motor.Target_Voltage <= USER_VQ_REF_LOW_LIMIT)
#endif

/*      --------------------------------------------------- SVM with Pseudo Zero Vectors ---------------------------------------- */
#define USER_INVERSE_SVM_LAMDA                      (float)(20.0)
/*      --------------------------------------------------- MCU Parameters ---------------------------------------- */

#if(UC_SERIES == XMC14)
	#define USER_MCLK_FREQ_MHz                          (48U)       /* CPU Clock in Mhz*/
	#define USER_PCLK_FREQ_MHz                          (96U)       /* Peripheral CLK frequency = double of CPU Clock */
#else
	#define USER_MCLK_FREQ_MHz                          (32U)       /* CPU Clock in Mhz*/
	#define USER_PCLK_FREQ_MHz                          (64U)       /* Peripheral CLK frequency = double of CPU Clock */
#endif
#define USER_CORDIC_MPS                             (2.0f)      /* CORDIC module MPS Setting value */
#define CORDIC_K                                    (1.646760258f)                  /* CORDIC SCALE (Inherent Gain Factor) */
#define CORDIC_SHIFT                                (14U)             /* 8 ~ 16. Shift for CORDIC input / output registers, whose [7:0] are 0x00. Normally no need change.*/
/*      --------------------------------------------------- Parameters for Startup Lock / Stall Detection ---------------------------------------- */
#define USER_MAX_RETRY_MOTORSTARTUP_STALL           (3U)           /* Max retry of motor startup if stall  */
/*      --------------------------------------------------- MISC Constant ---------------------------------------- */
#define USER_SQRT_3_CONSTANT                        (1.7320508075f)
#define USER_CCU4_DEBUG_KHZ                         (160U)
#define USER_PI                                     (3.1415926536f)
#define SPEED_TO_RPM_SCALE                          (11U)
#define SHIFT_BIAS_LPF                              (3U)                    /* Shift times for unity gain LPF: Y[n] = Y[n-1] + (X[n]-Y[n-1])>>SHIFT_BIAS_LPF. */

#define SQRT3                                       (1.732050807569F)       /* √3 */
#define DIV_SQRT3                                   (591)                  /* ((int16_t)((1/SQRT3) * (1<<SCALE_SQRT3))) */
#define DIV_SQRT3_Q14                               (9459U)
#define SCALE_DIV_3                                 (14U)                   /* For 1/3 scaling. */
#define DIV_3                                       (5461U)                 /* ((int16_t)((1/3) * (1<<SCALE_DIV_3))) */

#define DEGREE_90                                   (4194304U << 8U)        /* 90° angle (0 ~ 2^23 represent electrical angle 0° ~ 180° in CORDIC) */
#define DEGREE_X                                    (DEGREE_90 * 1U)        /* X = 0°, 90°, 180°, or 270° */
#define DEGREE_SHIFT                                (652448U << 8U)         /* 14° angle shift */

#define CORDIC_VECTORING_MODE                       (0x62)                  /* CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default).*/
#define CORDIC_ROTATION_MODE                        (0x6A)                  /*  CORDIC: Circular Rotation Mode. MPS: Divide by 2 (default).*/


#define PMSM_FOC_SECONDARYLOOP_CALLBACK       ENABLED  /*ENABLED or DISABLED, pmsm_foc_secondaryloop_callback() to be define*/



#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_CONST_H_ */
