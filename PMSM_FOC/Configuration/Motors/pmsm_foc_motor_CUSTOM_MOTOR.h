/**
 * @pmsm_foc_motor_CUSTOM_MOTOR.h
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
 * @file pmsm_foc_motor_parameters.h
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
 *
 *
 */

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_MOTOR_PARAMETERS_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_MOTOR_PARAMETERS_H_
#include "../pmsm_foc_user_config.h"

#if(MOTOR_TYPE == CUSTOM_MOTOR)
/* ********************************************* MAXON_MOTOR_267121 **********************************/
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (6.8f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (3865.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (4530.0f)
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 10U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (500U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (500U)
#define USER_RATIO_S                                (1U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
//#define USER_STARTUP_VF_OFFSET_V                   (float) (USER_VDC_LINK_V * 0.05f)                                  /* V/F startup offset in V */
//#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ          (float) (USER_VDC_LINK_V /ELECTRICAL_SPEED_FREQ_HZ)
#define USER_STARTUP_VF_OFFSET_V                    (1.0f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.1f)           /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
#define USER_PI_SPEED_KP                             ((uint16_t)1U<<15U)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI                             ((uint16_t)3)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI                     (10 + USER_RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/
#define USER_PI_TORQUE_KP                            (CALCULATED_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_TORQUE_KI                            (CALCULATED_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_TORQUE_SCALE_KPKI                    (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_FLUX_KP                              (CALCULATED_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_FLUX_KI                              (CALCULATED_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_FLUX_SCALE_KPKI                      (CALCULATED_DEFAULT_SCALING_CURRENT_KPKI + 0)
#define USER_PI_PLL_KP                               ((uint16_t)(1<<8))              /* Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI                               ((uint16_t)(1<<6))              /* (1<<4). Integral gain Ki, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI                       (19 - USER_RES_INC)
/*      --------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN                       (-(((1<<15) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX                       (((1<<15) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */
#define PI_SPEED_UK_LIMIT_MIN                       (16)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX                       (32767)          /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- Torque/Iq PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */

#define PI_TORQUE_IK_LIMIT_MIN                      (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX                      (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MIN                      (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX                      32767         /* U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */

#define PI_FLUX_IK_LIMIT_MIN                        (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX                        (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MIN                        (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX                        32767         /* U[k] output limit HIGH. Normally no need change. */
/*      --------------------------------------------------- PLL Estimator PI Controller Parameters ---------------------------------------- */
/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN                         (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX                         ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */
#define PI_PLL_UK_LIMIT_MIN                         ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX                         (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/

#define USER_ROTOR_PREPOSITION_TIME_MS              (100U)            /* Rotor startup pre alignment time in miliseconds */
#define USER_REFERENCE_SPEED_RPM                    (200U)            /* V/F open loop only Reference Speed */
#define USER_SPEED_THRESHOLD_FW_RPM_PER_S           (10000U)        /* Threshold speed to use Flux Weakening */
#define PWM_THRESHOLD_USEC                          (2U)       /* PWM THRESHOLD to switch from 3 to 2 shunt at high speed */

#endif

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_MOTOR_PARAMETERS_H_ */
