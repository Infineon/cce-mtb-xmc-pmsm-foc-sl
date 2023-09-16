/**
 * @file pmsm_foc_variables_scaling.h
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
 * @file pmsm_foc_variables_scaling.h
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
 */

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_VARIABLES_SCALING_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_VARIABLES_SCALING_H_


#include "pmsm_foc_user_config.h"
#include "pmsm_foc_const.h"

#include INVERTERCARD_TYPE_PATH
#include MCUCARD_TYPE_PATH
#include MOTOR_TYPE_PATH
#include <math.h>



#define PI_SPEED_KP                                 USER_PI_SPEED_KP
#define PI_SPEED_KI                                 USER_PI_SPEED_KI                              /* (1<<3). Integral gain Ki, uint16_t. */
#define PI_SPEED_SCALE_KPKI                         USER_PI_SPEED_SCALE_KPKI                      /* RES_INC: Angle/speed resolution increase from 16 bit.*/
#define PI_TORQUE_KP                                USER_PI_TORQUE_KP
#define PI_TORQUE_KI                                USER_PI_TORQUE_KI
#define PI_TORQUE_SCALE_KPKI                        USER_PI_TORQUE_SCALE_KPKI
#define PI_FLUX_KP                                  USER_PI_FLUX_KP
#define PI_FLUX_KI                                  USER_PI_FLUX_KI
#define PI_FLUX_SCALE_KPKI                          USER_PI_FLUX_SCALE_KPKI
#define PI_PLL_KP                                   USER_PI_PLL_KP
#define PI_PLL_KI                                   USER_PI_PLL_KI
#define PI_PLL_SCALE_KPKI                           USER_PI_PLL_SCALE_KPKI


/* ********************************************* MCU Parameters  *********************************/
#define CCU8_PERIOD_REG                             ((uint32_t)(USER_PCLK_FREQ_MHz*1000000)/(USER_CCU8_PWM_FREQ_HZ*(CCU8_MODULE_PRESCALER_VALUE+1)))
#define CCU8_TIME_T0_THRESHOLD                      ((uint32_t)1000000/(PWM_THRESHOLD_USEC*(USER_CCU8_PWM_FREQ_HZ*(CCU8_MODULE_PRESCALER_VALUE+1))))
#define CCU4_PERIOD_REG                             ((uint32_t)(USER_PCLK_FREQ_MHz*1000)/USER_CCU4_DEBUG_KHZ)

#define CCU8_DEADTIME_RISE                          (uint32_t)((USER_DEAD_TIME_US*USER_PCLK_FREQ_MHz) - 0.5)
#define CCU8_DEADTIME_FALL                          (uint32_t)((USER_DEAD_TIME_US*USER_PCLK_FREQ_MHz) - 0.5)
#define CCU8_DEAD_TIME                              (uint32_t)(CCU8_DEADTIME_RISE + 256 * CCU8_DEADTIME_FALL)
#define ADC_TRIGGER_POINT                           (uint32_t)(SVM_TZ_PZV * 0.85f)
#define SVM_LUTTABLE_SCALE                          (uint32_t)((CCU8_PERIOD_REG / MAX_VREF_AMPLITUDE) * 32768)
#define CCU4_SECONDARY_LOOP_PERIOD   			    (uint32_t)((USER_PCLK_FREQ_MHz*1000000)/(USER_SECONDARY_LOOP_FREQ_HZ*(SECONDARY_LOOP_SLICE_PRESCALER+1)))
#define SECONDARY_LOOP_PERIOD   					CCU4_SECONDARY_LOOP_PERIOD

#if(OVERCURRENT_PROTECTION == ENABLED)
#define USER_IDC_MAXCURRENT_A                       (10.0f)
#define G_OPAMP_DC_CURRENT                          (USER_R_DCCURRENT_FEEDBACK_KOHM/ USER_RIN_DCCURRENT_KOHM)
#define IDC_MAX_LIMIT                               (uint32_t)((USER_IDC_MAXCURRENT_A * (1<<12)) / ((VAREF_V/(USER_DC_SHUNT_OHM*G_OPAMP_DC_CURRENT))/2.0f))          /* IDC Max limit to USER defined IDC Max Current */
#endif

#if(VDC_UNDER_OVERVOLTAGE_PROTECTION == ENABLED)
#define VDC_OVER_LIMIT                              ((uint16_t)(VADC_DCLINK * (100 + VDC_UNDER_OVERVOLTAGE_PERCENTAGE) / 100))                      /* VADC_DCLINK + 20%, DC link voltage Vdc maximum limit */
#define VDC_MIN_LIMIT                               ((uint16_t)(VADC_DCLINK * (100 - VDC_UNDER_OVERVOLTAGE_PERCENTAGE) / 100))                     /* VADC_DCLINK - 20%, DC link voltage Vdc min limit */
#endif
/* ********************************************* Normalization (u,v,w) represented by +2^15 *******/

#define N_VREF_SVM_V                                VREF_MAX_V
/* ********************************************* Normalization (alpha,beta) represented by +2^15 **/
#define N_I_ALPHABETA_A                            I_MAX_A
#define N_VREF_ALPHABETA_V                         VREF_MAX_V
/* ********************************************* Motor Control Timing   ***************************/
#define BOOTSTRAP_BRAKE_TIME                       ((USER_BOOTSTRAP_PRECHARGE_TIME_MS * 1000) / PWM_PERIOD_TS_US)
#define PRE_ALIGNMENT_TIME                          (uint32_t)(((USER_ROTOR_PREPOSITION_TIME_MS * 1000) / PWM_PERIOD_TS_US))
/* ********************************************* Amplitude Limit **********************************/
#define MAX_VREF_AMPLITUDE                          (32768 - 0.5f)
#define VREF_MAX_V                                  (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT)
/* ********************************************* V/F Startup Parameter  ***************************/
#define STARTUP_CURRENT_A                          (uint32_t)((USER_STARTUP_VF_OFFSET_V / USER_MOTOR_R_PER_PHASE_OHM))
#define STARTUP_SPEED                              (uint32_t)(((USER_STARTUP_SPEED_RPM * USER_MOTOR_POLE_PAIR)/(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define STARTUP_SPEED_THRESHOLD                    (uint32_t)(((USER_STARTUP_SPEED_THRESHOLD_RPM * USER_MOTOR_POLE_PAIR)/(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define STARTUP_VF_OFFSET                          (uint32_t)((USER_STARTUP_VF_OFFSET_V * 32768) / (N_VREF_SVM_V))
#define STARTUP_VF_SLEWRATE                        (uint32_t)(((USER_STARTUP_VF_SLEWRATE_V_PER_HZ * 32768) / (N_VREF_SVM_V) * (USER_MOTOR_POLE_PAIR)) / \
                                                     (USER_CCU8_PWM_FREQ_HZ*60)*65536)
/* ********************************************* POT ADC, or PWM to Adjust Speed  ****************/
#define SPEED_LOW_LIMIT_RPM                        (uint32_t)(((USER_SPEED_LOW_LIMIT_RPM * USER_MOTOR_POLE_PAIR) /(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define REFERENCE_SPEED_USER                       (uint32_t)(((USER_REFERENCE_SPEED_RPM * USER_MOTOR_POLE_PAIR) /(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define SPEED_HIGH_LIMIT_RPM                       (uint32_t)(((USER_SPEED_HIGH_LIMIT_RPM * USER_MOTOR_POLE_PAIR) /(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define RAMP_UP_SPEED                              (uint32_t)(USER_CCU8_PWM_FREQ_HZ / (((USER_SPEED_RAMPUP_RPM_PER_S * USER_MOTOR_POLE_PAIR)/(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC)))
#define RAMP_DOWN_SPEED                            (uint32_t)(USER_CCU8_PWM_FREQ_HZ / \
                                                      (((float)(USER_SPEED_RAMPDOWN_RPM_PER_S * USER_MOTOR_POLE_PAIR)/(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC)))
#define SPEED_RAMP_UPDOWN_STEP                      (1U)
#define ELECTRICAL_SPEED_FREQ_HZ                    ((float)USER_SPEED_HIGH_LIMIT_RPM/(60/USER_MOTOR_POLE_PAIR) )


/* ********************************************* PI Controller Parameters (pmsm_foc_pi.h, if used) *****************************************************************************/
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
#define USER_CURRENTCTRL_CUTOFF_FREQ_HZ             (ELECTRICAL_SPEED_FREQ_HZ * 1.0f)
#else
#define USER_CURRENTCTRL_CUTOFF_FREQ_HZ             (ELECTRICAL_SPEED_FREQ_HZ * 3)
#endif

#define CALCULATED_DEFAULT_SCALING_CURRENT_KPKI     (uint32_t)(log2f((1<<15)/((USER_CURRENTCTRL_CUTOFF_FREQ_HZ/ELECTRICAL_SPEED_FREQ_HZ)*(SPEED_HIGH_LIMIT_RPM/(1<<USER_RES_INC))*DEFAULT_L_SCALEDUP /(1<<(uint32_t)DEFAULT_SCALE_OF_L))))
#define CALCULATED_DEFAULT_IQID_KP                  (uint32_t)((USER_CURRENTCTRL_CUTOFF_FREQ_HZ/ELECTRICAL_SPEED_FREQ_HZ)*(SPEED_HIGH_LIMIT_RPM/(1<<USER_RES_INC))*DEFAULT_L_SCALEDUP /(1<<(uint32_t)(DEFAULT_SCALE_OF_L- CALCULATED_DEFAULT_SCALING_CURRENT_KPKI)))
#define CALCULATED_DEFAULT_IQID_KI                  (uint32_t)((CALCULATED_DEFAULT_IQID_KP) * USER_MOTOR_R_PER_PHASE_OHM * \
                                                       PWM_PERIOD_TS_US / USER_MOTOR_L_PER_PHASE_uH)

#define SCALEUP_MPS_K                               (uint16_t)(1<<8)
#define CORDIC_MPS_PER_K                            (USER_CORDIC_MPS / CORDIC_K) * (1 << SCALEUP_MPS_K)
/* ********************************************* Motor Parameter Scaling Conversion ***************/
#define DEFAULT_L_NO_SCALEUP                       (float)((3.0f/2.0f)*(2.0f*USER_PI*USER_CCU8_PWM_FREQ_HZ) * \
                                                     (N_I_ALPHABETA_A/N_VREF_ALPHABETA_V) *(USER_MOTOR_L_PER_PHASE_uH/1000000)/(1<<16))
//#define DEFAULT_L_SCALEDUP                         (uint32_t)((DEFAULT_L_NO_SCALEUP) * (1<<(uint32_t)DEFAULT_SCALE_OF_L))
#define DEFAULT_L_SCALEDUP                         ((DEFAULT_L_NO_SCALEUP) * (1<<(uint32_t)DEFAULT_SCALE_OF_L))
#define DEFAULT_SCALE_OF_L                         (uint32_t)(log2f((((1<<16) - 1)/(SPEED_HIGH_LIMIT_RPM/(1<< USER_RES_INC)))/(DEFAULT_L_NO_SCALEUP)))

/* ********************************************* MISC Integer Speed in SW to RPM speed of real world  **************************************************************************/
#define CONVERT_SPEED_TO_RPM                        (uint32_t)((USER_SPEED_HIGH_LIMIT_RPM * (1<<SPEED_TO_RPM_SCALE)) / \
                                                       SPEED_HIGH_LIMIT_RPM)

/*      --------------------------------------------------- pmsm_foc_parameter.h ---------------------------------------- */
/* ********************************************* Scaling SVM Modulator ****************************/
#define PWM_PERIOD_TS_US                            (1.0f/(USER_CCU8_PWM_FREQ_HZ)*1000000)
//#define SVM_LAMDA                                   (1.0f/USER_INVERSE_SVM_LAMDA)
#define SVM_LAMDA                                   (1.0f/20.0f)
#define SVM_TZ_PZV                                  (uint32_t)((CCU8_PERIOD_REG * SVM_LAMDA) + 0.5f)
#define uTZ_LAMDA_TS_US                             (SVM_LAMDA * PWM_PERIOD_TS_US)
/* ********************************************* Scaling SVM Modulator ****************************/
#define KS_SCALE_SVM                                (CCU8_PERIOD_REG / MAX_VREF_AMPLITUDE)
/* ********************************************* ADC Range  ***************************************/
#define VAREF_V                                     USER_MAX_ADC_VDD_V
#define VADC_DCLINK                                 (uint32_t)(((USER_VDC_LINK_V * USER_DC_LINK_DIVIDER_RATIO)/VAREF_V) * (1<<12))
#define VADC_DCLINK_SCALE							(VAREF_V/((1<<12)*USER_DC_LINK_DIVIDER_RATIO))
#define VDC_MAX_LIMIT                               ((uint16_t)((VADC_DCLINK * 19U)>>4U))                                         /* Vdc_ideal + 18.7%, DC link voltage Vdc maximum limit, only for braking usage, voltage clamping */
















/* ********************************************* uC Probe Parameters  ***********************************************************/
/*********************************************************************************************************************
 * MACROS
 ***************************************/
/* Timing parameters */
#define PERIOD_REG                    CCU8_PERIOD_REG
#define TZ_PZV                        SVM_TZ_PZV
#define BRAKE_TIME                    BOOTSTRAP_BRAKE_TIME
#define ALIGNMENT_TIME                PRE_ALIGNMENT_TIME

/* Scale of SVM sine Look-Up Table (LUT) */
#define SVM_LUT_SCALE                 SVM_LUTTABLE_SCALE

/* Motor parameters for ωL|I|, Vref/L in MET and PLL Observer */
#define L_OMEGALI                     DEFAULT_L_SCALEDUP
#define SCALE_L                       DEFAULT_SCALE_OF_L

/* V/f parameter */

#define VQ_VF_OFFSET                  STARTUP_VF_OFFSET
#define VQ_VF_SLEW                    STARTUP_VF_SLEWRATE
#define DEFAULT_SPEED_STARTUP         STARTUP_SPEED
#define VF_TRANSITION_SPEED           STARTUP_SPEED_THRESHOLD
#define DEFAULT_SPEED_REFERENCE       REFERENCE_SPEED_USER

#define RAMPUP_RATE                   RAMP_UP_SPEED
#define RAMPDOWN_RATE                 RAMP_DOWN_SPEED

/* Motor speed limit */
#define SPEED_LOW_LIMIT               SPEED_LOW_LIMIT_RPM
#define SPEED_HIGH_LIMIT              SPEED_HIGH_LIMIT_RPM
#define TZ_PZVX2                      (SVM_TZ_PZV<<1)
#define HALF_TZ_PZV                   (SVM_TZ_PZV>>1)

/* For SW debug */
#define CCU4_PWM_PERIOD               CCU4_PERIOD_REG

/* CCU8 dead time */
#define DEAD_TIME                     CCU8_DEAD_TIME

#define SPEEDRAMPSTEP                 SPEED_RAMP_UPDOWN_STEP

/* For MET Fine-Tuning */
#define THRESHOLD_HIGH                USER_MET_THRESHOLD_HIGH
#define THRESHOLD_LOW                 USER_MET_THRESHOLD_LOW
#define SHIFT_MET_PLL                 USER_MET_LPF

/* ADC trigger point of Pseudo Zero Vectors */
#define TRIGGER_POINT                 ADC_TRIGGER_POINT

/* SVM voltage compensation */
#define ADC_DCLINK_IDEAL              VADC_DCLINK
/* Angle/speed resolution increase, especially for low-speed motor drive. */
/* Parameters for Startup Lock / Stall Detection */
#define MAX_RETRY_START_STALL         USER_MAX_RETRY_MOTORSTARTUP_STALL

/* Increase Angle (and Speed) Resolution */
#define RES_INC                       USER_RES_INC

/* Integer speed in SW to rpm speed of real world (for debug) */
#define SPEED_TO_RPM                  CONVERT_SPEED_TO_RPM
#define RPM_TO_SPEED					1/CONVERT_SPEED_TO_RPM
#define SCALE_SPEED_TO_RPM            SPEED_TO_RPM_SCALE

/* Use UART to set POT ADC, and hence target motor speed. */
#define ADC_FOR_STEP_50_RPM         ADC_STEP_INC_EACHRPM

/* Direct FOC startup. Motor startup to FOC closed-loop directly, no V/f or MET.*/
#if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC)
/* Initial ramp up rate in FOC transition mode.*/
  #define RAMP_RATE_INIT_FOC    (RAMPUP_RATE << 0U)
#else
  #define RAMP_RATE_INIT_FOC    (RAMPUP_RATE << 0U)
#endif
/* (SPEED_LOW_LIMIT), (DEFAULT_SPEED_REFERENCE). Minimum startup speed for FOC. */
#define MIN_STARTUP_SPEED_FOC   (SPEED_LOW_LIMIT << 1)
/* Threshold speed to exit FOC. */
#define FOC_EXIT_SPEED  (SPEED_LOW_LIMIT * 3)


#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_VARIABLES_SCALING_H_ */
