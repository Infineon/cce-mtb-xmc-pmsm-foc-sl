/**
 * @pmsm_foc_EVAL_M1_05F310.h
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

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_INVERTERCARD_PARAMETERS_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_INVERTERCARD_PARAMETERS_H_

#include "../pmsm_foc_user_config.h"



#if(INVERTERCARD_TYPE == EVAL_M1_CM610N3)
/********************************************** EVAL_M1_CM610N3 ******MADK inverter Vin<70V max 300W *******************/
//NOTE this parameters are different respect default values
#define INTERNAL_OP_GAIN                            ENABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (320.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_CCU8_PWM_FREQ_HZ                       (20000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_DEAD_TIME_US                           (0.75f)                 /* deadtime, rise(left) and fall values in us  */

#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (20U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(4.7f/(4.7f+1000.0f+1000.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */

/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.1f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.1f)               /* DC link shunt current resistor in ohm */

#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             	(3U)                       /* Different HW Board has different OP Gain factor, XMC14 built-in Gain Factor available 1, 3, 6 and 12 only*/
#define USER_RIN_OFFSET_KOHM						(2.0f)						/* this reistence is on Control Card to generate ADC input offset */
#define USER_RIN_PULL_UP_KOHM						(10.0f)						/* this reistence is on Control Card to generate ADC input offset */
#define CIRCUIT_GAIN_FACTOR							(OP_GAIN_FACTOR * (USER_RIN_PULL_UP_KOHM / (USER_RIN_OFFSET_KOHM + USER_RIN_PULL_UP_KOHM)))

#elif(INTERNAL_OP_GAIN == DISABLED)
#define USER_RIN_PHASECURRENT_KOHM                  (1.0f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (12.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (1.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (12.0f)                   /* Rin for dc current sensing */
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define CIRCUIT_GAIN_FACTOR                         G_OPAMP_PER_PHASECURRENT
#endif

#define I_MAX_A                                    ((VAREF_V/(USER_R_SHUNT_OHM * CIRCUIT_GAIN_FACTOR))/ 2U)

#define GATE_DRIVER_INPUT_LOGIC                    PASSIVE_LOW                   /*1. PASSIVE_HIGH       2. PASSIVE_LOW (Please refer the gate driver datasheet) */
#if(GATE_DRIVER_INPUT_LOGIC == PASSIVE_HIGH)
#define MOTOR_COASTING_HIGH_SIDE                    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define MOTOR_COASTING_LOW_SIDE                     XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define MOTOR_RUN_HIGH_SIDE                         XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define MOTOR_RUN_LOW_SIDE                          XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#elif(GATE_DRIVER_INPUT_LOGIC == PASSIVE_LOW)
#define MOTOR_COASTING_HIGH_SIDE                    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#define MOTOR_COASTING_LOW_SIDE                     XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH

#define MOTOR_RUN_HIGH_SIDE                         XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#define MOTOR_RUN_LOW_SIDE                          XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW
#endif

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (0U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

#endif

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_INVERTERCARD_PARAMETERS_H_ */

