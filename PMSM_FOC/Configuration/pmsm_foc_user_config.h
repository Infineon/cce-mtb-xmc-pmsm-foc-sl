/**
 * @file pmsm_foc_user_config.h
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

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_CONFIG_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_CONFIG_H_

/**********************************************************************************************************************
 * HEADER FILES
 ****************************************/
#include "pmsm_foc_macro.h"
#include "math.h"

/**
 * @addtogroup
 * @{
 */

/*********************************************************************************************************************
 * MACROS
 ***************************************/
#define  PMSM_FOC_HARDWARE_KIT                    KIT_XMC1X_AK_MOTOR_001                 /* 1. KIT_XMC1X_AK_MOTOR_001
																						2. KIT_XMC750WATT_MC_AK_V1
   																						3. CUSTOM_KIT
																						4. KIT_XMC_IFI_24V_250W
																						5. IFX_MADK_EVAL_M1_05F310
																						6. IFX_MADK_EVAL_M1_05_65D_V1
																						7* IFX_MADK_EVAL_M1_CM610N3*/

/*      --------------------------------------------------- Current feedback Sensing Mechanism ---------------------------------------- */
#define  CURRENT_SENSING                            USER_THREE_SHUNT_SYNC_CONV             /*1. USER_SINGLE_SHUNT_CONV
                                                                                              2. USER_THREE_SHUNT_ASSYNC_CONV
                                                                                              3. USER_THREE_SHUNT_SYNC_CONV*/
/*      --------------------------------------------------- FOC Control and Startup Scheme (Only Select 1 Scheme at one time) ---------------------------------------- */
#define MY_FOC_CONTROL_SCHEME                       SPEED_CONTROLLED_DIRECT_FOC              /*1. SPEED_CONTROLLED_VF_ONLY,
                                                                                             2. SPEED_CONTROLLED_VF_MET_FOC
                                                                                             3. SPEED_CONTROLLED_DIRECT_FOC
                                                                                             4. TORQUE_CONTROLLED_DIRECT_FOC
                                                                                             5. VQ_CONTROLLED_DIRECT_FOC */
/*      --------------------------------------------------- Micrium uC Probe Enable/Disable  ---------------------------------------- */
#define uCPROBE_GUI_OSCILLOSCOPE										ENABLED							 /*1. ENABLED       2. DISABLED (Disable Probe Sampling in PWM irq )*/

/*      --------------------------------------------------- Reference Speed Adjustment Method ---------------------------------------- */
#define SETTING_TARGET_SPEED                        SET_TARGET_SPEED                                     /*1. SET_TARGET_SPEED
                                                                                                     2. BY_POT_ONLY
                                                                                                     3. BY_UART_ONLY */

/*      --------------------------------------------------- Constant Torque Control Mode (Used when Constant Torque Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- IQ_REF-limit low < IQ_REF < IQ_REF-limit high  ---------------------------------------- */
#if(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
#define USER_IQ_CURRENT_ALLOWED_A                   (2.0f)                                                                          /* 0 < USER_IQ_CURRENT_ALLOWED_A < I_MAX_A*/
#define USER_IQ_REF_LOW_LIMIT                       (0U)
#define USER_IQ_REF_HIGH_LIMIT                      (uint32_t) (32768 * USER_IQ_CURRENT_ALLOWED_A /I_MAX_A)                          /*  I_MAX_A = (VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2.0f), IFX_XMC_LVPB_R3 - 13.6A */
#define USER_IQ_RAMPUP                              (10U)
#define USER_IQ_RAMPDOWN                            (10U)
#define USER_IQ_RAMP_SLEWRATE                       (50U)                                                                           /* USER_IQ_RAMP_SLEWRATE x PWM period, every cycle increase USER_IQ_RAMPUP or USER_IQ_RAMPDOWN */
#endif
/*      --------------------------------------------------- Constant VQ Control Mode (Used when Constant VQ Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- VQ_REF-limit low < VQ_REF < VQ_REF-limit high  ---------------------------------------- */
#if(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
#define USER_VQ_VOLTAGE_ALLOWED_V                   (10U)                                                                            /* 0 < USER_VQ_VOLTAGE_ALLOWED_V < VREF_MAX_V          VREF_MAX_V =  (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT)*/
#define USER_VQ_REF_LOW_LIMIT                       (0U)
#define USER_VQ_REF_HIGH_LIMIT                      (uint32_t)(32768U * USER_VQ_VOLTAGE_ALLOWED_V/VREF_MAX_V)
#define USER_VQ_RAMPUP                              (2U)
#define USER_VQ_RAMPDOWN                            (2U)
#define USER_VQ_RAMP_SLEWRATE                       (10U)         /* USER_VQ_RAMP_SLEWRATE x PWM period, every cycle increase USER_VQ_RAMPUP or USER_VQ_RAMPDOWN */
#endif

/*      --------------------------------------------------- SVM Switching Sequences ---------------------------------------- */
#define SVM_SWITCHING_SCHEME                        STANDARD_SVM_7_SEGMENT                        /*1. STANDARD_SVM_7_SEGMENT          2. STANDARD_SVM_5_SEGMENT     */
/*      --------------------------------------------------- ADC_STARTUP_CALIBRATION only for XMC1300 AA step ---------------------------------------- */
#define ADC_STARTUP_CALIBRATION                     DISABLED                                       /*1. ENABLED       2. DISABLED*/

#define ADC_ALTERNATE_REFERENCE                     DISABLED                                      /*1. ENABLED       2. DISABLED*/

/*      --------------------------------------------------- Add d-q voltage decoupling components ---------------------------------------- */
#define DQ_DECOUPLING                               ENABLED                                         /*1. ENABLED       2. DISABLED*/

#define WATCH_DOG_TIMER                             ENABLED                                         /*1. ENABLED       2. DISABLED*/
/*      --------------------------------------------------- FOC Control Safety Protection ---------------------------------------- */
#define VDC_UNDER_OVERVOLTAGE_PROTECTION            DISABLED                                         /*1. ENABLED       2. DISABLED*/
#define VDC_UNDER_OVERVOLTAGE_PERCENTAGE			(20U)
#define OVERCURRENT_PROTECTION                      DISABLED                                         /*1. ENABLED       2. DISABLED*/ /* Not possible with MADK HW */

/*
 * Hardware kit.
 *
 * KIT_XMC1X_AK_MOTOR_001
 * KIT_XMC750WATT_MC_AK_V1
 * KIT_XMC14_BOOT_001
 * KIT_MOTOR_DC_250W_24V
 * IFX_MADK_EVAL_M1_05F310 Kit
 * IFX_MADK_EVAL_M1_05_65D_V1
 * IFX_MADK_EVAL_M1_CM610N3
 * CUSTOM_KIT
*/

/*
 *  MCU Card.
 *
 *  KIT_XMC13_BOOT_001
 *  KIT_XMC1300_DC_V1
 *  EVAL_M1_1302
 *  EVAL_M1_1402
 *  KIT_XMC14_BOOT_001
 *  KIT_XMC1400_DC_V1
 *  CUSTOM_MCU
*/

/*
 *  Inverter Card
 *
 *	EVAL_M1_05_65A
 *	EVAL_M1_05F310
 *	EVAL_M1_CM610N3
 *	KIT_MOTOR_DC_250W_24V
 *	PMSM_LV15W
 *	POWERINVERTER_750W
 *	CUSTOM_INVERTER
 *
*/

/*
 * Motor Type
 *
 * MAXON_MOTOR_267121
 * CUSTOM_MOTOR
 * NANOTEC_MOTOR_DB42S03
*/


#if(PMSM_FOC_HARDWARE_KIT == KIT_XMC1X_AK_MOTOR_001)
#if (UC_SERIES == XMC13)
#define MCUCARD_TYPE                                      KIT_XMC13_BOOT_001
#define MCUCARD_TYPE_PATH                                 "Controller_Card/pmsm_foc_KIT_XMC13_BOOT_001.h"
#elif (UC_SERIES == XMC14)
#define MCUCARD_TYPE                                      KIT_XMC14_BOOT_001
#define MCUCARD_TYPE_PATH                                 "Controller_Card/pmsm_foc_KIT_XMC14_BOOT_001.h"
#endif
#define INVERTERCARD_TYPE                                 PMSM_LV15W
#define INVERTERCARD_TYPE_PATH                            "Inverter_Card/pmsm_foc_PMSM_LV15W.h"
#define MOTOR_TYPE                                        MAXON_MOTOR_267121
#define MOTOR_TYPE_PATH                                   "Motors/pmsm_foc_motor_MAXON_MOTOR_267121.h"
#elif(PMSM_FOC_HARDWARE_KIT == KIT_XMC750WATT_MC_AK_V1)
#define MCUCARD_TYPE                                      KIT_XMC1300_DC_V1
#define MCUCARD_TYPE_PATH                                 "Controller_Card/pmsm_foc_KIT_XMC1300_DC_V1.h"
#define INVERTERCARD_TYPE                                 POWERINVERTER_750W
#define INVERTERCARD_TYPE_PATH                            "Inverter_Card/pmsm_foc_POWERINVERTER_750W.h"
#define MOTOR_TYPE                                        CUSTOM_MOTOR
#define MOTOR_TYPE_PATH                                   "Motors/pmsm_foc_motor_CUSTOM_MOTOR.h"
#elif(PMSM_FOC_HARDWARE_KIT == KIT_XMC_IFI_24V_250W)
#define MCUCARD_TYPE                                      KIT_XMC1300_DC_V1
#define MCUCARD_TYPE_PATH                                 "Controller_Card/pmsm_foc_KIT_XMC1300_DC_V1.h"
#define INVERTERCARD_TYPE                                 KIT_MOTOR_DC_250W_24V
#define INVERTERCARD_TYPE_PATH                            "Inverter_Card/pmsm_foc_KIT_MOTOR_DC_250W_24V.h"
#define MOTOR_TYPE                                        NANOTEC_MOTOR_DB42S03
#define MOTOR_TYPE_PATH                                   "Motors/pmsm_foc_motor_NANOTEC_MOTOR_DB42S03.h"
#elif(PMSM_FOC_HARDWARE_KIT == IFX_MADK_EVAL_M1_05F310)
#define MCUCARD_TYPE                                      EVAL_M1_1302
#define MCUCARD_TYPE_PATH                                 "Controller_Card/pmsm_foc_EVAL_M1_1302.h"
#define INVERTERCARD_TYPE                                 EVAL_M1_05F310
#define INVERTERCARD_TYPE_PATH                           "Inverter_Card/pmsm_foc_EVAL_M1_05F310.h"
#define MOTOR_TYPE                                        MAXON_MOTOR_267121
#define MOTOR_TYPE_PATH                                   "Motors/pmsm_foc_motor_MAXON_MOTOR_267121.h"
#elif(PMSM_FOC_HARDWARE_KIT == IFX_MADK_EVAL_M1_05_65D_V1)
#define MCUCARD_TYPE                                      EVAL_M1_1302
#define MCUCARD_TYPE_PATH                                 "Controller_Card/pmsm_foc_EVAL_M1_1302.h"
#define INVERTERCARD_TYPE                                 EVAL_M1_05_65A
#define INVERTERCARD_TYPE_PATH                           "Inverter_Card/pmsm_foc_EVAL_M1_05_65A.h"
#define MOTOR_TYPE                                        CUSTOM_MOTOR
#define MOTOR_TYPE_PATH                                   "Motors/pmsm_foc_motor_CUSTOM_MOTOR.h"
#elif(PMSM_FOC_HARDWARE_KIT == IFX_MADK_EVAL_M1_CM610N3)
#define MCUCARD_TYPE                                      EVAL_M1_1302
#define MCUCARD_TYPE_PATH                                 "Controller_Card/pmsm_foc_EVAL_M1_1302.h"
#define INVERTERCARD_TYPE                                 EVAL_M1_CM610N3
#define INVERTERCARD_TYPE_PATH                           "Inverter_Card/pmsm_foc_EVAL_M1_CM610N3.h"
#define MOTOR_TYPE                                        CUSTOM_MOTOR
#define MOTOR_TYPE_PATH                                   "Motors/pmsm_foc_motor_CUSTOM_MOTOR.h"
#elif(PMSM_FOC_HARDWARE_KIT == CUSTOM_KIT)
#define MCUCARD_TYPE                                      CUSTOM_MCU
#define MCUCARD_TYPE_PATH                                 "Controller_Card/pmsm_foc_CUSTOM_MCU.h"
#define INVERTERCARD_TYPE                                 CUSTOM_INVERTER
#define INVERTERCARD_TYPE_PATH                            "Inverter_Card/pmsm_foc_CUSTOM_INVERTER.h"
#define MOTOR_TYPE                                        CUSTOM_MOTOR
#define MOTOR_TYPE_PATH                                   "Motors/pmsm_foc_motor_CUSTOM_MOTOR.h"
#endif

#define USER_SECONDARY_LOOP_FREQ_HZ         (1000U)         /*Secondary Loop Freq for non-real time operation*/

#if(ADC_ALTERNATE_REFERENCE == ENABLED)
  #if((CURRENT_SENSING == USER_THREE_SHUNT_ASSYNC_CONV) || (CURRENT_SENSING == USER_THREE_SHUNT_SYNC_CONV))
      #define ADC_ALTERNATE_REF_PHASEUVW            ENABLED
      #define ADC_ALTERNATE_REF_SINGLESHUNT         DISABLED
  #elif(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      #define ADC_ALTERNATE_REF_PHASEUVW            DISABLED
      #define ADC_ALTERNATE_REF_SINGLESHUNT         ENABLED
  #endif
#elif(ADC_ALTERNATE_REFERENCE == DISABLED)
      #define ADC_ALTERNATE_REF_PHASEUVW            DISABLED
      #define ADC_ALTERNATE_REF_SINGLESHUNT         DISABLED
#endif

#define CATCH_FREE_RUNNING                             DISABLED                                 /*1. ENABLED       2. DISABLED*/


#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_CONFIG_H_ */
/**
 * @}
 */

/**
 * @}
 */
