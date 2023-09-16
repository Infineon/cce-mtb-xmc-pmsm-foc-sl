/**
 *
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
 */

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_CONST_MARCOS_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_CONST_MARCOS_H_

/**
 * @addtogroup
 * @{
 */

/**
 * @addtogroup
 * @{
 */
#include <xmc_common.h>

/*******************************************************************************
 * MACROS
 **/

/***********************************************************************************************************************
 * Device Related
 *****************************************/
/* only for XCM1300 */
#define   AA_STEP                                   (1U)
#define   AB_STEP                                   (2U)

/* Hardware kit. Refer to the header file description above */
#define   KIT_XMC1X_AK_MOTOR_001                    (1U)
#define   KIT_XMC750WATT_MC_AK_V1                   (2U)
#define   CUSTOM_KIT                                (8U)
//#define   IFCN_XMC_LVPB_R2                          (3U)
#define   KIT_XMC_IFI_24V_250W                         (4U)
//#define   IFCN_XMC_LVPB_R3                          (5U)
//#define   IFX_XMC_PINUS_V2                          (6U)
#define   IFX_MADK_EVAL_M1_05_65D_V1                (7U)
#define   IFX_MADK_EVAL_M1_05F310	                (9U)
#define   IFX_MADK_EVAL_M1_CM610N3	       			 (10U)

/* MCU Card. Refer to the header file description above */
#define   KIT_XMC13_BOOT_001                        (1U)
#define   KIT_XMC1300_DC_V1                         (2U)
//#define   MINITKIT_XMC1300_V1                       (3U)
//#define   XMC_PINUS_MCU_V2                          (4U)
#define   EVAL_M1_1302                              (5U)
#define   CUSTOM_MCU                                (6U)
#define   KIT_XMC14_BOOT_001                        (7U)
#define   KIT_XMC1400_DC_V1  	                    (8U)
#define   EVAL_M1_1402		                        (9U)

/* Inverter Card. Refer to the header file description above */
#define   PMSM_LV15W                                (1U)
#define   POWERINVERTER_750W                        (2U)
//#define   IFX_XMC_LVPB_R3                           (3U)
//#define   IFX_XMC_LVPB_R2                           (4U)
#define   KIT_MOTOR_DC_250W_24V                             (5U)
//#define   XMC_PINUS_INVERTER_V2                     (6U)
#define   EVAL_M1_05_65A                            (7U)
#define   CUSTOM_INVERTER                           (8U)
#define   EVAL_M1_05F310                            (9U)
#define   EVAL_M1_CM610N3 							(10U)

/* Motor Type. Refer to the header file description above  */
#define  VORNADO_FAN_MOTOR                          (3U)
#define  NANOTEC_MOTOR_DB42S03                              (4U)
#define  MAXON_MOTOR_267121                                (5U)
#define  EBM_PAPST_VENTI_FAN_MOTOR                  (7U)
#define  CUSTOM_MOTOR                               (8U)

#define  STANDARD_SVM_7_SEGMENT                     (1U)
#define  STANDARD_SVM_5_SEGMENT                     (2U)

/* Current Sensing Feedback Scheme */
#define  USER_THREE_SHUNT_ASSYNC_CONV                (1U)
#define  USER_THREE_SHUNT_SYNC_CONV                  (2U)
#define  USER_SINGLE_SHUNT_CONV                      (3U)
/*      --------------------------------------------------- FOC Control and Startup Scheme  ---------------------------------------- */
#define  SPEED_CONTROLLED_VF_ONLY                   (1U)
#define  SPEED_CONTROLLED_VF_MET_FOC                (2U)
#define  SPEED_CONTROLLED_DIRECT_FOC                (3U)
#define  TORQUE_CONTROLLED_DIRECT_FOC               (4U)
#define  VQ_CONTROLLED_DIRECT_FOC                   (5U)
/*      -----------------------------Over current / Over Voltage Protection Scheme  -------------------------------- */
#define ENABLED                                      (1U)
#define DISABLED                                     (0U)
/*      -----------------------------Gate driver Input logic definition  -------------------------------- */
#define PASSIVE_HIGH                                 (0U)
#define PASSIVE_LOW                                  (1U)
/*      -----------------------------UART USIC Channel Configuration  -------------------------------- */
#define USIC_DISABLED_ALL                            (0U)
#define USIC0_CH0_P1_4_P1_5                          (1U)
#define USIC0_CH1_P1_2_P1_3                          (2U)

#define FALSE                                        (0U)
#define TRUE                                         (1U)
/*      ----------------------------- Reference Speed Adjustment Method  -------------------------------- */
#define SET_TARGET_SPEED                              (1U)
#define BY_POT_ONLY                                  (2U)
#define BY_UART_ONLY                                 (3U)                           /* Baudrate: 460800, Data: 8-bit, Parity: None, Stop: 1-bit*/


//#define MOTOR_TRANSITION                            0                       /* Motor is in transition mode */
//#define MOTOR_STABLE                                0xAB                    /* Motor is in stable mode */
#define DIRECTION_INC                               0                       /* Motor rotation direction - rotor angle increasing */
#define ADJUST_DONE                                 0                       /* Parameter adjustment has been done */

#define ACTIVE_HIGH                                 0
#define ACTIVE_LOW                                  1

#define CFR_PHASE_U                                 0
#define CFR_PHASE_V                                 1
#define CFR_PHASE_W                                 2




/*      --------------------------------------------------- MET Fine-tuning ---------------------------------------- */
#define USER_MET_THRESHOLD_HIGH                     (64U)
#define USER_MET_THRESHOLD_LOW                      (16U)
#define USER_MET_LPF                                (2U)
/* *********************************************  Increase Angle (and Speed Resolution) ****************/
#define USER_RES_INC                                (3U)

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_CONST_MARCOS_H_ */

/**
 * @}
 */

/**
 * @}
 */
