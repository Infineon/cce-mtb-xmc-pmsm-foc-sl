/**
 * @pmsm_foc_EVAL_M1_1402.h
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

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_MCUCARD_PARAMETERS_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_MCUCARD_PARAMETERS_H_

#include "../pmsm_foc_user_config.h"
#include <xmc_vadc.h>
#include <xmc_ccu4.h>
#include <xmc_ccu8.h>
#include <xmc_scu.h>
#include <xmc_gpio.h>
#include <xmc_math.h>
#include <xmc_wdt.h>
#include "xmc1_gpio_map.h"



#if(MCUCARD_TYPE == EVAL_M1_1402)
/* ********************************************* EVAL_M1_1402 *************************/
/*********************************************************************************************************************
 * XMC1400 Controller board for MADK platform
 * GPIO Resources Configuration
 ***************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_7
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_LS_PIN         P0_6
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_HS_OUTPUT0_SOURCE XMC_CCU8_SOURCE_OUT0_INV_ST1
#define PHASE_U_LS_OUTPUT1_SOURCE XMC_CCU8_SOURCE_OUT1_ST1
#define PHASE_U_HS_OUTPUT2_SOURCE DISABLED
#define PHASE_U_LS_OUTPUT3_SOURCE DISABLED

#define PHASE_V_HS_OUTPUT0_SOURCE XMC_CCU8_SOURCE_OUT0_INV_ST1
#define PHASE_V_LS_OUTPUT1_SOURCE XMC_CCU8_SOURCE_OUT1_ST1
#define PHASE_V_HS_OUTPUT2_SOURCE DISABLED
#define PHASE_V_LS_OUTPUT3_SOURCE DISABLED

#define PHASE_W_HS_OUTPUT0_SOURCE XMC_CCU8_SOURCE_OUT0_INV_ST1
#define PHASE_W_LS_OUTPUT1_SOURCE XMC_CCU8_SOURCE_OUT1_ST1
#define PHASE_W_HS_OUTPUT2_SOURCE DISABLED
#define PHASE_W_LS_OUTPUT3_SOURCE DISABLED

#define TEST_PIN           P0_5

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ***************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83
#define CCU8_MODULE_PRESCALER_VALUE         (0U)

/*********************************************************************************************************************
 * Secondary loop define
 ****************************************/
#define SECONDARY_LOOP_MODULE        CCU40
#define SECONDARY_LOOP_SLICE        CCU40_CC42
#define SECONDARY_LOOP_SLICE_PRESCALER     (2U)
#define SECONDARY_LOOP_SLICE_NUM      (2U)
#define SECONDARY_LOOP_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_2

/* TRAP LED blinking period. */
#define LED_BLINK_PRS     (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)

/*********************************************************************************************************************
 * VADC Resources Configuration
 ***************************************/

/* ADC is configured as three shunt in SYNC*/
#if (CURRENT_SENSING ==  USER_THREE_SHUNT_SYNC_CONV)
/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_G0_CHANNEL    (2U)        /* P2.9, VADC group0 channel 2 */

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_G0_CHANNEL    (4U)       /* P2.11, VADC group0 channel 4 */



/* ADC is configured as three shunt in ASSYNC*/
#elif (CURRENT_SENSING ==  USER_THREE_SHUNT_ASSYNC_CONV)
/* Motor Phase U VADC define */
#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_RESULT_REG    (4U)

/* Motor Phase V VADC define */
#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_RESULT_REG    (3U)

/* ADC is configured as single shunt */
#elif(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (1U)       /* P2.7 VADC group1 channel 1 */
#define VADC_ISS_RESULT_REG   (15U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (5U)      /* P2.3 VADC group1 channel 5 */
#define VADC_VDC_RESULT_REG   (5U)

/* DC link average current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (6U)       /* P2.1 VADC group0 channel 6 */
#define VADC_IDC_RESULT_REG   (6U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)


/*********************************************************************************************************************
 * UART Resources Configuration
 ***************************************/
#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3              /* 1. USIC_DISABLED_ALL, -- which enable POT ADC speed adjustment
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */
#endif

/* NVIC Interrupt Resources Configuration */
/* ***************************************/


#if(UC_SERIES == XMC14)
	#define pmsm_foc_controlloop_isr            IRQ25_Handler
	#define pmsm_foc_trap_protection_irq        IRQ26_Handler
	#define pmsm_foc_secondaryloop_isr			IRQ21_Handler
    #define pmsm_foc_over_under_voltage_isr          IRQ19_Handler
	#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
	  #if (VADC_ISS_GROUP_NO == 0)
		#define pmsm_foc_vadc_source_irqhandler              IRQ18_Handler
	  #else
		#define pmsm_foc_vadc_source_irqhandler              IRQ20_Handler
	  #endif
	#endif
#else
	#define pmsm_foc_controlloop_isr            CCU80_0_IRQHandler
	#define pmsm_foc_trap_protection_irq        CCU80_1_IRQHandler
	#define pmsm_foc_secondaryloop_isr          CCU40_0_IRQHandler
    #define pmsm_foc_over_under_voltage_isr     VADC0_G1_0_IRQHandler
	#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
	  #if (VADC_ISS_GROUP_NO == 0)
		#define pmsm_foc_vadc_source_irqhandler              VADC0_G0_1_IRQHandler
	  #else
		#define pmsm_foc_vadc_source_irqhandler              VADC0_G1_1_IRQHandler
	  #endif
	#endif
#endif



/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ***************************************/
#define DEBUG_PWM_0_ENABLE      (0U)        /* 1 = Enable Debug PWM P1.0, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE      (1U)        /* 1 = Enable Debug PWM P0.4, 0 = Disable Debug PWM */

#if(DEBUG_PWM_0_ENABLE == 1U || DEBUG_PWM_1_ENABLE == 1U)

  #define DEBUG_PWM_CCU4_MODULE   CCU40
  #define DEBUG_PWM_PERIOD_CNTS (400U)
  #define DEBUG_PWM_50_PERCENT_DC_CNTS  ((uint16_t)(DEBUG_PWM_PERIOD_CNTS >> 1))
  #define REVERSE_CRS_OR_0  (- Tmp_CRS) /* Tmp_CRS = 0 or (- Tmp_CRS) if Tmp_CRS < 0. */

  #if (DEBUG_PWM_0_ENABLE == 1U)
    #define DEBUG_PWM_0_SLICE                         CCU40_CC40
    #define DEBUG_PWM_0_SLICE_NUM                     (0U)
    #define DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_0
    #define DEBUG_PWM_0_PORT                          XMC_GPIO_PORT1
    #define DEBUG_PWM_0_PIN                           (0U)
    #define DEBUG_PWM_0_ALT_OUT                       XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2
  #endif
  #if (DEBUG_PWM_1_ENABLE == 1U)
    #define DEBUG_PWM_1_SLICE                         CCU40_CC41
    #define DEBUG_PWM_1_SLICE_NUM                     (1U)
    #define DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_1
    #define DEBUG_PWM_1_PORT                          XMC_GPIO_PORT0
    #define DEBUG_PWM_1_PIN                           (4U)
    #define DEBUG_PWM_1_ALT_OUT                       XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT4
  #endif
#endif






#endif /*(MCUCARD_TYPE == EVAL_M1_1402)*/
#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_MCUCARD_PARAMETERS_H_ */
