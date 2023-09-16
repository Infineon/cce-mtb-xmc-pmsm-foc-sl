/**
 * @file ccu4.c
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
 * @file ccu4.c
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

/*********************************************************************************************************************
 * HEADER FILES
 ***************************************/
#include "ccu4.h"
#include <xmc_common.h>

/*********************************************************************************************************************
 * DATA STRUCTURES
 ***************************************/

#if (( DEBUG_PWM_0_ENABLE) | (DEBUG_PWM_1_ENABLE))
/**
 *  Data Structure initialization - CCU4 Slice Configuration.
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t DebugPwmSlice_Cfg =
{
  .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear = 0U,
  .dither_timer_period = 0U,
  .dither_duty_cycle = 0U,
  .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable = 0U,
  .prescaler_initval = 0U, /* divide by 1--> 64MHz (as fast as possible) */
  .float_limit = 0U,
  .dither_limit = 0U,
  .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
  .timer_concatenation = false
};
#endif /* ((DEBUG_PWM_0_ENABLE) | (DEBUG_PWM_1_ENABLE)) */

XMC_CCU4_SLICE_COMPARE_CONFIG_t  Secondary_loop_Cfg  =
{
  .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear = 0U,
  .dither_timer_period = 0U,
  .dither_duty_cycle = 0U,
  .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable = 0U,
  .prescaler_initval = SECONDARY_LOOP_SLICE_PRESCALER,
  .float_limit = 0U,
  .dither_limit = 0U,
  .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
  .timer_concatenation = false
};

/**
 *  Data Structure initialization - CC8 Slice event 0 set to sync start.
 */
XMC_CCU4_SLICE_EVENT_CONFIG_t CCU4_Input_event0_config =
{

#if(UC_SERIES == XMC14)
  .mapped_input = XMC_CCU4_SLICE_INPUT_AI,
#else
  .mapped_input = XMC_CCU4_SLICE_INPUT_I,
#endif
  .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE
};

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ***************************************/

/* API to initialize CCU4 module to outputs debug information. Outputs P1. 0, P0.4, P1.2, P1.3 */
void pmsm_foc_ccu4_init(void)
{

  /* Init CCU40 */
  XMC_CCU4_SetModuleClock(SECONDARY_LOOP_MODULE, XMC_CCU4_CLOCK_SCU);
  XMC_CCU4_Init(SECONDARY_LOOP_MODULE, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
  /* Get the slice out of idle mode */
  XMC_CCU4_EnableClock(SECONDARY_LOOP_MODULE, SECONDARY_LOOP_SLICE_NUM);

  XMC_CCU4_StartPrescaler(SECONDARY_LOOP_MODULE);

  /* Initialize the Slice */
  XMC_CCU4_SLICE_CompareInit(SECONDARY_LOOP_SLICE, &Secondary_loop_Cfg);



  /* Enable compare match and period match events */
  XMC_CCU4_SLICE_EnableEvent(SECONDARY_LOOP_SLICE, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
  /* Connect period match event to SR0 */
  XMC_CCU4_SLICE_SetInterruptNode(SECONDARY_LOOP_SLICE, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_0);

  XMC_CCU4_SLICE_ConfigureEvent(SECONDARY_LOOP_SLICE, XMC_CCU4_SLICE_EVENT_0, &CCU4_Input_event0_config);

  #if(UC_SERIES == XMC14)
  XMC_SCU_SetInterruptControl(IRQ21_IRQn, XMC_SCU_IRQCTRL_CCU40_SR0_IRQ21);
	NVIC_SetPriority (IRQ21_IRQn, 3U);
	NVIC_EnableIRQ(IRQ21_IRQn);
#else
	NVIC_SetPriority (CCU40_0_IRQn, 3U);
	NVIC_EnableIRQ(CCU40_0_IRQn);
#endif

  XMC_CCU4_SLICE_SetTimerPeriodMatch(SECONDARY_LOOP_SLICE, SECONDARY_LOOP_PERIOD);
 // XMC_CCU4_SLICE_SetTimerCompareMatch(SECONDARY_LOOP_SLICE, 23437U);
  XMC_CCU4_EnableShadowTransfer(SECONDARY_LOOP_MODULE, (uint32_t) SECONDARY_LOOP_SLICE_SHADOW_TRANS_ENABLE_Msk);


//  XMC_CCU4_SLICE_StartTimer(SECONDARY_LOOP_SLICE);



  #if	(DEBUG_PWM_0_ENABLE == 1U)
  /* Init Debug PWM Slice */
  /* Get the slice out of idle mode */
  XMC_CCU4_EnableClock(DEBUG_PWM_CCU4_MODULE, DEBUG_PWM_0_SLICE_NUM);
  /* Initialize the Slice */
  XMC_CCU4_SLICE_CompareInit(DEBUG_PWM_0_SLICE, &DebugPwmSlice_Cfg);

  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_0_SLICE, DEBUG_PWM_50_PERCENT_DC_CNTS);
  XMC_CCU4_SLICE_SetTimerPeriodMatch( DEBUG_PWM_0_SLICE, DEBUG_PWM_PERIOD_CNTS);
  XMC_CCU4_SLICE_SetTimerValue( DEBUG_PWM_0_SLICE, 0U);
  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t) DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk);

  /* Setup the I/O Pin */
  XMC_GPIO_SetMode(DEBUG_PWM_0_PORT, DEBUG_PWM_0_PIN, DEBUG_PWM_0_ALT_OUT);

  XMC_CCU4_SLICE_StartTimer(DEBUG_PWM_0_SLICE);
  #endif /* (DEBUG_PWM_0_ENABLE == 1) */

  #if	(DEBUG_PWM_1_ENABLE == 1U)
  /* Init Debug PWM Slice */
  /* Get the slice out of idle mode */
  XMC_CCU4_EnableClock(DEBUG_PWM_CCU4_MODULE, DEBUG_PWM_1_SLICE_NUM);
  /* Initialize the Slice */
  XMC_CCU4_SLICE_CompareInit(DEBUG_PWM_1_SLICE, &DebugPwmSlice_Cfg);

  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_1_SLICE, DEBUG_PWM_50_PERCENT_DC_CNTS);
  XMC_CCU4_SLICE_SetTimerPeriodMatch( DEBUG_PWM_1_SLICE, DEBUG_PWM_PERIOD_CNTS);
  XMC_CCU4_SLICE_SetTimerValue(DEBUG_PWM_1_SLICE, 0U);
  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t) DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk);

  /* Setup the I/O Pin */
  XMC_GPIO_SetMode(DEBUG_PWM_1_PORT, DEBUG_PWM_1_PIN, DEBUG_PWM_1_ALT_OUT);

  XMC_CCU4_SLICE_StartTimer(DEBUG_PWM_1_SLICE);
  XMC_CCU4_StartPrescaler(DEBUG_PWM_CCU4_MODULE);
  #endif /* (DEBUG_PWM_1_ENABLE == 1) */


}



























