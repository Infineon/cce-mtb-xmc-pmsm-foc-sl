/**
 * @file gpio.c
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
 * @file gpio.c
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
#include "gpio.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ***************************************/
/**
 *  Data Structure initialization - GPIO Configuration for Gate Driver enable pin .
 */
XMC_GPIO_CONFIG_t IO_PadConfig_Pushpull  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
  .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)ENABLE_LEVEL,

  .input_hysteresis= XMC_GPIO_INPUT_HYSTERESIS_STANDARD

};


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ***************************************/

/* API to initialize GPIO pins used */
void pmsm_foc_gpio_Init(void)
{
	/* P0.11 as gate driver enable pin */
	XMC_GPIO_Init(INVERTER_EN_PIN, &IO_PadConfig_Pushpull);

	/* P0.0	ALT5 CCU80.OUT00 */
	XMC_GPIO_SetMode(PHASE_U_HS_PIN, PHASE_U_HS_ALT_SELECT);

	/* P0.1 ALT5 CCU80.OUT01 */
	XMC_GPIO_SetMode(PHASE_U_LS_PIN, PHASE_U_LS_ALT_SELECT);

	/* P0.2 ALT7 CCU80.OUT10 */
	XMC_GPIO_SetMode(PHASE_V_HS_PIN, PHASE_V_HS_ALT_SELECT);

	/* P0.3 ALT7 CCU80.OUT11 */
	XMC_GPIO_SetMode(PHASE_V_LS_PIN, PHASE_V_LS_ALT_SELECT);

	/* P0.8 ALT5 CCU80.OUT20 */
	XMC_GPIO_SetMode(PHASE_W_HS_PIN, PHASE_W_HS_ALT_SELECT);

	/* P0.9 ALT5 CCU80.OUT21 */
	XMC_GPIO_SetMode(PHASE_W_LS_PIN, PHASE_W_LS_ALT_SELECT);

	/* P0.12 as CCU80 Trap input, internal pull-up */
	XMC_GPIO_SetMode(TRAP_PIN, XMC_GPIO_MODE_INPUT_PULL_UP);

//  /* P1.4 as GPIO */
	XMC_GPIO_SetMode (TEST_PIN,XMC_GPIO_MODE_OUTPUT_PUSH_PULL);

	XMC_GPIO_SetOutputLow(TEST_PIN);

}


