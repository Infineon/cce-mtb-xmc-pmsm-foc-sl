/**
 * @file pmsm_foc_current_singleshunt.c
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
 * @file pmsm_foc_current_singleshunt.c
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

#include "pmsm_foc_current_singleshunt.h"

/* First CCU83 period, for ADC trigger */
#define CCU83_PERIOD_1ST  TZ_PZVX2
/* Second CCU83 period, for ADC trigger */
#define CCU83_PERIOD_2ND  (PERIOD_REG - CCU83_PERIOD_1ST-1)


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ***************************************/

__RAM_FUNC void pmsm_foc_adc34_triggersetting (ADCType* const HandlePtr)
{
  /* Set ADC trigger for ADC3/4 of single-shunt current sensing: */
  /* Second CCU83 period is a constant, e.g.: (Ts - 2Tz - 1). */
  CCU80_CC83->PRS = CCU83_PERIOD_2ND;
  /* For ADC 3 trigger. */
  CCU80_CC83->CR1S = HandlePtr->ADC3Trig_Point;
  /* For ADC 4 trigger. */
  CCU80_CC83->CR2S = HandlePtr->ADC4Trig_Point;
  /* Enable shadow shadow transfer for slice 3 for CCU80 Kernel */
  CCU80->GCSS = 0x1000;

}


__RAM_FUNC void pmsm_foc_adctz12_triggersetting (void)
{
  /* First CCU83 period is a constant, e.g.: 2Tz. */
  CCU80_CC83->PRS = CCU83_PERIOD_1ST;
  /* For ADCTz1 trigger. */
  CCU80_CC83->CR1S = TRIGGER_POINT;
  /* For ADCTz2 trigger. */
  CCU80_CC83->CR2S = TZ_PZV + TRIGGER_POINT;
  /* Enable shadow transfer for slice 3 for CCU80 Kernel.*/
  CCU80->GCSS = 0x1000;

}
