/**
 * @file pmsm_foc_vadc_isr.c
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
 * @file pmsm_foc_vadc_isr.c
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
#include <xmc_common.h>   /* SFR declarations of the selected device */
#include "../MCUInit/adc.h"
//#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_interface.h"

/***********************************************************************************************************************
 * GLOBAL DATA
***********************************************************************************************************************/
extern ADCType ADC;                             /* ADC results, trigger positions. */
extern MotorControlType Motor; /* Motor control information */


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ***************************************/


void pmsm_foc_vadc_source_irqhandler(void)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  if(ADC.Result_Flag == RESULTS_ADCTZ12)
  {
    ADC.ADC_ResultTz1 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1];
    ADC.ADC_ResultTz2 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1];
    ADC.Result_Flag = RESULTS_ADC34;
    XMC_GPIO_SetOutputHigh (P1_5);
  }
  else if(ADC.Result_Flag == RESULTS_ADC34)
  {
    ADC.ADC_Result3 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1];
    ADC.ADC_Result4 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1];
    ADC.Result_Flag = RESULTS_ADCTZ12;

    ADC.ADC_Result1 = (ADC.ADC_Result3 - ADC.ADC_ResultTz1) << 3;
    ADC.ADC_Result2 = (ADC.ADC_Result4 - ADC.ADC_ResultTz2) << 3;
    XMC_GPIO_SetOutputHigh (P1_5);
  }
  else
  {
    ADC.ADC_Result1 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1] - ADC.ADC_Bias;
    ADC.ADC_Result2 = (uint16_t)VADC_ISS_GROUP->RES[VADC_IDC_SS_RESULT_REG_FIFO1] - ADC.ADC_Bias;
    ADC.ADC_Result1 <<= 4;
    ADC.ADC_Result2 <<= 4;
  }

  //XMC_GPIO_SetOutputHigh (P1_5);
#endif
}

#if(VDC_UNDER_OVERVOLTAGE_PROTECTION == ENABLED)
void pmsm_foc_over_under_voltage_isr(){
	uint16_t DCLink_adc_result = VADC_VDC_GROUP->RES[VADC_VDC_RESULT_REG];
	if (Motor.Transition_Status == MOTOR_STABLE)  {
		if(VADC_VDC_GROUP->CEFLAG & (1 << VADC_VDC_CHANNEL))
		{
		  if (DCLink_adc_result > VDC_OVER_LIMIT)
		  {
			/* Motor.error_status = PMSM_FOC_EID_OVER_VOLT;*/
			Motor.State = DCLINK_OVER_VOLTAGE;
			/* Disable gate driver. */
			pmsm_foc_disable_inverter();
		  }
		  else if(DCLink_adc_result < VDC_MIN_LIMIT)
		  {
			/* Motor.error_status =  PMSM_FOC_EID_UNDER_VOLT; */
			Motor.State = DCLINK_UNDER_VOLTAGE;
			/* Disable gate driver. */
			pmsm_foc_disable_inverter();
		  }
		VADC_VDC_GROUP->CEFCLR |= (1 << VADC_VDC_CHANNEL);

		}
	 }
}
#endif

