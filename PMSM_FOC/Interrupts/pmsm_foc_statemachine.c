/**
 * @file pmsm_foc_statemachine.c
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
 * @file pmsm_foc_statemachine.c
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

#include "pmsm_foc_statemachine.h"
#if (uCPROBE_GUI_OSCILLOSCOPE == ENABLED)
#include "../../ProbeScope/probe_scope.h"
#endif

#if    (PMSM_FOC_SECONDARYLOOP_CALLBACK== ENABLED)
  void pmsm_foc_secondaryloop_callback (void);
#endif

/*********************************************************************************************************************
 * GLOBAL DATA
 ***************************************/

extern MotorControlType Motor; /* Motor control information */
extern SVMType SVM; /* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
extern FOCOutputType FOCOutput; /* Output for FOC LIB. */
extern FOCInputType FOCInput; /* Parameters input for FOC LIB. */
extern Car2PolType Car2Polar;
extern CurrentType Current; /* Motor current and current space vector. */
extern PLL_EstimatorType PLL_Estimator;
extern ADCType ADC;
#if(CATCH_FREE_RUNNING == ENABLED)
extern CFR_type CFR;
#endif
/*********************************************************************************************************************
 * LOCAL ROUTINES
 ***************************************/

__RAM_FUNC void pmsm_foc_controlloop_isr(void);


__RAM_FUNC void pmsm_foc_controlloop_isr(void)
{

   switch (Motor.State)
    {

#if((MY_FOC_CONTROL_SCHEME != SPEED_CONTROLLED_VF_ONLY))
      case FOC_CLOSED_LOOP:
      case FOC_CLOSED_LOOP_BRAKE:

  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
        if(SVM.SVM_Flag == SVM_USE_PZV)
        {
          pmsm_foc_adc34_triggersetting(&ADC);
          ADC.Result_Flag = RESULTS_ADCTZ12;
        }
        else
        {
          /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
          ADC.Result_Flag = RESULTS_STANDARD_SVM;
        }

  #endif

        pmsm_foc_controller();

        /* Miscellaneous works in FOC, such as ramp up, speed adjustment, stop motor, etc. Execution time: 2.8us*/
        pmsm_foc_misc_works_of_foc ();

        /* Update SVM PWM. Execution time: 5.9us */
        pmsm_foc_svpwm_update((Car2Polar.Vref32 >> CORDIC_SHIFT), ( 0xFFFFFF & (Car2Polar.Vref_AngleQ31 >> 8U)));

        /* Record SVM sector information. */
        FOCOutput.New_SVM_SectorNo = SVM.CurrentSectorNo;

        break;
#endif

#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)|| (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
      case  VFOPENLOOP_RAMP_UP:

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        pmsm_foc_adc34_triggersetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }
#endif
      pmsm_foc_vf_openloop_rampup();

        break;
#endif
        #if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC)
      case MET_FOC:
  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
        if(SVM.SVM_Flag == SVM_USE_PZV)
        {
          pmsm_foc_adc34_triggersetting(&ADC);
          ADC.Result_Flag = RESULTS_ADCTZ12;
        }
        else
        {
          /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
          ADC.Result_Flag = RESULTS_STANDARD_SVM;
        }
  #endif
        Motor.Transition_Status = pmsm_foc_vf_smooth_transition_foc();
        pmsm_foc_misc_works_of_met();

          break;
#endif
        case EN_INVERTER_BOOTSTRAP:
          /* Brake the motor before motor startup. Charge gate driver bootstrap capacitors (if any)*/
          pmsm_foc_enable_inverter();
          pmsm_foc_bootstrap_charge();
          break;

        case MOTOR_HOLD:
  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
          if(SVM.SVM_Flag == SVM_USE_PZV)
          {
            pmsm_foc_adc34_triggersetting(&ADC);
            ADC.Result_Flag = RESULTS_ADCTZ12;
          }
          else
          {
            /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
            ADC.Result_Flag = RESULTS_STANDARD_SVM;
          }
  #endif
          pmsm_foc_motor_hold ();
          break;

        case PRE_POSITIONING:
  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
          if(SVM.SVM_Flag == SVM_USE_PZV)
          {
            pmsm_foc_adc34_triggersetting(&ADC);
            ADC.Result_Flag = RESULTS_ADCTZ12;
          }
          else
          {
            /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
            ADC.Result_Flag = RESULTS_STANDARD_SVM;
          }
  #endif
          pmsm_foc_directfocrotor_pre_positioning ();
          break;

#if(CATCH_FREE_RUNNING == ENABLED)
        case MOTOR_COASTING:
          pmsm_foc_motor_coasting();
          break;
        case PRE_CHARGE:
          pmsm_foc_cfr_precharge_bootstrap();
          break;
        case CATCH_FREERUNNING:
          pmsm_foc_readbemf_voltage(ADC.BEMF_U,ADC.BEMF_V, ADC.BEMF_W,&CFR);
          pmsm_foc_calculate_rotor_mag_angle(&CFR);
          break;
#endif
        case MOTOR_STOP:
            pmsm_foc_disable_inverter();
            Motor.Ref_Speed = 0;
            Motor.Speed = 0;
            Motor.State = MOTOR_IDLE;
        case MOTOR_IDLE:
        	pmsm_foc_get_current_bias();
          break;
        default:
          /* For trap protection if CCU8_TRAP_ENABLE (CCU8 TRAP functionality enabled)*/
          pmsm_foc_error_handling ();

          break;
      }
#if (uCPROBE_GUI_OSCILLOSCOPE == ENABLED)
ProbeScope_Sampling();
#endif

}

void pmsm_foc_secondaryloop_isr(void)
{
  uint16_t DCLink_adc_result;

  #if(WATCH_DOG_TIMER == ENABLED)
  /* Service watchdog. Without WDT service regularly , it will reset system.*/
  XMC_WDT_Service();
  #endif

     /* DC link ADC LPF. Read RES5 for ADC result (Previous ADC result) */
  DCLink_adc_result = VADC_VDC_GROUP->RES[VADC_VDC_RESULT_REG];
  #if(ADC_STARTUP_CALIBRATION == ENABLED)
        /* Clear offset calibration values*/
        CLEAR_OFFSET_CALIB_VALUES;
  #endif
  ADC.ADC_DCLink = (ADC.ADC_DCLink * ((1<<ADCLPF)-1) + DCLink_adc_result) >> ADCLPF;

  pmsm_foc_misc_works_of_irq ();

#if(PMSM_FOC_SECONDARYLOOP_CALLBACK== ENABLED)
   pmsm_foc_secondaryloop_callback();
#endif

}


