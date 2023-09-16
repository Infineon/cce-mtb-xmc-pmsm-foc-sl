/**
 * @file pmsm_foc_transitions.c
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
 * @file pmsm_foc_transitions.c
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
#include "pmsm_foc_transitions.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ***************************************/
/* Global variables:*/
extern ClarkeTransformType Clarke_Transform;
extern Car2PolType Car2Polar;
extern PLL_EstimatorType PLL_Estimator;
/* Angle ? (1Q23 << 8) of current space vector, from last PWM cycle */
extern int32_t I_AngleQ31;
/* Local Variables */
int32_t Epsilon;
/* ADC results, trigger positions. */
extern ADCType ADC;
/* Motor control information */
extern MotorControlType Motor;
/* Motor current and current space vector. */
extern CurrentType Current;

/* Parameters input for FOC LIB. */
extern FOCInputType FOCInput;
/* PLL rotor speed PI controller. */
extern PI_Coefs_Type PI_PLL;

/* Indicates a running CORDIC calculation if MATH->STATC[0] (i.e.: BSY) = 1. */
#define CORDIC_IS_BSY (MATH->STATC & 0x01)

/** To calculate |Vref|sin(?-?), wL|I|, and e=|Vref|sin(?-?)+wL|I|, for MET (Maximum Efficiency Tracking) .
** Execution time: 5.9us - 6.3us (O3 - Optimize most).
* ----------------------------------------------------------------------------------------------------------*/
void pmsm_foc_init_smooth_transition_to_foc ()
{

    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    // Get_ADC_SingleShuntCurrent(&ADC);
    pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

    #else
    pmsm_foc_get_adcphasecurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);

    pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif

    pmsm_foc_clarketransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

    PLL_Imag(Car2Polar.Vref_AngleQ31,Clarke_Transform.I_Alpha_1Q31,Clarke_Transform.I_Beta_1Q31);

    /* Wait if CORDIC is still running calculation. Omit if CCU4 outputs debug information.*/
    while (CORDIC_IS_BSY)
    {
      continue;
    }
    PLL_Imag_GetResult(&PLL_Estimator);

    PLL_Vref(PLL_Estimator.Delta_IV, Car2Polar.Vref32,PI_PLL.Uk, FOCInput.Phase_L,&PLL_Estimator);

    /* CPU computes the following simultaneously when CORDIC #7 */
    /* ?, used for smooth MET->FOC transition and motor startup lock / fail / stall detection.*/
    I_AngleQ31 = PLL_Estimator.Delta_IV + Car2Polar.Vref_AngleQ31;

    /* Results of CORDIC #7 - Vrefxsin(γ-θ) and Vrefxcos(?-?) */
    /* Wait if CORDIC is still running calculation.*/
    while (CORDIC_IS_BSY)
    {
      continue;
    }
    PLL_Vref_GetResult(&PLL_Estimator);

    /* Shift to get real result (16-bit).*/
    PLL_Estimator.VrefxSinDelta >>= CORDIC_SHIFT;

    PLL_Estimator.VrefxSinDelta = (PLL_Estimator.VrefxSinDelta * 311) >> 8;

    /* Unity gain LPF: Y[n] = Y[n-1] + (X[n]-Y[n-1])>>FOCInput.LPF_N_BEMF */
    /* |Vref|sin(?-?) with LPF.*/
    FOCInput.BEMF1 = (FOCInput.BEMF1 * ((1 << FOCInput.LPF_N_BEMF) - 1) + PLL_Estimator.VrefxSinDelta)
    >> FOCInput.LPF_N_BEMF;

    /* ε = |Vref|sin(?-?) + wL|I|. Motor rotates in one direction only. Rotor angle always increasing.*/
    Epsilon = FOCInput.BEMF1 +  FOCInput.BEMF2;

}



#define MET_VREF_STEP     (1U)
/** For control strategy MET (Maximum Efficiency Tracking)
** Execution time: ?us (O3 - Optimize most).
* -----------------------------------------------------------*/
uint16_t pmsm_foc_vf_smooth_transition_foc (void)
{
    int32_t Vref_Change_Step;
    /* ΔV, change (increase / decrease, if necessary) step for |Vref| of SVM */
    uint16_t Status = MOTOR_TRANSITION;
    /* A temporary variable to indicate leading or lagging of I to V.*/
    int32_t Flag_Leading_Lagging;
    /* 2or3-shunt phase current sensing.*/
    FOCInput.Ref_Speed = Motor.Speed;

    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    //  Get_ADC_SingleShuntCurrent(&ADC);
    pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

    #else
    pmsm_foc_get_adcphasecurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);

    pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif
    pmsm_foc_clarketransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

    /* θ[k] = θ[k-1] + ω[k]. Motor rotates in one direction only. Rotor angle always increasing.*/
    Car2Polar.Vref_AngleQ31 += (FOCInput.Ref_Speed << (16U-FOCInput.Res_Inc));

    /* To calculate |Vref|sin(γ-θ), ωL|I|, and ε = |Vref|sin(γ-θ)+ ωL|I|. */
    pmsm_foc_init_smooth_transition_to_foc ();

    /* Bang-bang controller (aka: hysteresis controller, on-off controller) of MET Controller
    * Execution time: 1.5us~2.1us (O3 - Optimize most).
    * ------------------------------------------------------------------------------------------ */
    /* (γ-θ) >= zero (i.e.: I is leading V) is an urgent condition, to increase |Vref| immediately.*/
    Flag_Leading_Lagging = FOCInput.BEMF1;

    if (Flag_Leading_Lagging >= 0)
    {
      /* An urgent condition,*/
      /* Increase |Vref| immediately.*/
      Car2Polar.Vref32 += (MET_VREF_STEP << CORDIC_SHIFT);

      /* MET loop unlocked, ε_Th = ε_Th_L Threshold_LOW.*/
      FOCInput.Threshold = (FOCInput.Ref_Speed * FOCInput.Threshold_LOW) >> FOCInput.Res_Inc;
    }
    else
    { /* (γ-θ) < zero.*/
      if (Epsilon < 0)
      {
        Epsilon = -Epsilon;
        /* Reverse ε value, i.e.: find |ε|. */
        Vref_Change_Step = -MET_VREF_STEP;

        /* |Vref| should be decreased (or no change) if ε < 0. */
        #define MIN_AMPLITUDE (20U)
        /* Minimum value of |Vref| */
        if (Car2Polar.Vref32 <= (MIN_AMPLITUDE << CORDIC_SHIFT))
        {
          /* |Vref| cannot decrease further if it is too small.*/
          Vref_Change_Step = 0;
        }
      }
      else
      {
        /* |Vref| should be increased (or no change) if ε >= 0.*/
        Vref_Change_Step = MET_VREF_STEP;
      }

      if (Epsilon > FOCInput.Threshold)
      {
        /* If |ε| > ε_Th,*/
        /* |Vref| changes (increase or decrease) by a step.*/
        Car2Polar.Vref32 += (Vref_Change_Step << CORDIC_SHIFT);

        /* MET loop unlocked, ε_Th = ε_Th_L Threshold_LOW.*/
        FOCInput.Threshold = (FOCInput.Ref_Speed * FOCInput.Threshold_LOW) >> FOCInput.Res_Inc;
      }
      else
      { /* If |ε| <= ε_Th, |Vref| no need change.*/
        /*  MET loop locked,  ε_Th = ε_Th_H Threshold_HIGH.*/
        FOCInput.Threshold = (FOCInput.Ref_Speed * FOCInput.Threshold_HIGH) >> FOCInput.Res_Inc;

        if (FOCInput.Flag_State == 0)
        {
          /* If use 3-step motor start-up V/f-> MET-> FOC,*/
          /*  Change flag: motor in MET stable mode once it finds |ε| <= ε_Th, try to jump to FOC immediately. */
          Status = MOTOR_STABLE;
        }
      }
    }
    /* Update SVM PWM.*/
//    pmsm_foc_svpwm_update((Car2Polar.Vref32 >> CORDIC_SHIFT), (0xFFFF & Car2Polar.Vref_AngleQ31>>16U));
    pmsm_foc_svpwm_update((Car2Polar.Vref32 >> CORDIC_SHIFT), ( 0xFFFFFF & (Car2Polar.Vref_AngleQ31 >> 8U)));

    return (Status);
}


/* ~0.5s, max time that MET control takes before considered stable, x PWM period. */
#define TIME_OF_MET     (7500U)

/** Miscellaneous works in MET, such as ramp up, speed adjustment, transition from MET to FOC, etc
* ---------------------------------------------------------------------------------------------------*/
void pmsm_foc_misc_works_of_met (void)
{
    if (Motor.Transition_Status != MOTOR_TRANSITION)
    {         // Motor in transition mode.
      pmsm_foc_transition_foc();                // Transition from MET to FOC, using 3-step motor start-up V/f->MET->FOC.
    }

    if ((MOTOR_HOLD_THRESHOLD) && (Motor.Speed <= Motor.Target_Speed))
    {
      /* If PWM duty cycle or POT ADC too low, and speed reached PWM set speed */
      Motor.Counter = 0;
      Motor.Ramp_Counter = 0;
      /* Set low motor speed, so motor stop and resume can be fast.*/
      Motor.Speed = SPEED_LOW_LIMIT >> 5;
       /* Next, go to Motor Stop. */
      Motor.State = MOTOR_HOLD;
  }

} /* End of pmsm_foc_misc_works_of_met () */


/*
 * Init variables for transition to FOC, e.g. from MET to FOC: MET -> FOC
 * Execution time: ?us (O3 - Optimize most)
 * 3-step motor start-up: V/f open-loop -> MET closed-loop -> FOC closed-loop.
 */
void pmsm_foc_transition_foc(void)
{
  Motor.Counter = TIME_OF_MET + 1; /* Go to FOC immediately once it find |ε| <= ε_Th. */

  if (Motor.Counter > TIME_OF_MET)
  {
    Motor.State = FOC_CLOSED_LOOP; /* Next, go to FOC closed-loop. */

    pmsm_foc_init_foc_rotorangle(); /* Init rotor angle for first FOC PWM cycle, Lag/lead current angle γ by a 90° angle. */
    pmsm_foc_init_foc_pi_iks(); /* To init PI controllers' Ik for first FOC PWM cycle. */

    PI_PLL.Uk = Motor.Speed; /*
                              * Init FOC rotor speed ωr = PI_PLL.Uk, needed for ωL|I|, ωLId, ωLIq,
                              * and FG frequency calculation.
                              */
    Motor.Ref_Speed = Motor.Speed; /* Motor reference speed of FOC. */

    pmsm_foc_systemparameters_init_onceonly(); /* Init parameters of FOC LIB. Init once only. */

    Motor.Transition_Status = MOTOR_TRANSITION; /* Motor in transition mode. */
    Motor.Counter = 0; /* Clear counters. */
    Motor.Ramp_Counter = 0;
    Motor.Ramp_Up_Rate = RAMPUP_RATE << USER_RATIO_S; /* Slower ramp up and ramp down for S-curve profile. */
  }
} /* End of pmsm_foc_transition_foc () */



/* API to Shift times for unity gain LPF: Y[n] = Y[n-1] + (X[n]-Y[n-1]) */
__RAM_FUNC int32_t pmsm_foc_unity_gain_lpf(int32_t filter_input, uint8_t gain)
{
  static int32_t temp_output;

  temp_output = (temp_output * ((1 << gain)-1) + filter_input) >> gain;

  return temp_output;
}






