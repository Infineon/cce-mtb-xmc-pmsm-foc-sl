/**
 * @file pmsm_foc_interface.c
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
 * @file pmsm_foc_interface.c
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

#include "pmsm_foc_interface.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ***************************************/
/* Global variables:*/
extern ClarkeTransformType Clarke_Transform;
extern Car2PolType Car2Polar;
extern PLL_EstimatorType PLL_Estimator;
/* Angle γ (1Q23 << 8) of current space vector, from last PWM cycle */
extern int32_t I_AngleQ31;
/* ADC results, trigger positions. */
extern ADCType ADC;
/* Motor control information */
extern MotorControlType Motor;
/* Motor current and current space vector. */
extern CurrentType Current;
/* For trip / over-current detection, and protection. */
extern TripType Trip;
/* For motor startup lock / fail / stall detection, and protection. */
extern StallType Stall;
/* For Hall signal processing. */
extern HallType Hall;
/* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
extern SVMType SVM;
/* Output for FOC LIB. */
extern FOCOutputType FOCOutput;
/* Parameters input for FOC LIB. */
extern FOCInputType FOCInput;
/* Speed PI controller. */
extern PI_Coefs_Type PI_Speed;
/* Torque / Iq PI controller. */
extern PI_Coefs_Type PI_Torque;
/* Flux /Id PI controller. */
extern PI_Coefs_Type PI_Flux;
/* PLL rotor speed PI controller. */
extern PI_Coefs_Type PI_PLL;
#if(CATCH_FREE_RUNNING == ENABLED)
/* Catch Free Running. */
extern CFR_type CFR;
#endif

/* Data Structure initialization */



/* Global variable. MCU Reset Status Information, reason of last reset. */
extern uint32_t * NEW_SHS0_CALOC1;
extern uint32_t g_mcu_reset_status;

/* 0.05s, time that motor keeps in Stop Mode, x PWM period. */
#define TIME_OF_STOP 	(200U)

#define TIME_OF_VFRAMP		(750U)
/* Step that voltage increases. */
#define ALIGNMENT_STEP		(32U)
/* Voltage for rotor preposition/alignment. */
#define ALIGNMENT_VOLT		((VQ_VF_OFFSET * 2) >> 1)
/* Ratio for ramp up slowly in V/f open loop.*/
#define RAMP_RATIO_VF		(1U)
/* Indicates a running CORDIC calculation if MATH->STATC[0] (i.e.: BSY) = 1. */
#define CORDIC_IS_BSY (MATH->STATC & 0x01)


/* In case of any error, motor start function won't be started
* until clear all the errors.
* ----------------------------------------------------------*/
void pmsm_foc_motor_start(void)
{
  if((Motor.State != TRAP_PROTECTION) || (Motor.State != DCLINK_OVER_VOLTAGE)|| (Motor.State != DCLINK_UNDER_VOLTAGE))
  {
    Motor.State = EN_INVERTER_BOOTSTRAP;
  }

}



void pmsm_foc_motor_stop(void)
{
	if (Motor.State == TRAP_PROTECTION){
		pmsm_foc_Clear_trap();
		pmsm_foc_init ();
	}
	Motor.State = MOTOR_STOP;

}

void pmsm_foc_motor_brake(void){

	if ((Motor.State == TRAP_PROTECTION) | (Motor.State == MOTOR_HOLD) | (Motor.State == DCLINK_UNDER_VOLTAGE) | (Motor.State == DCLINK_OVER_VOLTAGE)){
		pmsm_foc_motor_stop();
		pmsm_foc_Clear_trap();
		pmsm_foc_init ();
	} else {
		Motor.State = FOC_CLOSED_LOOP_BRAKE;
	}
           /* Next, go to Motor Stop (may brake motor and cause vibration). */
}
void pmsm_foc_Clear_trap(void){
  XMC_CCU8_SLICE_ClearEvent(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_IRQ_ID_EVENT2);
  Motor.CCU8_Trap_Status = 0x00;
  Motor.State = MOTOR_IDLE;           /* Next, go to Motor Stop (may brake motor and cause vibration). */
}

/* API to perform motor STOP
* ----------------------------------------------------------*/
//void pmsm_foc_setspeedzero(void)
//{
//  /* Change motor control state machine to stop */
//  ADC.ADC_POT = 0;
//}
/* V/f Open Loop Ramp Up with rotor initial preposition/alignment
 * ----------------------------------------------------------*/
void pmsm_foc_vf_openloop_rampup(void)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

#else
    pmsm_foc_get_adcphasecurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);
    pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
#endif

    /* To get I_Alpha and I_Beta of last PWM cycle, scale down I_Mag (i.e.: |I|) by 2/3. */
    pmsm_foc_clarketransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

    /* Update V/f voltage amplitude, Vref = Offset + Kω. */
    Car2Polar.SVM_Vref16 = VQ_VF_OFFSET + (VQ_VF_SLEW * (Motor.Speed >> RES_INC));

    /* To update angle θ (16-bit) of SVM reference vector Vref. */
    pmsm_foc_update_vref_angle (Motor.Speed);

    if (Motor.Transition_Status == MOTOR_TRANSITION)
    {       /* Motor is in transition mode */
      if (Motor.Speed < VF_TRANSITION_SPEED)
      {
      /* Motor speed not reach V/f open-loop to MET/FOC transition speed */
      /* Speed ramp counter ++. */
      Motor.Ramp_Counter ++;
        if (Motor.Ramp_Counter > Motor.Ramp_Up_Rate)
        {
            /* Ramp up slowly in V/f.*/
            if (Motor.Ramp_Up_Rate > (RAMPUP_RATE << RAMP_RATIO_VF))
            {
              /* Increase acceleration step by step.*/
              Motor.Ramp_Up_Rate --;
            }
            /* Motor speed ++. */
            Motor.Speed += SPEEDRAMPSTEP;
            /* Clear ramp counter.*/
            Motor.Ramp_Counter = 0;
        }
      }
      else
      {
        /* Motor run at V/f constant speed for a while.*/
        Motor.Counter ++;
        if (Motor.Counter > TIME_OF_VFRAMP)
        {
          /* Change flag: motor in stable mode of V/f ramp-up. */
          Motor.Transition_Status = MOTOR_STABLE;
          Motor.Counter = 0;
        }
      }
    }
    else
    {
#if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)
      if(MOTOR_HOLD_THRESHOLD)
      {
        Motor.Counter = 0;                /* Clear counters. */
        Motor.Ramp_Counter = 0;
        Motor.State = MOTOR_HOLD;         /* Next, go to Motor Stop. */
      }

#elif(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC)
      /* Next, go to MET (Maximum Efficiency Tracking) closed-loop control. */
      Motor.State = MET_FOC;

      FOCInput.BEMF1 = 0;
      FOCInput.BEMF2 = 0;

      /* MET loop unlocked, Init e Threshold LOW. (Initially, e_Th = e_Th_L.) */
      FOCInput.Threshold_LOW = THRESHOLD_LOW;
      FOCInput.Threshold_HIGH = THRESHOLD_HIGH;
      FOCInput.Threshold = (Motor.Speed * THRESHOLD_LOW) >> RES_INC;

      FOCInput.Phase_L = L_OMEGALI;
      FOCInput.Phase_L_Scale = SCALE_L;

      /* Resolution increase, use (16 + Res_Inc) bit to represent 360 deg. */
      FOCInput.Res_Inc = RES_INC;
      FOCInput.LPF_N_BEMF = SHIFT_MET_PLL;

      /* Motor in transition mode.*/
      Motor.Transition_Status = MOTOR_TRANSITION;
      Motor.Counter = 0;
      Motor.Ramp_Counter = 0;
      Motor.Alignment_Counter = 0;
      /* Slower ramp up and ramp down for S-curve profile.*/
      Motor.Ramp_Up_Rate = RAMPUP_RATE << USER_RATIO_S;
#endif
    }

    /* Limit of |Vref| (16-bit). */
    #define SVM_VREF16_MAX    (32767U)
    if (Car2Polar.SVM_Vref16 > SVM_VREF16_MAX)
    {
      /*  Limit |Vref| maximum value.*/
      Car2Polar.SVM_Vref16 = SVM_VREF16_MAX;
    }

    /*  Update SVM PWM. */
//    pmsm_foc_svpwm_update(Car2Polar.SVM_Vref16, (0xFFFF & Car2Polar.SVM_Angle16));
    pmsm_foc_svpwm_update(Car2Polar.SVM_Vref16,  ( 0xFFFFFF & (Car2Polar.SVM_Angle16 << 8U)));

    /* Record SVM reference vector magnitude (32-bit) of last PWM cycle.*/
    Car2Polar.Vref32_Previous = Car2Polar.Vref32;
    Car2Polar.Vref32 = Car2Polar.SVM_Vref16 << CORDIC_SHIFT;

    /* Init for smooth transition from V/f to FOC closed-loop.*/
    pmsm_foc_init_smooth_transition_to_foc ();
    Car2Polar.Vref_AngleQ31 = Car2Polar.SVM_Angle16 << 16;

}




/** Stop the motor, check PWM or POT ADC (for adjusting motor speed)
 ** Execution time: ?us (O3 - Optimize most).
	* ---------------------------------------------------------------------*/
void pmsm_foc_motor_hold (void)
{
    static uint32_t  local_counter = 0;      // General purpose counter

    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);
    #else

      /* 2or3-shunt 3-phase current reconstruction, to get Iu and Iv */
    pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif
    pmsm_foc_clarketransform(Current.I_U, Current.I_V,Current.I_W, &Clarke_Transform);
    local_counter ++;


    if (MOTOR_HOLD_THRESHOLD)
    {
      /* If system is idle, i.e.: PWM duty cycle or POT ADC too low.*/
      /* Reset counter, local_counter < TIME_OF_STOP to prevent it from re-start of motor.*/
      local_counter = 0;
      Motor.Speed = 0;
    }


    if (local_counter > TIME_OF_STOP)
    {
      local_counter = 0;

      #if (CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      /* Init ADC, for current sensing, ADC of DC link Vdc (and POT). Do at later stage of the init */
      pmsm_adc_module_init();
      pmsm_phasecurrent_init();
      pmsm_adc_dclink_init();
      pmsm_adc_pot_init();
      #endif

      /* Direct FOC startup. Motor startup to FOC closed-loop directly, no V/f or MET*/
      #if(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC || MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC || MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
      /* Next, go to rotor initial preposition/alignment.*/
      Motor.State = EN_INVERTER_BOOTSTRAP;

      #elif(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC)
      Motor.State = VFOPENLOOP_RAMP_UP;             // Next, go to V/f ramp-up and re-start the motor.
      #elif(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)
      Motor.State = EN_INVERTER_BOOTSTRAP;
      #endif

      if (Motor.State == VFOPENLOOP_RAMP_UP)
      {
        Motor.Ramp_Up_Rate = (RAMPUP_RATE * 40); /* In V/f, much slower initial ramp up for S-curve profile. */
      }
    }
    else
    {
      /* To update angle θ (16-bit) of SVM reference vector Vref */
      pmsm_foc_update_vref_angle (Motor.Speed);
      /* Update SVM PWM, brake motor.*/
//      pmsm_foc_svpwm_update(0, (0xFFFF & Car2Polar.SVM_Angle16));
      pmsm_foc_svpwm_update(0,  ( 0xFFFFFF & (Car2Polar.SVM_Angle16 << 8U)));
    }

}

/** To brake the motor, charge gate driver bootstrap capacitors (if any)
 ** Execution time: ?us (O3 - Optimize most).
  * -------------------------------------------------------------------------*/
void pmsm_foc_bootstrap_charge(void)
{

    Motor.Counter++;

    if (Motor.Counter > BRAKE_TIME)
    {

#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY)||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
      /* Timer decides when to stop motor braking.*/
      Motor.State = VFOPENLOOP_RAMP_UP;
      /* Next, go to V/f ramp-up.*/
      Motor.Transition_Status = MOTOR_TRANSITION;
      /* Motor in transition mode.*/
      if (Motor.State == VFOPENLOOP_RAMP_UP)
      {
        /* In V/f, much slower initial ramp up for S-curve profile.*/
        Motor.Ramp_Up_Rate = (RAMPUP_RATE * 40);
      }
#elif((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC)||(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)||(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC))
      /* Next, go to rotor initial preposition/alignment. */
      Motor.State = (uint32_t) PRE_POSITIONING;

      /* Motor in transition mode. */
      Motor.Transition_Status = MOTOR_TRANSITION;

#endif
      /* Clear counters. */
      Motor.Counter = 0U;
      Motor.Ramp_Counter = 0U;

      Current.I_U = 0;
      Current.I_V = 0;
      Current.I_W = 0;

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      ADC.ADC3Trig_Point = 0;
      ADC.ADC4Trig_Point = 0;
      ADC.ADC_Result1 = 0;
      ADC.ADC_Result2 = 0;
      ADC.Result_Flag = RESULTS_ADCTZ12;
      /* Set ADC trigger for ADCTz1/Tz2, for single-shunt current sensing only. */
      pmsm_foc_adctz12_triggersetting ();
#endif
    }
}

#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
__RAM_FUNC void pmsm_foc_linear_ramp_generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val)
{
  if (*reference_val != set_val)
  {
    /* in FOC, ωref not reach the target speed.*/
    if (*reference_val < set_val)
    {
      /* Speed ramp counter ++.*/
      Motor.Ramp_Counter ++;
      if (Motor.Ramp_Counter > Motor.Ramp_Up_Rate)
      {
        /* Ramp up slowly (if needed) at start of FOC.*/
        if (Motor.Ramp_Up_Rate > rampup_rate)
        {
          /* Increase acceleration step by step.*/
          Motor.Ramp_Up_Rate --;
        }

        /*  ωref ++.*/
        *reference_val += speedrampstep;
        /* Clear ramp counter. */
        Motor.Ramp_Counter = 0;
      }
    }
    else
    {
      /* Speed ramp counter ++.*/
      Motor.Ramp_Counter ++;
      if (Motor.Ramp_Counter > rampdown_rate)
      {
        /* ωref --.*/
        *reference_val -= speedrampstep;
        Motor.Ramp_Counter = 0;
      }
    }
  }
  else
  {
    /* ωref reach the target speed */
    /* Update counter */
    Motor.Counter ++;
    /*15, 150 or 1500. Time that FOC becomes stable, x PWM period.*/
    if (Motor.Counter > 2U)
    {
      /* Change flag: in FOC stable mode. */
      Motor.Transition_Status = MOTOR_STABLE;
      Motor.Counter = 0;
      /* Clear counter */
      Motor.Ramp_Counter = 0;
    }
  }
}

/*  To use S-curve profile in motor ramp up / down. Comment out to use trapezoidal profile. */
#define S_CURVE_PROFILE   1
/* Speed threshold for entering second S-curve of ramp up / down. */
#define SPEED_TH_2ND_S    (SPEED_LOW_LIMIT >> 0U)


__RAM_FUNC void pmsm_foc_scurve_ramp_generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val)
{
  if (*reference_val == set_val)
  {
    /* For most of the time, motor ref speed = speed set by POT ADC or PWM. */
    Motor.Ramp_Counter = 0;
    Motor.Ramp_Up_Rate = rampup_rate << USER_RATIO_S;

    /* Reset to slower ramp up and ramp down for S-curve profile. */
    Motor.Ramp_Dn_Rate = rampdown_rate << (USER_RATIO_S - 1);
  }
  else if (*reference_val < set_val)
  {
    /* Motor ref speed lower than speed set by POT or PWM. */
    /* Speed ramp counter ++. */
    Motor.Ramp_Counter++;
    if (Motor.Ramp_Counter > Motor.Ramp_Up_Rate)
    {
      if ((set_val - *reference_val) > SPEED_TH_2ND_S)
      {
        /* First S-curve of ramp up, and constant acceleration. */
        if (Motor.Ramp_Up_Rate > rampup_rate)
        {
          /* Increase acceleration step by step. */
          Motor.Ramp_Up_Rate--;
        }
      }
      else
      {
        /* Second S-curve of ramp up. */
        if (Motor.Ramp_Up_Rate < (rampup_rate << USER_RATIO_S))
        {
          Motor.Ramp_Up_Rate++;
        }
      }
      /* Motor ref speed ++. */
      *reference_val += speedrampstep;
      Motor.Ramp_Counter = 0;
    }
  }
  else
  {
    /* Motor ref speed higher than speed set by POT or PWM. */
    /* Speed ramp counter ++. */
    Motor.Ramp_Counter++;
    if (Motor.Ramp_Counter > Motor.Ramp_Dn_Rate)
    {
      if ((*reference_val - set_val) > SPEED_TH_2ND_S)
      {
        /* First S-curve of ramp down, and constant deceleration. */
        if (Motor.Ramp_Dn_Rate > rampdown_rate)
        {
          /* Increase deceleration step by step. */
          Motor.Ramp_Dn_Rate--;
        }
      }
      else
      {
        /* Second S-curve of ramp down. */
        if (Motor.Ramp_Dn_Rate < (rampdown_rate << (USER_RATIO_S - 1)))
        {
          Motor.Ramp_Dn_Rate++;
        }
      }

        if (ADC.ADC_DCLink < VDC_MAX_LIMIT)
        {
          /* If DC link voltage Vdc is too high, stop ramp-down motor.*/
          /* Motor ref speed --.*/
          *reference_val -= speedrampstep;
        }
        Motor.Ramp_Counter = 0;
      }

    }

 }

#elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
__RAM_FUNC void pmsm_foc_linear_torque_ramp_generator(int32_t current_set, int32_t inc_step, int32_t dec_step, FOCInputType* const HandlePtr)
{
    static uint32_t Iq_counter = 0;

    Iq_counter++;
    if(Iq_counter >= USER_IQ_RAMP_SLEWRATE)
    {
        Iq_counter = 0;
        if( HandlePtr->Ref_Iq < current_set)
        {
          HandlePtr->Ref_Iq += inc_step;
        }
        else if(HandlePtr->Ref_Iq > current_set)
        {
          if((HandlePtr->Ref_Iq >= dec_step )&& (ADC.ADC_DCLink < VDC_MAX_LIMIT))
          {
            HandlePtr->Ref_Iq -=  dec_step;
          }

        }
    }

    /* Limit protection for ref_iq, the max value is capped up to 1Q15*/
    if(HandlePtr->Ref_Iq > USER_IQ_REF_HIGH_LIMIT)
      HandlePtr->Ref_Iq = USER_IQ_REF_HIGH_LIMIT - 1;

}
#elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
__RAM_FUNC void pmsm_foc_linear_vq_ramp_generator(int32_t current_set, int32_t inc_step,
                                                                    int32_t dec_step, FOCInputType* const HandlePtr)
{
    static uint32_t vq_counter = 0;

    vq_counter++;
    if(vq_counter >= USER_VQ_RAMP_SLEWRATE)
    {
        vq_counter = 0;
        if( HandlePtr->Vq < current_set)
        {
          HandlePtr->Vq += inc_step;
        }
        else if(HandlePtr->Vq > current_set)
        {
          if((HandlePtr->Vq >= dec_step )&& (ADC.ADC_DCLink < VDC_MAX_LIMIT))
          {
            HandlePtr->Vq -=  dec_step;
          }

        }
    }
  /* Limit protection for Vq, the max value is capped up to 1Q15 */
  if (HandlePtr->Vq > USER_VQ_REF_HIGH_LIMIT)
  {
    HandlePtr->Vq = USER_VQ_REF_HIGH_LIMIT - 1;
  }

}
#endif


/* Non-Real-Time Tasks Configuration */
/* 2 ~ 100, x CCU8 PWM period. For tasks that don't need real-time computing.*/
#define NON_REALTIME_RATE 64
#define POTADC_LPF    (5U)          // (5U). ADC uses LPF.
/** Miscellaneous works in CCU80_0_IRQHandler, such as tasks that don't need real-time computing
	* -------------------------------------------------------------------------------------------------*/
__RAM_FUNC void pmsm_foc_misc_works_of_irq (void)
{
  /* Handle tasks that don't need real-time computing:*/
  #if(SETTING_TARGET_SPEED == BY_POT_ONLY)
      uint16_t pot_adc_result;
  #endif


      /* Counter ++. */
	Motor.Non_RealTime_Counter ++;
	if (Motor.Non_RealTime_Counter > NON_REALTIME_RATE)
	{
	    /* Reset counter.*/
		  Motor.Non_RealTime_Counter = 0;

#if(SETTING_TARGET_SPEED == BY_POT_ONLY)
        pot_adc_result = XMC_VADC_GROUP_GetResult(VADC_POT_GROUP,VADC_POT_RESULT_REG);
#if(ADC_STARTUP_CALIBRATION == ENABLED)
      /* Clear offset calibration values*/
      CLEAR_OFFSET_CALIB_VALUES;
#endif
        /* POT ADC LPF. Read RES7 for ADC result (Previous ADC result). */
        ADC.ADC_POT = (ADC.ADC_POT * ((1<<POTADC_LPF)-1) + pot_adc_result) >> POTADC_LPF;
#endif


#if( SETTING_TARGET_SPEED == BY_POT_ONLY | SETTING_TARGET_SPEED == BY_UART_ONLY)
      #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
        /* POT ADC values 0 ~ 2^12 represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/
        Motor.Target_Speed = SPEED_LOW_LIMIT + (((SPEED_HIGH_LIMIT - SPEED_LOW_LIMIT) * ADC.ADC_POT) >> 12);
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        Motor.Target_Speed = MIN_MAX_LIMIT(Motor.Target_Speed, SPEED_HIGH_LIMIT, SPEED_LOW_LIMIT);
      #elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
        /* POT ADC values 0 ~ 2^12 represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/
        Motor.Target_Torque = USER_IQ_REF_LOW_LIMIT + (((USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT) * ADC.ADC_POT) >> 12);
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        Motor.Target_Torque = MIN_MAX_LIMIT(Motor.Target_Torque, USER_IQ_REF_HIGH_LIMIT, USER_IQ_REF_LOW_LIMIT);
      #elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
        /* POT ADC values 0 ~ 2^12 represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/
        Motor.Target_Voltage = USER_VQ_REF_LOW_LIMIT + (((USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT) * ADC.ADC_POT) >> 12);
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        Motor.Target_Voltage = MIN_MAX_LIMIT(Motor.Target_Voltage, USER_VQ_REF_HIGH_LIMIT, USER_VQ_REF_LOW_LIMIT);
      #endif
#endif

			Motor.UART_Counter ++;
			if (Motor.UART_Counter > (3U))
			{
        /* Reset counter.*/
        Motor.UART_Counter = 0;
        #if(SETTING_TARGET_SPEED == BY_UART_ONLY)
        /* Use UART to adjust POT ADC values, and hence motor speed.*/
        /* Use UART to set POT ADC, by polling.*/
        pmsm_foc_uart_set_pot_adc();
        #endif
			}

	}

}




#if(CATCH_FREE_RUNNING == ENABLED)



#define TIME_CPU_WAIT   (300)               // 0.2s, Time that CPU wait, x SVMPWM period.
/* Motor phase U, V and W are High_Z, motor coasting. CPU wait for some time, e.g.: for motor current decrease to zero
   * -------------------------------------------------------------------*/
void pmsm_foc_motor_coasting(void)
{
  pmsm_foc_adc_cfr_init();
  pmsm_foc_ccu8_cfr_init();

  CFR.counter1 ++;

  if(CFR.counter1 > TIME_CPU_WAIT)
  {
    CFR.counter1 = 0;
    CFR.counter2 = 0;
    Motor.State = PRE_CHARGE;
  }

}


#define   TIME_1PHASE_LOW               1              /* Time that one motor phase is pulled to LOW continuously, x SVMPWM period.*/
#define   TOTAL_PRECHARGE_TIME          (400U)         /* Total pre-charge time of bootstrap capacitors. */
/* Repetitive and alternate charging sequences for bootstrap capacitors
 * to sequentially ON only one low-side switching device at one time (U->V->W->U->V->W->...)
   * -------------------------------------------------------------------*/
void pmsm_foc_cfr_precharge_bootstrap (void)
{
    CFR.counter1 ++;
    if(CFR.counter1 > TIME_1PHASE_LOW)
    {
      CFR.counter1 = 0;
      CFR.motor_phase++;
      if(CFR.motor_phase > CFR_PHASE_W)
      {
        CFR.motor_phase = CFR_PHASE_U;
      }

      CFR.counter2 ++;
      if(CFR.counter2 > TOTAL_PRECHARGE_TIME)
      {
        CFR.motor_phase = (CFR_PHASE_W + 1);

        CFR.counter1 = 0;
        CFR.counter2 = 0;
        ADC.BEMF_U = 0;
        ADC.BEMF_V = 0;
        ADC.BEMF_W = 0;

        Motor.State = CATCH_FREERUNNING;
      }
    }
}


/* Set CCU8 to pull only one motor phase LOW in next PWM cycle. For catch free-running motor
 *
 * -------------------------------------------------------------------*/
void pmsm_foc_pull_1phase_low(uint16_t phase_no)
{
  switch(phase_no)
  {
    case CFR_PHASE_U:
      break;
    case CFR_PHASE_V:
      break;
    case CFR_PHASE_W:
    break;
    default:
      break;

  }
  CCU8_MODULE->GCSS = (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0|XMC_CCU8_SHADOW_TRANSFER_SLICE_1|XMC_CCU8_SHADOW_TRANSFER_SLICE_2);

}

#endif


int32_t pmsm_foc_get_motor_speed(void){
  int32_t Speed_in_rpm;
  Speed_in_rpm = (Motor.Speed * SPEED_TO_RPM ) >> SCALE_SPEED_TO_RPM;
  return Speed_in_rpm;
}


float_t pmsm_foc_get_Vdc_link(void){
	float_t Vdc_link;
	Vdc_link = ((ADC.ADC_DCLink)*VADC_DCLINK_SCALE);
	return Vdc_link;
}


void pmsm_foc_disable_inverter(void){

  XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, DISABLE_LEVEL); /* Disable gate driver. */
  XMC_GPIO_SetMode(PHASE_U_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_U_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_V_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_V_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_W_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_W_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  Motor.Inverter_status = 0;

}
void pmsm_foc_enable_inverter(void){

  XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, ENABLE_LEVEL); /* Enable gate driver. */
  XMC_GPIO_SetMode(PHASE_U_HS_PIN, PHASE_U_HS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_U_LS_PIN, PHASE_U_LS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_V_HS_PIN, PHASE_V_HS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_V_LS_PIN, PHASE_V_LS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_W_HS_PIN, PHASE_W_HS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_W_LS_PIN, PHASE_W_LS_ALT_SELECT);
  Motor.Inverter_status = 1;

}



#if(SETTING_TARGET_SPEED == SET_TARGET_SPEED)
#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
  void pmsm_foc_set_motor_target_speed(int32_t motor_target_speed_rpm){
	  int32_t motor_target_speed;
	  motor_target_speed = (motor_target_speed_rpm << SCALE_SPEED_TO_RPM) * RPM_TO_SPEED;
	  Motor.Target_Speed = motor_target_speed;
  }
#elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
  void pmsm_foc_set_motor_target_torque(int32_t motor_target_torque){
	  Motor.Target_Torque = motor_target_torque;
  }
#elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
  void pmsm_foc_set_motor_target_voltage(int32_t motor_target_voltage){
    Motor.Target_Voltage = motor_target_voltage;
  }
#endif

#endif
