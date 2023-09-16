/**
 * @file pmsm_foc_functions.c
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
 * @file pmsm_foc_functions.c
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

#include <xmc_common.h>                   /* SFR declarations of the selected device */
#include "pmsm_foc_functions.h"
#include "pmsm_foc_interface.h"

FOCInputType FOCInput; /* Parameters input for FOC LIB. */
FOCOutputType FOCOutput; /* Output for FOC LIB. */

ClarkeTransformType Clarke_Transform;
ParkTransformType Park_Transform;

Car2PolType Car2Polar;
/* ADC results, trigger positions. */
extern ADCType ADC;
/* Motor current and current space vector. */
CurrentType Current;
/* Motor control information */
MotorControlType Motor;

/* For trip / over-current detection, and protection. */
TripType Trip;
/* For motor startup lock / fail / stall detection, and protection. */
StallType Stall;
/* For over/under-voltage detection, and protection. */
OverUnderVoltType OverUnderVolt;
/* Angle γ (1Q23 << 8) of current space vector, from last PWM cycle */
int32_t I_AngleQ31;


HallType Hall;
/* Parameters input for FOC LIB. */
extern FOCInputType FOCInput;
/* Sine LUT used for SVM calculations, array size 256 or 1024.*/
extern const uint16_t Sinus60_tab[];
/* Speed PI controller.*/
extern PI_Coefs_Type PI_Speed;
/*  Torque / Iq PI controller.*/
extern PI_Coefs_Type PI_Torque;
/*  Flux /Id PI controller.*/
extern PI_Coefs_Type PI_Flux;
/* PLL rotor speed PI controller.*/
extern PI_Coefs_Type PI_PLL;
extern PLL_EstimatorType PLL_Estimator;
extern int32_t VrefxSinDelta;
/* |I|, magnitude of current space vector */
extern uint32_t Current_I_Mag;
extern int32_t Delta_IV;

#define SHIFT_TO_1Q15		(3U)
#define MET_VREF_STEP		(1U)
/**
  * @brief	3-Shunt 3-Phase Current Reconstruction, ADC values are from last PWM cycle
  * 		ADCs of 2or3-Shunt are triggered by CCU83 CR1S
  *
  * @param	VADC_G1_RES_0
  * 		VADC_G0_RES_0
  * 		VADC_G1_RES_1
  *
  *@retval 	Current.I_U
  * 		Current.I_V
  * 		Current.I_W
  */

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
__RAM_FUNC void pmsm_foc_current_reconstruction (uint16_t Previous_SVM_SectorNo,int32_t ADC_result1, int32_t ADC_result2, CurrentType * const HandlePtr)
{
   switch (Previous_SVM_SectorNo)
  {
    case 0:  /* Sector A */
      HandlePtr->I_U = ADC_result1;
      HandlePtr->I_V = ADC_result2 - ADC_result1;
      break;

    case 1:  /* Sector B */
      HandlePtr->I_U = ADC_result2 - ADC_result1;
      HandlePtr->I_V = ADC_result1;
      break;

    case 2:  /* Sector C */
      HandlePtr->I_U = -ADC_result2;
      HandlePtr->I_V = ADC_result1;
      break;

    case 3:  /* Sector D */
      HandlePtr->I_U = -ADC_result2;
      HandlePtr->I_V = ADC_result2 - ADC_result1;
      break;

    case 4:  /* Sector E */
      HandlePtr->I_U = ADC_result2 - ADC_result1;
      HandlePtr->I_V = -ADC_result2;
      break;

    default:  /* Sector F */
      HandlePtr->I_U = ADC_result1;
      HandlePtr->I_V = -ADC_result2;
      break;
  }
   pmsm_foc_ccu4_debug3output (HandlePtr->I_U, 1,13,HandlePtr->I_V ,1,13);
}
#else
__RAM_FUNC void pmsm_foc_current_reconstruction (int32_t ADC_Iu, int32_t ADC_Iv, int32_t ADC_Iw, CurrentType * const HandlePtr)
{

	/* Motor phase current, Iu, Iv, Iw*/
	HandlePtr->I_U = ((int16_t)(ADC.ADC_Bias_Iu - ADC_Iu)) << 3;
	HandlePtr->I_V = ((int16_t)(ADC.ADC_Bias_Iv - ADC_Iv)) << 3;
	HandlePtr->I_W = ((int16_t)(ADC.ADC_Bias_Iw - ADC_Iw)) << 3;


}
#endif

	/*###* FOC controller LIB, calculated once in each PWM cycle ####
		 * ------------------------------------------------------------*/
__RAM_FUNC void pmsm_foc_controller (void)
{
  #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  pmsm_foc_current_reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);
  #else
  pmsm_foc_get_adcphasecurrent(FOCOutput.Previous_SVM_SectorNo, FOCOutput.New_SVM_SectorNo, &ADC);
	pmsm_foc_current_reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
  #endif
	Motor.Speed = FOCOutput.Speed_by_Estimator;

  #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
  #if(OVERCURRENT_PROTECTION == ENABLED)
	pmsm_foc_get_IDCLink_current();
	pmsm_foc_over_current_protection_check(ADC.ADC_IDCLink, Park_Transform.Iq, &FOCInput.overcurrent_factor);
  FOCInput.Ref_Speed = (Motor.Ref_Speed * FOCInput.overcurrent_factor) >> 12;        // Motor reference speed.
  #else
  FOCInput.Ref_Speed = Motor.Ref_Speed;        // Motor reference speed.
  #endif
  #else
  FOCInput.Ref_Speed = Motor.Ref_Speed;        // Motor reference speed.
  #endif
  pmsm_foc_clarketransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

  pmsm_foc_parktransform(Clarke_Transform.I_Alpha_1Q31, Clarke_Transform.I_Beta_1Q31, PLL_Estimator.RotorAngleQ31);

	FOCOutput.Previous_SVM_SectorNo = FOCOutput.New_SVM_SectorNo;	// Record previous SVM sector number.

  #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
	/* PI Controller #1 - Speed / Speed PI controller of FOC */
	pmsm_foc_pi_controller_anti_windup(FOCInput.Ref_Speed,PLL_Estimator.RotorSpeed_In, &PI_Speed);
  #endif

	pmsm_foc_parktransform_getresult(&Park_Transform);
	PLL_Imag(Car2Polar.Vref_AngleQ31, Clarke_Transform.I_Alpha_1Q31,Clarke_Transform.I_Beta_1Q31);
  #if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC))
 /* PI Controller #2 - Torque / Iq PI controller of FOC */
	pmsm_foc_pi_controller(PI_Speed.Uk, Park_Transform.Iq, &PI_Torque);
	Car2Polar.Torque_Vq = PI_Torque.Uk;
  #elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
  /* PI Controller #2 - Torque / Iq PI controller of FOC */
	pmsm_foc_pi_controller(FOCInput.Ref_Iq, Park_Transform.Iq, &PI_Torque);
	Car2Polar.Torque_Vq = PI_Torque.Uk;
  #elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
	Car2Polar.Torque_Vq = FOCInput.Vq;
  #endif

	Car2Polar.Torque_Vq = PI_Torque.Uk;
	PLL_Imag_GetResult(&PLL_Estimator);
	PLL_Vref(PLL_Estimator.Delta_IV, Car2Polar.Vref32, PI_PLL.Uk, FOCInput.Phase_L,&PLL_Estimator);
	/* PI Controller #3 - Flux / Id PI controller of FOC */
	pmsm_foc_pi_controller(FOCInput.Ref_Id, Park_Transform.Id, &PI_Flux);
	Car2Polar.Flux_Vd = PI_Flux.Uk;

  #if(DQ_DECOUPLING == ENABLED)
  int32_t wL_dq_Decoupling;

  // Use lower resolution speed ω = PI_PLL.Uk to prevent overflow in multiplications of ωLId and ωLIq.
  // Temp variable for ωL, L scaled up by (MPS/K)^2.
  wL_dq_Decoupling = (PI_PLL.Uk >> RES_INC) * ((uint32_t)FOCInput.Phase_L);

  Car2Polar.Torque_Vq += ((wL_dq_Decoupling * Park_Transform.Id) >> FOCInput.Phase_L_Scale);
  Car2Polar.Flux_Vd -= ((wL_dq_Decoupling * Park_Transform.Iq) >> FOCInput.Phase_L_Scale);

  Car2Polar.Torque_Vq = MIN_MAX_LIMIT(Car2Polar.Torque_Vq,PI_TORQUE_UK_LIMIT_MAX, PI_TORQUE_UK_LIMIT_MIN);
  Car2Polar.Flux_Vd = MIN_MAX_LIMIT(Car2Polar.Flux_Vd,PI_FLUX_UK_LIMIT_MAX, PI_FLUX_UK_LIMIT_MIN);
  #endif

	PLL_Vref_GetResult(&PLL_Estimator);

	pmsm_foc_cart2polar(Car2Polar.Torque_Vq, Car2Polar.Flux_Vd,PLL_Estimator.RotorAngleQ31);

	PLL_GetPosSpd(&PLL_Estimator);
	pmsm_foc_car2pol_getresult(&Car2Polar);

	uint32_t SVM_Vref16;
	SVM_Vref16 = Car2Polar.Vref32 >>CORDIC_SHIFT;

	SVM_Vref16 = (SVM_Vref16 * 311) >> 8;
	Car2Polar.Vref32 = SVM_Vref16 << CORDIC_SHIFT;

	FOCOutput.Speed_by_Estimator = PLL_Estimator.RotorSpeed_In;
	FOCOutput.Rotor_PositionQ31 = PLL_Estimator.RotorAngleQ31;
}




/* To init rotor angle 1Q31 for first FOC PWM cycle, Lag/lead current angle Î³ by a 90Â° angle */
void pmsm_foc_init_foc_rotorangle(void)
{
  PLL_Estimator.RotorAngleQ31 = (I_AngleQ31 - DEGREE_90) + (FOCInput.Ref_Speed << (16U - FOCInput.Res_Inc));

} /* End of pmsm_foc_init_foc_rotorangle () */

/* To init PI controllers' integral terms (Ik) for first FOC PWM cycle */
void pmsm_foc_init_foc_pi_iks(void)
{
  PI_Speed.Ik = PLL_Estimator.Current_I_Mag; /* Init PI integral terms for smooth transition to FOC. */
  PI_Torque.Ik = (PLL_Estimator.VrefxCosDelta * 256) >> 8; /*
                                              * Init Vq of torque / Iq PI controller,
                                              * |Vref|cos(Î³-Î¸) = |Vref|cos(Î¸-Î³).
                                              */
  PI_Flux.Ik = (PLL_Estimator.VrefxSinDelta * 256) >> 8; /* Init Vd of flux / Id PI controller, |Vref|sin(Î³-Î¸) < 0 typically. */

  PI_PLL.Ik = FOCInput.Ref_Speed; /* Init rotor speed Ï‰r of PLL Observer PI controller. */

  PI_Speed.Ik <<= PI_Speed.Scale_KpKi; /* All PI integral terms left shift by PI_data->Scale_KpKi. */
  PI_Torque.Ik <<= PI_Torque.Scale_KpKi;
  PI_Flux.Ik <<= PI_Flux.Scale_KpKi;
  PI_PLL.Ik <<= PI_PLL.Scale_KpKi;

	}	// End of pmsm_foc_init_foc_pi_iks ()



/*
 * To update angle θ (16-bit) of SVM reference vector Vref
 * Digital implementation θ[k] = θ[k-1] + ω[k]
 */
void pmsm_foc_update_vref_angle (int32_t Speed)
{
	Car2Polar.Vref_AngleQ31_Previous =  Car2Polar.Vref_AngleQ31;		// Record Vref angle θ of last PWM cycle.

  if (Motor.Rotation_Dir == DIRECTION_INC)
  {
    /* If motor rotation direction - angle increasing. θ[k] = θ[k-1] + ω[k]. */
    Car2Polar.Vref_AngleQ31 += (Speed << (16U - RES_INC)); /* θ[k] = θ[k-1] + ω[k]. */
  }
  else
  {
		Car2Polar.Vref_AngleQ31 -= (Speed << (16U-RES_INC));		// θ[k] = θ[k-1] - ω[k].
	}

	Car2Polar.SVM_Angle16 = Car2Polar.Vref_AngleQ31 >> 16U;			// SVM Vref angle θ (16-bit).
}	// End of pmsm_foc_update_vref_angle ()




	/** Miscellaneous works in FOC, such as ramp up, speed adjustment, stop motor, etc
	 ** Do NOT add any CORDIC calculations in this function.
		 * -----------------------------------------------------------------------------------*/
__RAM_FUNC void pmsm_foc_misc_works_of_foc (void)
{


#if(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
	if (Motor.State != FOC_CLOSED_LOOP_BRAKE && (MOTOR_HOLD_THRESHOLD) && (FOCInput.Ref_Iq <= Motor.Target_Torque)  )
	{
		Motor.State = MOTOR_HOLD;
	}


    if (Motor.State == FOC_CLOSED_LOOP_BRAKE)
    {
		pmsm_foc_linear_torque_ramp_generator(USER_IQ_REF_LOW_LIMIT,USER_IQ_RAMPUP, USER_IQ_RAMPDOWN, &FOCInput);
    	if (Motor.Speed < FOC_EXIT_SPEED)
    	{
    		Motor.State = MOTOR_STOP;
    	}
    }
	else
	{
		pmsm_foc_linear_torque_ramp_generator(Motor.Target_Torque,USER_IQ_RAMPUP, USER_IQ_RAMPDOWN, &FOCInput);
	}
#endif

#if(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
	if (Motor.State != FOC_CLOSED_LOOP_BRAKE && ( MOTOR_HOLD_THRESHOLD) && (FOCInput.Vq <= Motor.Target_Torque)  )
	{
		Motor.State = MOTOR_HOLD;
	}


    if (Motor.State == FOC_CLOSED_LOOP_BRAKE)
    {
		pmsm_foc_linear_vq_ramp_generator(USER_VQ_REF_LOW_LIMIT ,USER_VQ_RAMPUP, USER_VQ_RAMPDOWN, &FOCInput);
    	if (Motor.Speed < FOC_EXIT_SPEED)
    	{
    		Motor.State = MOTOR_STOP;
    	}
    }
	else
	{
		pmsm_foc_linear_vq_ramp_generator(Motor.Target_Voltage ,USER_VQ_RAMPUP, USER_VQ_RAMPDOWN, &FOCInput);
	}
#endif

#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
		if (Motor.State != FOC_CLOSED_LOOP_BRAKE && (MOTOR_HOLD_THRESHOLD) && (Motor.Ref_Speed <= Motor.Target_Speed))
		{
			Motor.State = MOTOR_HOLD;
		} else
		{

			if (Motor.Transition_Status == MOTOR_TRANSITION && Motor.State == FOC_CLOSED_LOOP)
			{
			  /* Motor in transition mode. Motor goes to higher speed first, important for startup. */
				pmsm_foc_linear_ramp_generator(MIN_STARTUP_SPEED_FOC,RAMP_RATE_INIT_FOC,RAMPDOWN_RATE,SPEEDRAMPSTEP,&Motor.Ref_Speed);
			}
			if (Motor.State == FOC_CLOSED_LOOP_BRAKE)
			{
				pmsm_foc_scurve_ramp_generator(0, RAMPUP_RATE, RAMPDOWN_RATE, SPEEDRAMPSTEP,&Motor.Ref_Speed);
				if ( Motor.Speed < FOC_EXIT_SPEED)
				{
					Motor.State = MOTOR_STOP;
				}
			}
			else
			{
				pmsm_foc_scurve_ramp_generator(Motor.Target_Speed, RAMPUP_RATE, RAMPDOWN_RATE, SPEEDRAMPSTEP,&Motor.Ref_Speed);
				pmsm_foc_adjust_foc_parameters ();   // Adjust parameters, e.g.: for PI controllers.
			}
		}
#endif


}


#define PI_SPEED_IK_LIMIT_FINAL		((((1<<15) * 6U) >> 3U) << PI_SPEED_SCALE_KPKI)
/* Step for parameter Ik change. */
#define SPEED_IK_ADJUST_STEP		(1<<14)

/** Adjust parameters, e.g.: for PI controllers, in FOC stable state
** Scheduling - using different parameters in different operating regions.
** Execution time: ?us (O3 - Optimize most).
 * ----------------------------------------------------------------------*/
__RAM_FUNC void pmsm_foc_adjust_foc_parameters (void)
{
     /* Parameter adjustment not finished yet. */
    if (Motor.Adjust_Para_Flag != ADJUST_DONE)
    {
      /* 1). Ik limit scheduling for Speed PI controller:*/
      if (PI_Speed.Ik_limit_max < PI_SPEED_IK_LIMIT_FINAL)
      {
        /* Parameter adjusted gradually and regularly every PWM cycle. */
        PI_Speed.Ik_limit_max += SPEED_IK_ADJUST_STEP;
        PI_Speed.Ik_limit_min = - PI_Speed.Ik_limit_max;
      }
      else
      {
        /* To indicate that adjustment of this parameter is done. */
        Motor.Adjust_Para_Flag = ADJUST_DONE;
      }
    }
}




/* Init parameters of LIB. Init once only */

void pmsm_foc_systemparameters_init_onceonly (void)
{

  /* Init below once only before go to FOC: */
  FOCInput.Phase_L = L_OMEGALI;
  FOCInput.Phase_L_Scale = SCALE_L;

  FOCInput.Res_Inc = RES_INC; /* Resolution increase, use (16 + Res_Inc) bit to represent 360 deg. */
  FOCInput.LPF_N_BEMF = SHIFT_MET_PLL;

  FOCInput.CCU8_Period = (uint32_t) PERIOD_REG;

  FOCInput.Ref_Id = 0;

  FOCInput.Vq_Flag = 0; /* FOC Vq from Iq PI controller. */

  FOCInput.Iq_PI_Flag = 0; /* Reference of Iq PI controller from speed PI output. */

  FOCInput.RotorSpeed_In = 0;

  FOCInput.SVM_5_Segment_Flag = 0; /* 7-segment SVM. For 3-shunt current sensing only. */

  FOCInput.Flag_State = 0;

    #if(OVERCURRENT_PROTECTION == ENABLED)
     FOCInput.overcurrent_factor = 4096;                /* */
    #endif
  PLL_Estimator.RotorSpeed_In = Motor.Speed;
  FOCInput.Ref_Speed = Motor.Speed; /* Motor reference speed. */

  PI_PLL.Ik = PLL_Estimator.RotorSpeed_In << PI_PLL.Scale_KpKi;

  FOCOutput.New_SVM_SectorNo = SVM.CurrentSectorNo;

  FOCOutput.Previous_SVM_SectorNo = 0;
  FOCOutput.New_SVM_SectorNo = 0; /* Use default SVM sector. */

} /* End of pmsm_foc_systemparameters_init_onceonly () */


void pmsm_foc_variables_init (void)
{
  Motor.Transition_Status = MOTOR_TRANSITION;				// Motor in transition mode.

  Motor.L_METPLL = L_OMEGALI;						// Motor inductance per phase
                                     //Using L_OMEGALI instead of Motor.L_METPLL in multiplication saves one MCU clock.

  Motor.Counter = 0;								// Init counters.
  Motor.Ramp_Counter = 0;
  Motor.Alignment_Counter = 0;
  Motor.Non_RealTime_Counter = 1;
  Motor.UART_Counter = 0;
  Motor.UART_Debug_Counter = 0;

  Motor.Speed = DEFAULT_SPEED_STARTUP;			// Init for V/f ramp-up.
  Motor.FG_Speed = Motor.Speed;					// Motor speed for Frequency Generation (FG) only.
  Motor.Ref_Speed = 0;


  Motor.Ramp_Up_Rate = RAMPUP_RATE << USER_RATIO_S;	// Slower ramp up and ramp down for S-curve profile.
  Motor.Ramp_Dn_Rate = RAMPDOWN_RATE << (USER_RATIO_S - 1);


  Motor.PWM_DutyCycle = 0;
  Motor.PWM_Speed_Raw = 0;
  Motor.PWM_Freq = 20;							// Init PWM frequency 20Hz.

//  Motor.CCU8_Trap_Status = 0x00;

  Car2Polar.SVM_Vref16 = 0;
  Car2Polar.SVM_Angle16 = (DEGREE_X >> 16U);		// Init Vref angle θ = X°.

  Car2Polar.Vref_AngleQ31 = Car2Polar.SVM_Angle16 << 16U;
  Car2Polar.Vref_AngleQ31_Previous = Car2Polar.Vref_AngleQ31;

  ADC.ADCTrig_Point = (uint32_t)(PERIOD_REG) >> 1;			// For ADC trigger for 2or3-shunt current sensing.

  ADC.ADC_DCLink = ADC_DCLINK_IDEAL;
  ADC.ADC_IDCLink = 0;

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  ADC.ADC3Trig_Point = 0;           /* For ADC 3 trigger, of single-shunt current sensing.*/
  ADC.ADC4Trig_Point = 0;           /* For ADC 4 trigger. */

  ADC.ADC_Result1 = 0;
  ADC.ADC_Result2 = 0;
  ADC.ADC_ResultTz1 = 0;
  ADC.ADC_ResultTz2 = 0;
  ADC.ADC_Result3 = 0;
  ADC.ADC_Result4 = 0;
  SVM.SVM_Flag = SVM_USE_PZV;        /* Init using SVM with Pseudo Zero Vectors (PZV). */
  ADC.Result_Flag = RESULTS_ADCTZ12;
#endif

  /* Init motor phase currents */
  Current.I_U = 0;
  Current.I_V = 0;
  Current.I_W = 0;

  SVM.PreviousSectorNo = 0;						// Init SVM sector No.

  SVM.Flag_3or2_ADC = USE_ALL_ADC;				// Init to use all (e.g.: three) ADC samplings for current reconstruction, for 2or3-shunt.

  pmsm_foc_pi_controller_init();							// Init parameters (Kp / Ki, limits) of PI controllers.

  pmsm_foc_systemparameters_init_onceonly();


}	// End of pmsm_foc_variables_init ()



