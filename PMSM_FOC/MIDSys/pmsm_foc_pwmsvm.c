/**
 * @file pmsm_foc_pwmsvm.c
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
 * @file pmsm_foc_pwmsvm.c
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

#include <xmc_common.h>              /* SFR declarations of the selected device */
#include "../ControlModules/pmsm_foc_functions.h"


/*********************************************************************************************************************
 * MACROS
 ***************************************/
#define DEGREE_60       (10922U)                   /* 60Â° angle (0 ~ 2^16 represent electrical angle 0Â° ~ 360Â°). */
#define RATIO_T0_111    (2U)                       /* = 2 for standard SVM. */
#define PERIOD_OF_PWM   ((uint16_t)PERIOD_REG + 1U) /* Period of a CCU8 PWM. */
#define RATIO_T0_111    (2U)                       /* = 2 for standard SVM.*/
#define CCU8_PERIOD_2ND  (PERIOD_REG - TZ_PZVX2-1)   // Second CCU83 period, for ADC trigger
#define SPEED_SS_THRESHOLD ((SPEED_HIGH_LIMIT * 3U) >> 2U)
/*********************************************************************************************************************
 * GLOBAL DATA
*********************************************************************************************************************/
SVMType SVM;		     						 /* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon.*/
extern const uint16_t Sinus60_tab[];	/* Sine LUT used for SVM calculations, array size 256 or 1024. */
extern ADCType ADC;
extern MotorControlType Motor;
/*********************************************************************************************************************
 * API IMPLEMENTATION
 ***************************************/

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* API to update SVPWM CCU8 duty cycles, use Pseudo Zero Vectors PWM) */
__RAM_FUNC void pmsm_foc_svpwm_update(uint16_t Amplitude, uint32_t Angle)
{
  /* SVM time T1, T2. */
  uint16_t T1;
  uint16_t T2;

  /* Time (T1+T2+2Tz) */
  uint16_t T1T2n2Tz;

  /* Time of zero vector [000], i.e.: first portion of T0, and Ts - T0_111. */
  uint16_t T0_000;
  uint16_t Period_T0_111;

  uint16_t AngleTemp;
  uint16_t SectorAngle;

  /* 4 segment SVM PWM variables */
  uint16_t T1nT2;
  uint16_t HalfT0;

  SVM.PreviousSectorNo = SVM.CurrentSectorNo;           /* Record sector information of last PWM cycle. */

  /* Angle: 0 ~ 2^16 represent electrical angle 0° ~ 360°. Or = (uint16_t)(Angle >> 8) * 6; */
  AngleTemp = (Angle >> ANGLETEMP_SHIFT) * 6U;
  SectorAngle = AngleTemp & SECTOR_ANGLE_AND;         /* Relative angle θrel in each sector. */
  SVM.CurrentSectorNo = AngleTemp >> SECTOR_NO_SHIFT; /* Update new SVM sector number. Or = (AngleTemp >> 8) & 0x7U; */

  /* Calculate T1 / T2 by LUT. */
  T1 = (((Amplitude * Sinus60_tab[MAX_LUT_INDEX - SectorAngle]) >> 15) * SVM_LUT_SCALE) >> 15;
  T2 = (((Amplitude * Sinus60_tab[SectorAngle]) >> 15) * SVM_LUT_SCALE) >> 15;

  if ((Motor.Speed < SPEED_SS_THRESHOLD) || (T1 < TZ_PZV ) || (T2 < TZ_PZV ))
  {
    SVM.SVM_Flag = SVM_USE_PZV;
    CCU80_CC83->PRS = TZ_PZVX2; /* First CCU83 period is a constant, e.g.: 2Tz. */
    CCU80_CC83->CR1S = TRIGGER_POINT; /* For ADCTz1 trigger. */
    CCU80_CC83->CR2S = TZ_PZV + TRIGGER_POINT; /* For ADCTz2 trigger. */

    T1 += TZ_PZV; /* Add Tz, the time of Pseudo Zero Vectors. */
    T2 += TZ_PZV;

    T1T2n2Tz = T1 + T2 + TZ_PZVX2;
    if (T1T2n2Tz > PERIOD_REG)
    {
      T1T2n2Tz = PERIOD_REG; /* Make sure below T0/2 >= 0 and Ts-T0/2 <= Ts. */
    }

    T0_000 = (PERIOD_REG - T1T2n2Tz) >> 1U; /* T0_000 = T0/2. */
    Period_T0_111 = (PERIOD_REG + T1T2n2Tz) >> 1U; /* Temp variable for Ts-T0/2. */

    /* To use 6-segment (with two Tz at beginning) switching sequences of SVM with Pseudo Zero Vectors (PZV)*/
    switch (SVM.CurrentSectorNo)
    {
      case 0: /* Sector A */
        CCU8_MODULE_PHASE_U->CR1S = 0;
        CCU8_MODULE_PHASE_U->CR2S = TZ_PZVX2 + T0_000;
        CCU8_MODULE_PHASE_V->CR1S = TZ_PZV;
        CCU8_MODULE_PHASE_V->CR2S = Period_T0_111 - T2;
        CCU8_MODULE_PHASE_W->CR1S = TZ_PZVX2;
        CCU8_MODULE_PHASE_W->CR2S = Period_T0_111;

        /* To trigger ADC at centre of active segments T1 and T2. */
        ADC.ADC3Trig_Point = T0_000 + (T1 >> 1);    // For ADC 3 trigger.
      //ADC.ADC4Trig_Point = T0_000 + T1 + (T2 >> 1); // For ADC 4 trigger.
        ADC.ADC4Trig_Point = CCU8_PERIOD_2ND - (T0_000) - (T2 >> 1); // For ADC 4 trigger.
        break;

      case 1: /*Sector B */
        CCU8_MODULE_PHASE_U->CR1S = TZ_PZV;
        CCU8_MODULE_PHASE_U->CR2S = Period_T0_111 - T1;
        CCU8_MODULE_PHASE_V->CR1S = 0;
        CCU8_MODULE_PHASE_V->CR2S = TZ_PZVX2 + T0_000;
        CCU8_MODULE_PHASE_W->CR1S = TZ_PZVX2;
        CCU8_MODULE_PHASE_W->CR2S = Period_T0_111;

        /* To trigger ADC at centre of active segments T1 and T2. */
        ADC.ADC3Trig_Point = T0_000 + (T2 >> 1);
        //ADC.ADC4Trig_Point = T0_000 + T2 + (T1 >> 1);
        ADC.ADC4Trig_Point = CCU8_PERIOD_2ND - (T0_000) - (T1 >> 1); // For ADC 4 trigger.
        break;

      case 2: /* Sector C */
        CCU8_MODULE_PHASE_U->CR1S = TZ_PZVX2;
        CCU8_MODULE_PHASE_U->CR2S = Period_T0_111;
        CCU8_MODULE_PHASE_V->CR1S = 0;
        CCU8_MODULE_PHASE_V->CR2S = TZ_PZVX2 + T0_000;
        CCU8_MODULE_PHASE_W->CR1S = TZ_PZV;
        CCU8_MODULE_PHASE_W->CR2S = Period_T0_111 - T2;

        /* To trigger ADC at centre of active segments T1 and T2. */
        ADC.ADC3Trig_Point = T0_000 + (T1 >> 1);
        //ADC.ADC4Trig_Point = T0_000 + T1 + (T2 >> 1);
        ADC.ADC4Trig_Point = CCU8_PERIOD_2ND - (T0_000) - (T2 >> 1); // For ADC 4 trigger.
        break;

      case 3: /* Sector D */
        CCU8_MODULE_PHASE_U->CR1S = TZ_PZVX2;
        CCU8_MODULE_PHASE_U->CR2S = Period_T0_111;
        CCU8_MODULE_PHASE_V->CR1S = TZ_PZV;
        CCU8_MODULE_PHASE_V->CR2S = Period_T0_111 - T1;
        CCU8_MODULE_PHASE_W->CR1S = 0;
        CCU8_MODULE_PHASE_W->CR2S = TZ_PZVX2 + T0_000;

        /* To trigger ADC at centre of active segments T1 and T2. */
        ADC.ADC3Trig_Point = T0_000 + (T2 >> 1);
//        ADC.ADC4Trig_Point = T0_000 + T2 + (T1 >> 1);
        ADC.ADC4Trig_Point = CCU8_PERIOD_2ND - (T0_000) - (T1 >> 1); // For ADC 4 trigger.

        break;

      case 4: /* Sector E */
        CCU8_MODULE_PHASE_U->CR1S = TZ_PZV;
        CCU8_MODULE_PHASE_U->CR2S = Period_T0_111 - T2;
        CCU8_MODULE_PHASE_V->CR1S = TZ_PZVX2;
        CCU8_MODULE_PHASE_V->CR2S = Period_T0_111;
        CCU8_MODULE_PHASE_W->CR1S = 0;
        CCU8_MODULE_PHASE_W->CR2S = TZ_PZVX2 + T0_000;

        /* To trigger ADC at centre of active segments T1 and T2. */
        ADC.ADC3Trig_Point = T0_000 + (T1 >> 1);
        //ADC.ADC4Trig_Point = T0_000 + T1 + (T2 >> 1);
        ADC.ADC4Trig_Point = CCU8_PERIOD_2ND - (T0_000) - (T2 >> 1); // For ADC 4 trigger.
        break;

      default: /* Process for all other cases, Sector F = 5. */
        CCU8_MODULE_PHASE_U->CR1S = 0;
        CCU8_MODULE_PHASE_U->CR2S = TZ_PZVX2 + T0_000;
        CCU8_MODULE_PHASE_V->CR1S = TZ_PZVX2;
        CCU8_MODULE_PHASE_V->CR2S = Period_T0_111;
        CCU8_MODULE_PHASE_W->CR1S = TZ_PZV;
        CCU8_MODULE_PHASE_W->CR2S = Period_T0_111 - T1;

        /* To trigger ADC at centre of active segments T1 and T2. */
        ADC.ADC3Trig_Point = T0_000 + (T2 >> 1);
        //ADC.ADC4Trig_Point = T0_000 + T2 + (T1 >> 1);
        ADC.ADC4Trig_Point = CCU8_PERIOD_2ND - (T0_000) - (T1 >> 1); // For ADC 4 trigger.
        break;
    }
  }
  else
  {
    SVM.SVM_Flag = SVM_USE_STANDARD;
    CCU80_CC83->PRS = PERIOD_REG;

    T1nT2 = T1 + T2; /* Temp variable for (T1+T2) <= PERIOD_REG. */
    if (T1nT2 > PERIOD_REG)
    {
      T1nT2 = PERIOD_REG; /* Make sure below T0/2 >= 0. */
    }

    HalfT0 = (PERIOD_REG - T1nT2) >> 1U;

    /* Standard 4-segment PWM: */
    switch (SVM.CurrentSectorNo)
    {
      case 0: /* Sector A */
        CCU8_MODULE_PHASE_U->CR1S = 0U;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) HalfT0;

        CCU8_MODULE_PHASE_V->CR1S = 0U;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) HalfT0 + T1;

        CCU8_MODULE_PHASE_W->CR1S = 0U;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM + T1nT2) >> 1U;

        CCU80_CC83->CR1S = HalfT0 + (T1 >> 1);      // For ADC trigger at centre of active segments T1 and T2.
        CCU80_CC83->CR2S = HalfT0 + T1 + (T2 >> 1);
        break;
      case 1: /* Sector B */
        CCU8_MODULE_PHASE_U->CR1S = 0U;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) HalfT0 + T2;

        CCU8_MODULE_PHASE_V->CR1S = 0U;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) HalfT0;

        CCU8_MODULE_PHASE_W->CR1S = 0U;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM + T1nT2) >> 1U;

        CCU80_CC83->CR1S = HalfT0 + (T2 >> 1);      // For ADC trigger at centre of active segments T1 and T2.
        CCU80_CC83->CR2S = HalfT0 + T2 + (T1 >> 1);

        break;
      case 2: /* Sector C */
        CCU8_MODULE_PHASE_U->CR1S = 0U;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM + T1nT2) >> 1U;

        CCU8_MODULE_PHASE_V->CR1S = 0U;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) HalfT0;

        CCU8_MODULE_PHASE_W->CR1S = 0U;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) HalfT0 + T1;

        CCU80_CC83->CR1S = HalfT0 + (T1 >> 1);      // For ADC trigger at centre of active segments T1 and T2.
        CCU80_CC83->CR2S = HalfT0 + T1 + (T2 >> 1);

        break;
      case 3: /* Sector D */
        CCU8_MODULE_PHASE_U->CR1S = 0U;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM + T1nT2) >> 1U;

        CCU8_MODULE_PHASE_V->CR1S = 0U;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) HalfT0 + T2;

        CCU8_MODULE_PHASE_W->CR1S = 0U;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) HalfT0;

        CCU80_CC83->CR1S = HalfT0 + (T2 >> 1);      // For ADC trigger at centre of active segments T1 and T2.
        CCU80_CC83->CR2S = HalfT0 + T2 + (T1 >> 1);

        break;
      case 4: /* Sector E */
        CCU8_MODULE_PHASE_U->CR1S = 0U;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) HalfT0 + T1;

        CCU8_MODULE_PHASE_V->CR1S = 0U;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM + T1nT2) >> 1U;

        CCU8_MODULE_PHASE_W->CR1S = 0U;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) HalfT0;

        CCU80_CC83->CR1S = HalfT0 + (T1 >> 1);      // For ADC trigger at centre of active segments T1 and T2.
        CCU80_CC83->CR2S = HalfT0 + T1 + (T2 >> 1);

        break;
      default: /* Process for all other cases, Sector F = 5. */
        CCU8_MODULE_PHASE_U->CR1S = 0U;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) HalfT0;

        CCU8_MODULE_PHASE_V->CR1S = 0U;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM + T1nT2) >> 1U;

        CCU8_MODULE_PHASE_W->CR1S = 0U;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) HalfT0 + T2;

        CCU80_CC83->CR1S = HalfT0 + (T2 >> 1);      // For ADC trigger at centre of active segments T1 and T2.
        CCU80_CC83->CR2S = HalfT0 + T2 + (T1 >> 1);

        break;
    } /* End of switch (SVM.CurrentSectorNo), standard 7-segment symmetric PWM. */

  }
  /* Enable shadow transfer for slice 0,1,2 for CCU80 Kernel. */
  CCU8_MODULE->GCSS |= (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0|XMC_CCU8_SHADOW_TRANSFER_SLICE_1|XMC_CCU8_SHADOW_TRANSFER_SLICE_2|XMC_CCU8_SHADOW_TRANSFER_SLICE_3 );
 }
#else
/* API to update SVPWM CCU8 duty cycles, use standard SVM (7-segment symmetric PWM) */
__RAM_FUNC void pmsm_foc_svpwm_update(uint16_t Amplitude, uint32_t Angle)
{
  /* SVM time T1, T2. */
  uint16_t T1;
  uint16_t T2;
  uint16_t T1nT2;               /* Time (T1+T2). */
  uint16_t T0;

  uint16_t AngleTemp;
  uint16_t SectorAngle;

  SVM.PreviousSectorNo = SVM.CurrentSectorNo;           /* Record sector information of last PWM cycle. */

  /* Angle: 0 ~ 2^16 represent electrical angle 0° ~ 360°. Or = (uint16_t)(Angle >> 8) * 6; */

  AngleTemp = (Angle >> ANGLETEMP_SHIFT) * 6U;
  SectorAngle = AngleTemp & SECTOR_ANGLE_AND;					/* Relative angle θrel in each sector. */
  SVM.CurrentSectorNo = AngleTemp >> SECTOR_NO_SHIFT; /* Update new SVM sector number. Or = (AngleTemp >> 8) & 0x7U; */

  /* Calculate T1 / T2 by LUT. */
  T1 = (((Amplitude * Sinus60_tab[MAX_LUT_INDEX - SectorAngle]) >> 15) * SVM_LUT_SCALE) >> 15;
  T2 = (((Amplitude * Sinus60_tab[SectorAngle]) >> 15) * SVM_LUT_SCALE) >> 15;

  T1nT2 = T1 + T2;												/* Temp variable for (T1+T2) <= PERIOD_REG. */


//  /* Overmodulation from Deyun */
//  if (T1nT2 > PERIOD_REG)
//  {
//    T1nT2 = PERIOD_REG;       /* Make sure below T0/2 >= 0. */
//  }

  if (T1nT2 > PERIOD_OF_PWM)
  {
    #define SHIFT_OVERMODULATION    (5U)
    MATH->DIVCON = (0x00008004 | (SHIFT_OVERMODULATION << 16U) | (SHIFT_OVERMODULATION << 8U));
    MATH->DVD = T1 * PERIOD_OF_PWM;
    MATH->DVS = T1nT2;

    T1nT2 = PERIOD_OF_PWM;

    while (MATH->DIVST)
    {
      /* CPU wait */
    }

    T1 = MATH->QUOT;
    T2 = PERIOD_OF_PWM - T1;

  }

  T0 = PERIOD_OF_PWM - T1nT2;

#if(CURRENT_SENSING == USER_THREE_SHUNT_SYNC_CONV)
  if (T0 > T0_THRESHOLD)
  {
    SVM.Flag_3or2_ADC = USE_ALL_ADC;			/* To use all (e.g.: three) ADC samplings for current reconstruction. */
  }
  else
  {
    SVM.Flag_3or2_ADC = USE_2_ADC;				/* To use two ADC samplings for current reconstruction. */
  }
#endif

#if(SVM_SWITCHING_SCHEME == STANDARD_SVM_7_SEGMENT)
  /* 7-segment SVM, T0, T0_111 for first [111], T0_111 + T1/2, T0_111 + T2/2, T0_111 + (T1+T2)/2. */
  uint16_t T0_111;
  uint16_t T0nHalfT1;
  uint16_t T0nHalfT2;
  uint16_t T0nHalfT1nT2;

  T0_111 = T0 >> RATIO_T0_111;												/* T0_111, time of first [111]. */
  T0nHalfT1 = (T0 + (uint16_t)(T1 << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;				/* T0_111 + T1/2. */
  T0nHalfT2 = (T0 + (uint16_t)(T2 << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;				/* T0_111 + T2/2. */
  T0nHalfT1nT2 = (T0 + (uint16_t)((T1 + T2) << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;	/* T0_111 + (T1+T2)/2. */

  /* Standard 7-segment symmetric PWM: */
  switch (SVM.CurrentSectorNo)
  {
    case 0:														/* Sector A */
      CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0nHalfT1nT2;
      CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

      CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0nHalfT2;
      CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT2);

      CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0_111;
      CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

      break;
    case 1:														/* Sector B */

      CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0nHalfT1;
      CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1);

      CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0nHalfT1nT2;
      CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

      CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0_111;
      CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

      break;
    case 2:														/* Sector C */
      CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0_111;
      CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

      CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0nHalfT1nT2;
      CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

      CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0nHalfT2;
      CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT2);

      break;
    case 3:														/* Sector D */

      CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0_111;
      CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

      CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0nHalfT1;
      CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1);

      CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0nHalfT1nT2;
      CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

      break;
    case 4:														/* Sector E */
      CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0nHalfT2;
      CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT2);

      CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0_111;
      CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

      CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0nHalfT1nT2;
      CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

      break;
    default:													/* Process for all other cases, Sector F = 5. */
      CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0nHalfT1nT2;
      CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

      CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0_111;
      CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

      CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0nHalfT1;
      CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1);


      break;
  }	/* End of switch (SVM.CurrentSectorNo), standard 7-segment symmetric PWM. */


#elif(SVM_SWITCHING_SCHEME == STANDARD_SVM_5_SEGMENT)
      /* 5-segment SVM, (T1+T2)/2, T2/2, T1/2 */
      uint16_t Half_T1nT2;
      uint16_t Half_T2;
      uint16_t Half_T1;

      Half_T1nT2 = T1nT2 >> 1;                // (T1+T2)/2.
      Half_T2 = T2 >> 1;                    // T2/2.
      Half_T1 = T1 >> 1;                    // T1/2.

      /* Standard 5-segment symmetric PWM: */
      switch (SVM.CurrentSectorNo)
      {
        case 0:                        // Sector A
          CCU8_MODULE_PHASE_U->CR1S = (uint32_t) Half_T1nT2;
          CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T1nT2);

          CCU8_MODULE_PHASE_V->CR1S = (uint32_t) Half_T2;
          CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T2);

          CCU8_MODULE_PHASE_W->CR1S = (uint32_t) 0;
          CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM + 1);

        break;

      case 1:                       // Sector B
        CCU8_MODULE_PHASE_U->CR1S = (uint32_t) Half_T1;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T1);

        CCU8_MODULE_PHASE_V->CR1S = (uint32_t) Half_T1nT2;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T1nT2);

        CCU8_MODULE_PHASE_W->CR1S = (uint32_t) 0;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM + 1);

        break;

      case 2:                       // Sector C
        CCU8_MODULE_PHASE_U->CR1S = (uint32_t) 0;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM + 1);

        CCU8_MODULE_PHASE_V->CR1S = (uint32_t) Half_T1nT2;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T1nT2);

        CCU8_MODULE_PHASE_W->CR1S = (uint32_t) Half_T2;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T2);

        break;

      case 3:                       // Sector D

        CCU8_MODULE_PHASE_U->CR1S = (uint32_t) 0;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM + 1);

        CCU8_MODULE_PHASE_V->CR1S = (uint32_t) Half_T1;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T1);

        CCU8_MODULE_PHASE_W->CR1S = (uint32_t) Half_T1nT2;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T1nT2);

        break;

      case 4:                       // Sector E

        CCU8_MODULE_PHASE_U->CR1S = (uint32_t) Half_T2;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T2);

        CCU8_MODULE_PHASE_V->CR1S = (uint32_t) 0;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM + 1);

        CCU8_MODULE_PHASE_W->CR1S = (uint32_t) Half_T1nT2;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T1nT2);

        break;

      default:                      // Process for all other cases. Sector F.

        CCU8_MODULE_PHASE_U->CR1S = (uint32_t) Half_T1nT2;
        CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T1nT2);

        CCU8_MODULE_PHASE_V->CR1S = (uint32_t) 0;
        CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM + 1);

        CCU8_MODULE_PHASE_W->CR1S = (uint32_t) Half_T1;
        CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - Half_T1);

        break;
      }

#endif
  /* Enable shadow transfer for slice 0,1,2 for CCU80 Kernel. */
  CCU8_MODULE->GCSS |= (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0|XMC_CCU8_SHADOW_TRANSFER_SLICE_1|XMC_CCU8_SHADOW_TRANSFER_SLICE_2);
  /* ADC triggered always at centre of [000] (fixed centre position in one PWM). */
}
#endif
