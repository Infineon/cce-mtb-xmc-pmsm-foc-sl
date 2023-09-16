/**
 * @file mcuinit.c
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
 * @file mcuinit.c
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
#include "mcuinit.h"

#define PMSM_FOC_SETTLING_TIME    0x7FFFF

/* Motor control information */
extern MotorControlType Motor;




#if(CATCH_FREE_RUNNING == ENABLED)
extern CFR_type CFR;
#endif


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ***************************************/

/* API to initialize MCU and peripherals for motor control */
void pmsm_foc_init(void)
{

  volatile uint32_t delay_counter;

  /* Reset configuration, clock configuration */
  pmsm_foc_reset_clock_init();

  /* Hardware Settling down Timing*/
  for(delay_counter = 0; delay_counter < PMSM_FOC_SETTLING_TIME; delay_counter++);

  /* Init CCU8 */
  pmsm_foc_ccu8_init();

  /* Init CCU4 for debug, PWM speed adjustment, or FG / RD */
  pmsm_foc_ccu4_init();

  #if(SETTING_TARGET_SPEED == BY_UART_ONLY)
  /* Init UART */
  pmsm_foc_uart_init();
  #endif
  /* Init MATH Unit (i.e.: CORDIC Coprocessor and Divider Unit DIV) */
  pmsm_foc_math_init();

  /* Init GPIOs */
  pmsm_foc_gpio_Init();

  /*  Init variables for motor control. Before start motor, brake the motor in case it is running */
  pmsm_motorcontrol_init();

  /* Init ADC, for current sensing, ADC of DC link Vdc (and POT). Do at later stage of the init */
  pmsm_adc_module_init();



  #if(WATCH_DOG_TIMER == ENABLED)
  /* Init WDT */
  pmsm_foc_wdt_init();
  #endif

  /* Synchronous start of CAPCOM modules, e.g.: CCU8x, and or CCU4x */
  pmsm_foc_CCUx_SynStart();

#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
  UART_TX_String("\r\nInfineon FOC\r\n");
#endif

}



/*********************************************************************************************************************
 * API IMPLEMENTATION
 ***************************************/
/*###* Init for motor control ####
   * ---------------------------*/
void pmsm_motorcontrol_init (void)
{
    Motor.State = MOTOR_IDLE; //boostrap is called by motor start API
    //Motor.State = EN_INVERTER_BOOTSTRAP;
                                  /*
                                   * First brake the motor before motor startup.
                                   * Charge gate driver bootstrap capacitors (if any).
                                   */

#if(CATCH_FREE_RUNNING == ENABLED)
    Motor.State = MOTOR_COASTING;
#endif
    Motor.Rotation_Dir = DIRECTION_INC; /* Motor rotation direction - rotor angle increasing. */
    pmsm_foc_variables_init ();                /* Init variables. */

    XMC_GPIO_ToggleOutput(TEST_PIN);

} /* End of pmsm_motorcontrol_init () */


/** To init variables for catch free-running motor
  * -----------------------------------------------------*/
void pmsm_foc_init_variables_cfr_motor(void)
{

#if(CATCH_FREE_RUNNING == ENABLED)
    CFR.motor_phase = CFR_PHASE_U;
    CFR.flag = NO_CATCH_FREE;
    CFR.counter1 = 0;
    CFR.counter2 = 0;
    CFR.counter3 = 0;
    CFR.counter4 = 0;
#endif

    ADC.BEMF_U = 0;
    ADC.BEMF_V = 0;
    ADC.BEMF_W = 0;
    ADC.BEMF_Max = 0;
    ADC.BEMF_UV_Threshold = 0;

    pmsm_foc_get_current_bias();
//    /* Init ADC bias */
//    ADC.ADC_Bias = IU_ADC_BIAS;
//
//    ADC.ADC_Bias_Iu = IU_ADC_BIAS;
//    ADC.ADC_Bias_Iv = IV_ADC_BIAS;
//    ADC.ADC_Bias_Iw = IW_ADC_BIAS;
}

/* API to enable synchronous start of CAPCOM modules. */
void pmsm_foc_CCUx_SynStart(void)
{
	/* Setup Event0 for external start trigger */
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_V, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_W, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_ADC_TR, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
	XMC_CCU4_SLICE_StartConfig(SECONDARY_LOOP_SLICE, XMC_CCU4_SLICE_EVENT_0, XMC_CCU4_SLICE_START_MODE_TIMER_START_CLEAR);

	/* Enable Global Start Control CCU80 */
	XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk | SCU_GENERAL_CCUCON_GSC40_Msk);


	/* Disable external start trigger */
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_EVENT_NONE, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_V, XMC_CCU8_SLICE_EVENT_NONE, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_W, XMC_CCU8_SLICE_EVENT_NONE, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_ADC_TR, XMC_CCU8_SLICE_EVENT_NONE, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU4_SLICE_StartConfig(SECONDARY_LOOP_SLICE, XMC_CCU4_SLICE_EVENT_NONE, XMC_CCU4_SLICE_START_MODE_TIMER_START);

	/* Disable Global Start Control CCU80 */
	XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk | SCU_GENERAL_CCUCON_GSC40_Msk);

}



