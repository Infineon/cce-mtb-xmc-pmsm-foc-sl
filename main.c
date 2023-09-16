/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the XMC13-14 PMSM FOC SL Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
 Copyright (c) 2015-2017, Infineon Technologies AG                            **
 All rights reserved.                                                         **
                                                                              **
 Redistribution and use in source and binary forms, with or without           **
 modification,are permitted provided that the following conditions are met:   **
                                                                              **
 *Redistributions of source code must retain the above copyright notice,      **
 this list of conditions and the following disclaimer.                        **
 *Redistributions in binary form must reproduce the above copyright notice,   **
 this list of conditions and the following disclaimer in the documentation    **
 and/or other materials provided with the distribution.                       **
 *Neither the name of the copyright holders nor the names of its contributors **
 may be used to endorse or promote products derived from this software without**
 specific prior written permission.                                           **
                                                                              **
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  **
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE    **
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE   **
 ARE  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE   **
 LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         **
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF         **
 SUBSTITUTE GOODS OR  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    **
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN      **
 CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)       **
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE   **
 POSSIBILITY OF SUCH DAMAGE.                                                  **
                                                                              **
 To improve the quality of the software, users are encouraged to share        **
 modifications, enhancements or bug fixes with Infineon Technologies AG       **
 dave@infineon.com).                                                          **
                                                                              **
********************************************************************************
**                                                                            **
**                                                                            **
** PLATFORM : Infineon XMC1300 Series                                         **
**                                                                            **
** AUTHOR : Motor Control  R&D Team                                           **
**                                                                            **
** version 1.5.0                                                                **
**                                                                            **
** Modified date: 2019-01-10                                                  **
**                                                                            **
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/

/* SFR declarations of the selected device */
#include "PMSM_FOC/Configuration/pmsm_foc_variables_scaling.h"
#include "PMSM_FOC/ControlModules/pmsm_foc_interface.h"


bool motor_request_start = false; //true = direct start; false = Micro Inspector Pro start (default)
bool motor_off = true;
float Vdc_link;
int32_t Motor_actual_speed;
int32_t Motor_target_set = 1000;


#if(uCPROBE_GUI_OSCILLOSCOPE == ENABLED)
#include "ProbeScope/probe_scope.h"
#endif


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
extern void pmsm_foc_motor_start(void);
extern void pmsm_foc_init (void);
#if defined ( __CC_ARM )
extern char Image$$RW_CODE$$Base ;
extern char Image$$RW_CODE$$Length ;
extern char Load$$RW_CODE$$Base ;

int load_ramcode( void )
{
  return (int)memcpy( &Image$$RW_CODE$$Base,
          &Load$$RW_CODE$$Base,
          ( size_t )&Image$$RW_CODE$$Length) ;
}
#endif


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* Initialization of MCU and motor control peripherals (CCU, Math unit, GPIOs, ADCs)
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
#if defined ( __CC_ARM )
  load_ramcode();
#endif

#if(uCPROBE_GUI_OSCILLOSCOPE == ENABLED)
  ProbeScope_Init(USER_CCU8_PWM_FREQ_HZ);
#endif


  /* Init MCU and motor control peripherals */
  pmsm_foc_init ();


#if (SETTING_TARGET_SPEED == BY_POT_ONLY)
  pmsm_foc_motor_start();
#endif


	while (1)
	 /* MCU main loop. Actually only require the processor to run when an interrupt occurs. */
	{

	}

	 return 0;
}
/* End of main () */


void pmsm_foc_secondaryloop_callback(){

  if (motor_request_start & motor_off)
  {
    pmsm_foc_motor_start();
    motor_off = false;
  }
  else if (!motor_request_start & !motor_off )
  {
    pmsm_foc_motor_brake();
    motor_off = true;
  }

  Vdc_link = pmsm_foc_get_Vdc_link();
  Motor_actual_speed = pmsm_foc_get_motor_speed();

#if((MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_ONLY) ||(MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == SPEED_CONTROLLED_DIRECT_FOC))
  pmsm_foc_set_motor_target_speed(Motor_target_set);
#elif(MY_FOC_CONTROL_SCHEME == TORQUE_CONTROLLED_DIRECT_FOC)
  pmsm_foc_set_motor_target_torque(Motor_target_set);
#elif(MY_FOC_CONTROL_SCHEME == VQ_CONTROLLED_DIRECT_FOC)
  void pmsm_foc_set_motor_target_voltage(Motor_target_set);
#endif

}

/* [] END OF FILE */
