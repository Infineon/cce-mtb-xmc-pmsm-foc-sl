/**
 * @file uart.h
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
 * @file uart.h
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

#ifndef PMSM_FOC_MCUINIT_UART_H_
#define PMSM_FOC_MCUINIT_UART_H_

/**
 * @addtogroup
 * @{
 */
#include "../ControlModules/pmsm_foc_functions.h"
#include "../Configuration/pmsm_foc_variables_scaling.h"
/**
 * @addtogroup
 * @{
 */

/*********************************************************************************************************************
 * MACROS
 ***************************************/
#if(UART_ENABLE == USIC0_CH1_P1_2_P1_3)
#define USIC0_CHX_IN0	(USIC0_CH1->IN[0])        /* UART uses USIC0_CH1 */
#define USIC0_CHX_TRBSR	(USIC0_CH1->TRBSR)
#define USIC0_CHX_OUTR	(USIC0_CH1->OUTR)
#elif(UART_ENABLE == USIC0_CH0_P1_4_P1_5)
#define USIC0_CHX_IN0 (USIC0_CH0->IN[0])        // UART uses USIC0_CH0
#define USIC0_CHX_TRBSR (USIC0_CH0->TRBSR)
#define USIC0_CHX_OUTR  (USIC0_CH0->OUTR)
#endif

#if(UART_ENABLE != USIC_DISABLED_ALL)
#define UART_FIFO_BUFFER  USIC0_CHX_IN0           /* USIC Transmit FIFO Buffer. */
#endif

#define DIFF_09_TO_ASCII  (0x30)                /* Difference between ASCII of an integer 0~9 and the integer. */
#define DIFF_AF_TO_ASCII  (0x37)                /* Difference between ASCII of an integer 0xA ~ 0xF and the integer. */
#define STRING_MAX_NO   (32)                /* Max length of a string (text). */
#define UART_FIFO_SIZE    (0x05UL)    /* FIFO buffer size 32, for UART Transmit/Receive Buffer */

#define RMD_FROM_DIV    1               /*
                                         * Read remainder from DIV MATH->RMD (0.2us faster code).
                                         * Comment out to calculate remainder by CPU.
                                         */

/* BAUD_RATE_xxxxxx = MCLK/( (BRG_PDIV+1) x (BRG_DCTQ+1) x (BRG_PCTQ+1) ) * (FDR_STEP/1024). */

#define FDR_STEP  590UL       /* UART baud rate constants for 460.8kbps @ MCLK=32MHz */
#define BRG_PDIV  3UL
#define BRG_DCTQ  9UL
#define BRG_PCTQ  0UL

#define UART_RATE   (3U)            /* To determine UART communication frequency. */
#define UART_SPEED_UPDATE_RATE    (UART_RATE * 30U)	/* To determine frequency of UART speed update (for debug only). */

/*********************************************************************************************************************
 * API Prototypes
 ***************************************/

void UART_TX_String(const char * String_Ptr);
/* INLINE_FUN void UART_TX_String (const char * String_Ptr); */
void UART_TX_Return_NewLine(uint16_t N_times);
void UART_TX_uint16_t(uint16_t Integer_16);
void UART_TX_int16_t(int16_t integer_16);
void UART_TX_uint16_t_Hex(uint16_t Int_16_Hex);
void UART_RX(void);
void pmsm_foc_uart_set_pot_adc(void);
void pmsm_foc_uart_init (void);
#endif /* PMSM_FOC_MCUINIT_UART_H_ */

/**
 * @}
 */

/**
 * @}
 */
