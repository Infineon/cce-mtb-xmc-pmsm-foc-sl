/**
 * @file uart.c
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
 * @file uart.c
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
#include "uart.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ***************************************/

extern MotorControlType Motor; /* Motor control information */

/**
  * @brief	Initialize USIC Module
  *
  * @param  None
  * @retval None
  */
#define XMC_SCU_GCU_PASSWD_PROT_DISABLE (0x000000C0UL)  /*
                                                         * Password for disabling protection.
                                                         * Access to protected bits allowed.
                                                         */
#define XMC_SCU_GCU_PASSWD_PROT_ENABLE  (0x000000C3UL)  /* Password for enabling protection. */

#if(UART_ENABLE == USIC0_CH1_P1_2_P1_3)
/**
 * @brief	Initialize USIC Module
 *
 * @param  None
 * @retval None
 */
void pmsm_foc_uart_init (void)
{
  /* Disable clock gating to USIC0: */
  SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_DISABLE;

  /* Stop gating USIC0 */
  SCU_CLK->CGATCLR0 = 0x00000008UL;

  /* Wait if VDDC is too low, for VDDC to stabilise */
  while (SCU_CLK->CLKCR & 0x40000000UL)
  {
    continue;
  }

  /* Enable bit protection */
  SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_ENABLE;

  /* Enable the module kernel clock and the module functionality: */
  USIC0_CH1->KSCFG |= USIC_CH_KSCFG_MODEN_Msk | USIC_CH_KSCFG_BPMODEN_Msk;

  /* fFD = fPB. */
  /* FDR.DM = 02b (Fractional divider mode). */
  USIC0_CH1->FDR &= ~(USIC_CH_FDR_DM_Msk | USIC_CH_FDR_STEP_Msk);
  USIC0_CH1->FDR |= (0x02UL << USIC_CH_FDR_DM_Pos) | (FDR_STEP << USIC_CH_FDR_STEP_Pos);

  /* Configure baud rate generator: */
  /* BAUDRATE = fCTQIN/(BRG.PCTQ x BRG.DCTQ). */
  /* CLKSEL = 0 (fPIN = fFD), CTQSEL = 00b (fCTQIN = fPDIV), PPPEN = 0 (fPPP=fPIN). */
  USIC0_CH1->BRG &= ~(USIC_CH_BRG_PCTQ_Msk | USIC_CH_BRG_DCTQ_Msk | USIC_CH_BRG_PDIV_Msk | USIC_CH_BRG_CLKSEL_Msk |
                    USIC_CH_BRG_PPPEN_Msk);
  USIC0_CH1->BRG |= (BRG_PCTQ << USIC_CH_BRG_PCTQ_Pos) | (BRG_DCTQ << USIC_CH_BRG_DCTQ_Pos) |
                    (BRG_PDIV << USIC_CH_BRG_PDIV_Pos);

  /* Configuration of USIC Shift Control: */
  /* SCTR.FLE = 8 (Frame Length). */
  /* SCTR.WLE = 8 (Word Length). */
  /* SCTR.TRM = 1 (Transmission Mode). */
  /*
   * SCTR.PDL = 1 (This bit defines the output level at the shift data output signal when no data is available
   * for transmission).
   */
  USIC0_CH1->SCTR &= ~(USIC_CH_SCTR_TRM_Msk | USIC_CH_SCTR_FLE_Msk | USIC_CH_SCTR_WLE_Msk);
  USIC0_CH1->SCTR |= USIC_CH_SCTR_PDL_Msk | (0x01UL << USIC_CH_SCTR_TRM_Pos) | (0x07UL << USIC_CH_SCTR_FLE_Pos) |
                     (0x07UL << USIC_CH_SCTR_WLE_Pos);

  /* Configuration of USIC Transmit Control/Status Register: */
  /* TBUF.TDEN = 1 (TBUF Data Enable: A transmission of the data word in TBUF can be started if TDV = 1. */
  /*
   * TBUF.TDSSM = 1 (Data Single Shot Mode: allow word-by-word data transmission which avoid sending the same data
   * several times.
   */
  USIC0_CH1->TCSR &= ~(USIC_CH_TCSR_TDEN_Msk);
  USIC0_CH1->TCSR |= USIC_CH_TCSR_TDSSM_Msk | (0x01UL << USIC_CH_TCSR_TDEN_Pos);

  /* Configuration of Protocol Control Register: */
  /* PCR.SMD = 1 (Sample Mode based on majority). */
  /* PCR.STPB = 0 (1x Stop bit). */
  /* PCR.SP = 5 (Sample Point). */
  /* PCR.PL = 0 (Pulse Length is equal to the bit length). */
  USIC0_CH1->PCR &= ~(USIC_CH_PCR_ASCMode_STPB_Msk | USIC_CH_PCR_ASCMode_SP_Msk | USIC_CH_PCR_ASCMode_PL_Msk);
  USIC0_CH1->PCR |= USIC_CH_PCR_ASCMode_SMD_Msk | (9 << USIC_CH_PCR_ASCMode_SP_Pos);

  /* Configure Transmit Buffer: */
  /* Standard transmit buffer event is enabled. */
  /* Define start entry of Transmit Data FIFO buffer DPTR = 0. */
  USIC0_CH1->TBCTR &= ~(USIC_CH_TBCTR_SIZE_Msk | USIC_CH_TBCTR_DPTR_Msk);

  /* Set Transmit Data Buffer size and set data pointer to position 0. */
  USIC0_CH1->TBCTR |= ((UART_FIFO_SIZE << USIC_CH_TBCTR_SIZE_Pos) | (0x00 << USIC_CH_TBCTR_DPTR_Pos));

  /* Init UART_RX (P1.3 --> DX0A, or P2.11 --> DX0E): */
  XMC_GPIO_SetMode(P1_3, XMC_GPIO_MODE_INPUT_TRISTATE);
  USIC0_CH1->DX0CR = 0x00000000; /* USIC0_CH1.DX0A <-- P1.3. */

  /* Configure Receive Buffer: */
  /* Standard Receive buffer event is enabled. */
  /* Define start entry of Receive Data FIFO buffer DPTR. */
  USIC0_CH1->RBCTR &= ~(USIC_CH_RBCTR_SIZE_Msk | USIC_CH_RBCTR_DPTR_Msk);

  /* Set Receive Data Buffer Size and set data pointer to position max. */
  USIC0_CH1->RBCTR |= ((UART_FIFO_SIZE << USIC_CH_RBCTR_SIZE_Pos) | ((1 << UART_FIFO_SIZE) << USIC_CH_RBCTR_DPTR_Pos));

  /* Init UART_TX (P1.2 --> DOUT): */
  XMC_GPIO_SetMode(P1_2, XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7);

  /* Configuration of Channel Control Register: */
  /* CCR.PM = 00 ( Disable parity generation). */
  /* CCR.MODE = 2 (ASC mode enabled). */
  USIC0_CH1->CCR &= ~(USIC_CH_CCR_PM_Msk | USIC_CH_CCR_MODE_Msk);
  USIC0_CH1->CCR |= 0x02UL << USIC_CH_CCR_MODE_Pos;

}

#elif(UART_ENABLE == USIC0_CH0_P1_4_P1_5)
void pmsm_foc_uart_init (void)
{
  /* Disable clock gating to USIC0: */
  SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_DISABLE;

  /* Stop gating USIC0 */
  SCU_CLK->CGATCLR0 = 0x00000008UL;

  /* Wait if VDDC is too low, for VDDC to stabilise */
  while (SCU_CLK->CLKCR & 0x40000000UL) continue;

  /* Enable bit protection */
  SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_ENABLE;

  /* Enable the module kernel clock and the module functionality: */
  USIC0_CH0->KSCFG |= USIC_CH_KSCFG_MODEN_Msk | USIC_CH_KSCFG_BPMODEN_Msk;

  // fFD = fPB.
  // FDR.DM = 02b (Fractional divider mode).
  USIC0_CH0->FDR &= ~(USIC_CH_FDR_DM_Msk | USIC_CH_FDR_STEP_Msk);
  USIC0_CH0->FDR |= (0x02UL << USIC_CH_FDR_DM_Pos) | (FDR_STEP << USIC_CH_FDR_STEP_Pos);

  // Configure baud rate generator:
  // BAUDRATE = fCTQIN/(BRG.PCTQ x BRG.DCTQ).
  // CLKSEL = 0 (fPIN = fFD), CTQSEL = 00b (fCTQIN = fPDIV), PPPEN = 0 (fPPP=fPIN).
  USIC0_CH0->BRG &= ~(USIC_CH_BRG_PCTQ_Msk | USIC_CH_BRG_DCTQ_Msk | USIC_CH_BRG_PDIV_Msk | USIC_CH_BRG_CLKSEL_Msk | USIC_CH_BRG_PPPEN_Msk);
  USIC0_CH0->BRG |= (BRG_PCTQ << USIC_CH_BRG_PCTQ_Pos) | (BRG_DCTQ << USIC_CH_BRG_DCTQ_Pos) | (BRG_PDIV << USIC_CH_BRG_PDIV_Pos);

  // Configuration of USIC Shift Control:
  // SCTR.FLE = 8 (Frame Length).
  // SCTR.WLE = 8 (Word Length).
  // SCTR.TRM = 1 (Transmission Mode).
  // SCTR.PDL = 1 (This bit defines the output level at the shift data output signal when no data is available for transmission).
  USIC0_CH0->SCTR &= ~(USIC_CH_SCTR_TRM_Msk | USIC_CH_SCTR_FLE_Msk | USIC_CH_SCTR_WLE_Msk);
  USIC0_CH0->SCTR |= USIC_CH_SCTR_PDL_Msk | (0x01UL << USIC_CH_SCTR_TRM_Pos) | (0x07UL << USIC_CH_SCTR_FLE_Pos) | (0x07UL << USIC_CH_SCTR_WLE_Pos);

  // Configuration of USIC Transmit Control/Status Register:
  // TBUF.TDEN = 1 (TBUF Data Enable: A transmission of the data word in TBUF can be started if TDV = 1.
  // TBUF.TDSSM = 1 (Data Single Shot Mode: allow word-by-word data transmission which avoid sending the same data several times.
  USIC0_CH0->TCSR &= ~(USIC_CH_TCSR_TDEN_Msk);
  USIC0_CH0->TCSR |= USIC_CH_TCSR_TDSSM_Msk | (0x01UL << USIC_CH_TCSR_TDEN_Pos);

  // Configuration of Protocol Control Register:
  // PCR.SMD = 1 (Sample Mode based on majority).
  // PCR.STPB = 0 (1x Stop bit).
  // PCR.SP = 5 (Sample Point).
  // PCR.PL = 0 (Pulse Length is equal to the bit length).
  USIC0_CH0->PCR &= ~(USIC_CH_PCR_ASCMode_STPB_Msk | USIC_CH_PCR_ASCMode_SP_Msk | USIC_CH_PCR_ASCMode_PL_Msk);
  USIC0_CH0->PCR |= USIC_CH_PCR_ASCMode_SMD_Msk | (9 << USIC_CH_PCR_ASCMode_SP_Pos);

  // Configure Transmit Buffer:
  // Standard transmit buffer event is enabled.
  // Define start entry of Transmit Data FIFO buffer DPTR = 0.
  USIC0_CH0->TBCTR &= ~(USIC_CH_TBCTR_SIZE_Msk | USIC_CH_TBCTR_DPTR_Msk);

  // Set Transmit Data Buffer size and set data pointer to position 0.
  USIC0_CH0->TBCTR |= ((UART_FIFO_SIZE << USIC_CH_TBCTR_SIZE_Pos)|(0x00 << USIC_CH_TBCTR_DPTR_Pos));

  // Init UART_RX (P1.4 --> DX5E, or P1.5 --> DOUT0):
  XMC_GPIO_SetMode(P1_4, XMC_GPIO_MODE_INPUT_TRISTATE);
  USIC0_CH0->DX0CR |= 0x00000016;      // USIC0_CH0.DX0E <-- P1.4.
  USIC0_CH0->DX3CR |= 0x00000015;      // USIC0_CH0.DX3E <-- P1.4.
  USIC0_CH0->DX5CR |= 0x00000014;      // USIC0_CH0.DX5E <-- P1.4.

  // Configure Receive Buffer:
  // Standard Receive buffer event is enabled.
  // Define start entry of Receive Data FIFO buffer DPTR.
  USIC0_CH0->RBCTR &= ~(USIC_CH_RBCTR_SIZE_Msk | USIC_CH_RBCTR_DPTR_Msk);

  // Set Receive Data Buffer Size and set data pointer to position max.
  USIC0_CH0->RBCTR |= ((UART_FIFO_SIZE << USIC_CH_RBCTR_SIZE_Pos)|((1<<UART_FIFO_SIZE) << USIC_CH_RBCTR_DPTR_Pos));

  // Init UART_TX (P1.2 --> DOUT):
  XMC_GPIO_SetMode(P1_5, XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2);

  // Configuration of Channel Control Register:
  // CCR.PM = 00 ( Disable parity generation).
  // CCR.MODE = 2 (ASC mode enabled).
  USIC0_CH0->CCR &= ~(USIC_CH_CCR_PM_Msk | USIC_CH_CCR_MODE_Msk);
  USIC0_CH0->CCR |= 0x02UL << USIC_CH_CCR_MODE_Pos;

}
#endif


#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
/**
  * @brief	Send one string (text) through UART TX
  *
  * @param  None
  * @retval None
  */
void UART_TX_String (const char * String_Ptr)
{

	const char *Temp_Ptr;
	uint16_t S_Count = 0;

	Temp_Ptr = String_Ptr;

	/* Transmit string until Null character (end of string), or to user-defined maximum length */
	while ((*Temp_Ptr != '\0') && (S_Count < STRING_MAX_NO))
	{
		/* Put one byte to USIC Transmit FIFO Buffer */
		UART_FIFO_BUFFER = *Temp_Ptr;
		S_Count ++;
		Temp_Ptr ++;
	}

}


/**
  * @brief	Send carriage return, and new line for N times
  *
  * @param  None
  * @retval None
  */

void UART_TX_Return_NewLine (uint16_t N_times)
{

	uint16_t K_Counter;

	for (K_Counter = 0; K_Counter < N_times; K_Counter ++)
	{
		/* Carriage return, \r. */
		UART_FIFO_BUFFER = 0x0D;

		/*  New line / Line Feed, \n.*/
		UART_FIFO_BUFFER = 0x0A;
	}

}

/**
  * @brief	Send ASCII of one uint16_t integer through UART TX, Integer_16 <= 99999
  *
  * @param  None
  * @retval None
  */
void UART_TX_uint16_t (uint16_t Integer_16)
{

	register uint16_t Int_digit;

	#define DVDSLC_UART		0								// Dividend Shift Left Count
	#define DVSSRC_UART		0								// Divisor Shift Right Count.
	#define QSCNT_UART		0								// Quotient Shift Count.

	/* Unsigned Div. Quotient right shift */
	MATH->DIVCON = 0x00008004 | (DVDSLC_UART << 16U) | (DVSSRC_UART << 24U) | (QSCNT_UART << 8U);

	/* Integer_16: 10000 ~ 65535 */
	if (Integer_16 > 9999)
	{
		/*11111** Divider Unit (DIV) #01 **11111*/
		MATH->DVD = Integer_16;

		/* Input Divisor, and auto start of DIV calculation (~35 kernel clock cycles) */
		MATH->DVS = 10000;

		/* Horizontal Tab. \v is Vertical Tab */
		UART_FIFO_BUFFER = '\t';

    /* Wait if DIV is still running calculation */
    while (MATH->DIVST)
    {
      continue;
    }

		/* Read DIV Quotient result */
		Int_digit = MATH->QUOT;

		Integer_16 = MATH->RMD;

		/* ASCII of ten-thousands digit */
		Int_digit += DIFF_09_TO_ASCII;
	}
	else
	{
		/* Horizontal Tab. \v is Vertical Tab */
		Int_digit = '\t';
	}

	/* Dividend */
	MATH->DVD = Integer_16;

	/* Input Divisor, and auto start of DIV calculation */
	MATH->DVS = 1000;

	/* Put ten-thousands digit (or \t) to USIC Transmit FIFO Buffer */
	UART_FIFO_BUFFER = Int_digit;

  /* Wait if DIV is still running calculation. */
  while (MATH->DIVST)
  {
    continue;
  }

	/* Read DIV Quotient result */
	Int_digit = MATH->QUOT;

	Integer_16 = MATH->RMD;

	/* Dividend */
	MATH->DVD = Integer_16;

	/* Input Divisor, and auto start of DIV calculation */
	MATH->DVS = 100;

	/* Put thousands digit to USIC Transmit FIFO Buffer */
	UART_FIFO_BUFFER = Int_digit + DIFF_09_TO_ASCII;

  /* Wait if DIV is still running calculation */
  while (MATH->DIVST)
  {
    continue;
  }

	/*  Read DIV Quotient result */
	Int_digit = MATH->QUOT;

	Integer_16 = MATH->RMD;

	/* Dividend */
	MATH->DVD = Integer_16;

	/* Input Divisor, and auto start of DIV calculation */
	MATH->DVS = 10;

	/* Put hundreds digit to USIC Transmit FIFO Buffer */
	UART_FIFO_BUFFER = Int_digit + DIFF_09_TO_ASCII;

  /* Wait if DIV is still running calculation */
  while (MATH->DIVST)
  {
    continue;
  }

	/* Read DIV Quotient result */
	Int_digit = MATH->QUOT;

	/* Put tens digit to USIC Transmit FIFO Buffer */
	UART_FIFO_BUFFER = Int_digit + DIFF_09_TO_ASCII;

	/* Put units digit to USIC Transmit FIFO Buffer */
	Integer_16 = MATH->RMD;

	/*  Put units digit to USIC Transmit FIFO Buffer. */
	UART_FIFO_BUFFER = Integer_16 + DIFF_09_TO_ASCII;

}

/*
 * Send ASCII of one int16_t integer through UART TX
 * Execution time: ?us (O3 - Optimize most).
 */
void UART_TX_int16_t(int16_t integer_16)
{
  if (integer_16 < 0)
  {
    integer_16 = -integer_16;

    UART_FIFO_BUFFER = 0x2D; /* Put minus sign '-' to USIC Transmit FIFO Buffer. */
  }
  UART_TX_uint16_t(integer_16); /* Send ASCII of one uint16_t integer through UART. */

} /* End of UART_TX_int16_t () */


/* Send one uint16_t hex integer through UART TX */
void UART_TX_uint16_t_Hex(uint16_t Int_16_Hex)
{
  /* P1_4_reset ();                      //P1.4 output 0 for debug. For XMC1302_TSSOP38. */

  /*    UART_FIFO_BUFFER = '\t'; */
  /*    UART_FIFO_BUFFER = '0';                    //Output prefix "0x" for hexadecimal (Hex). */
  /*    UART_FIFO_BUFFER = 'x'; */
  UART_FIFO_BUFFER = 'H';

  register uint16_t Hex_4bits;

  Hex_4bits = (Int_16_Hex & 0xF000) >> 12;
  if (Hex_4bits <= 9)
  {
    Hex_4bits += DIFF_09_TO_ASCII;
  }
  else
  {
    Hex_4bits += DIFF_AF_TO_ASCII;
  }
  UART_FIFO_BUFFER = Hex_4bits; /* Put bits 12~15 to USIC Transmit FIFO Buffer. */

  Hex_4bits = (Int_16_Hex & 0x0F00) >> 8;
  if (Hex_4bits <= 9)
  {
    Hex_4bits += DIFF_09_TO_ASCII;
  }
  else
  {
    Hex_4bits += DIFF_AF_TO_ASCII;
  }
  UART_FIFO_BUFFER = Hex_4bits; /* Put bits 8~11 to USIC Transmit FIFO Buffer. */

  Hex_4bits = (Int_16_Hex & 0x00F0) >> 4;
  if (Hex_4bits <= 9)
  {
    Hex_4bits += DIFF_09_TO_ASCII;
  }
  else
  {
    Hex_4bits += DIFF_AF_TO_ASCII;
  }
  UART_FIFO_BUFFER = Hex_4bits; /* Put bits 4~7 to USIC Transmit FIFO Buffer. */

  Hex_4bits = (Int_16_Hex & 0x000F) >> 0;
  if (Hex_4bits <= 9)
  {
    Hex_4bits += DIFF_09_TO_ASCII;
  }
  else
  {
    Hex_4bits += DIFF_AF_TO_ASCII;
  }
  UART_FIFO_BUFFER = Hex_4bits; /* Put bits 0~3 to USIC Transmit FIFO Buffer. */

  /* P1_4_set ();                        //P1.4 output 1 for debug. For XMC1302_TSSOP38. */

} /* End of UART_TX_uint16_t_Hex () */


/* Receive data through UART, by polling */
void UART_RX(void)
{
  if ((USIC0_CHX_TRBSR & 0x00000008) == 0)
  {
    /* REMPTY = 0, receive buffer is not empty. */
    Motor.UART_Data = (USIC0_CHX_OUTR & 0x0000FFFF); /* Data received via UART. */

    UART_TX_Return_NewLine(0x02); /* Send carriage return, and new line. */
    UART_FIFO_BUFFER = Motor.UART_Data; /* Put received data to USIC Transmit FIFO Buffer. */
    UART_TX_Return_NewLine(0x01); /* Send carriage return, and new line. */
  }
  else
  {
    Motor.UART_Debug_Counter++;

    #define UART_SPEED_UPDATE_RATE    (UART_RATE * 30U)	/* To determine frequency of UART speed update (for debug only). */
    /* if (Motor.UART_Debug_Counter > UART_SPEED_UPDATE_RATE) { */
    if ((Motor.UART_Debug_Counter > UART_SPEED_UPDATE_RATE) && (Motor.State != TRAP_PROTECTION))
    {
      Motor.UART_Debug_Counter = 0;

      UART_TX_String("\r\n rpm:");

      uint32_t Rpm_Speed;
      Rpm_Speed = (Motor.Speed * SPEED_TO_RPM ) >> SCALE_SPEED_TO_RPM;
      UART_TX_uint16_t(Rpm_Speed); /* Debug information, showing rotor speed in rpm. */

      /* UART_TX_String (".");                 // Debug information, showing MCU is running. */
    }
  }
} /* End of UART_RX () */

#endif
