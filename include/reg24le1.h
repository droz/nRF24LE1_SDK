/////////////////////////////////////////////////////////////////////////////
//
// File: reg24le1.h
//
// Copyright S. Brennen Ball, 2011
//
// The author provides no guarantees, warantees, or promises, implied or
//  otherwise.  By using this software you agree to indemnify the author
//  of any damages incurred by using it.
//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
/////////////////////////////////////////////////////////////////////////////

#ifndef REG24LE1_H_
#define REG24LE1_H_

#include <stdint.h>
#include "target_nrf24le1_sdk.h"

#ifndef __SDCC
#define __data
#define __xdata
#endif

#ifndef __SDCC
#include <stdbool.h>
#define __SFR(a,b) volatile uint8_t a;
#define __SFR16(a,b) volatile uint16_t a;
#define __SFR32(a,b) volatile uint32_t a;
#define __SBIT(a, b) volatile bool a;
#else
#define __SFR(a,b) __sfr __at b a;
#define __SFR16(a,b) __sfr16 __at b a;
#define __SFR32(a,b) __sfr32 __at b a;
#define __SBIT(a, b) __sbit __at b a;
#endif

///////////////////////////////////////////
// Single-byte __sfrs
///////////////////////////////////////////
__SFR(P0       , 0x80)
__SFR(SP       , 0x81)
__SFR(DPL      , 0x82)
__SFR(DPH      , 0x83)
__SFR(DPL1     , 0x84)
__SFR(DPH1     , 0x85)
__SFR(PCON     , 0x87)
__SFR(TCON     , 0x88)
__SFR(TMOD     , 0x89)
__SFR(TL0      , 0x8A)
__SFR(TL1      , 0x8B)
__SFR(TH0      , 0x8C)
__SFR(TH1      , 0x8D)
__SFR(P3CON    , 0x8F)
__SFR(P1       , 0x90)
__SFR(DPS      , 0x92)
__SFR(P0DIR    , 0x93)
__SFR(P1DIR    , 0x94)
__SFR(P2DIR    , 0x95)
__SFR(P3DIR    , 0x96)
__SFR(P2CON    , 0x97)
__SFR(S0CON    , 0x98)
__SFR(S0BUF    , 0x99)
__SFR(P0CON    , 0x9E)
__SFR(P1CON    , 0x9F)
__SFR(P2       , 0xA0)
__SFR(PWMDC0   , 0xA1)
__SFR(PWMDC1   , 0xA2)
__SFR(CLKCTRL  , 0xA3)
__SFR(PWRDWN   , 0xA4)
__SFR(WUCON    , 0xA5)
__SFR(INTEXP   , 0xA6)
__SFR(MEMCON   , 0xA7)
__SFR(IEN0     , 0xA8)
__SFR(IP0      , 0xA9)
__SFR(S0RELL   , 0xAA)
__SFR(RTC2CPT01, 0xAB)
__SFR(RTC2CPT10, 0xAC)
__SFR(CLKLFCTRL, 0xAD)
__SFR(OPMCON   , 0xAE)
__SFR(WDSV     , 0xAF)
__SFR(P3       , 0xB0)
__SFR(RSTREAS  , 0xB1)
__SFR(PWMCON   , 0xB2)
__SFR(RTC2CON  , 0xB3)
__SFR(RTC2CMP0 , 0xB4)
__SFR(RTC2CMP1 , 0xB5)
__SFR(RTC2CPT00, 0xB6)
__SFR(SPISRDSZ , 0xB7)
__SFR(IEN1     , 0xB8)
__SFR(IP1      , 0xB9)
__SFR(S0RELH   , 0xBA)
__SFR(SPISCON0 , 0xBC)
__SFR(SPISCON1 , 0xBD)
__SFR(SPISSTAT , 0xBE)
__SFR(SPISDAT  , 0xBF)
__SFR(IRCON    , 0xC0)
__SFR(CCEN     , 0xC1)
__SFR(CCL1     , 0xC2)
__SFR(CCH1     , 0xC3)
__SFR(CCL2     , 0xC4)
__SFR(CCH2     , 0xC5)
__SFR(CCL3     , 0xC6)
__SFR(CCH3     , 0xC7)
__SFR(T2CON    , 0xC8)
__SFR(MPAGE    , 0xC9)
__SFR(_XPAGE   , 0xC9) //for SDCC's memory initialization
__SFR(CRCL     , 0xCA)
__SFR(CRCH     , 0xCB)
__SFR(TL2      , 0xCC)
__SFR(TH2      , 0xCD)
__SFR(WUOPC1   , 0xCE)
__SFR(WUOPC0   , 0xCF)
__SFR(PSW      , 0xD0)
__SFR(ADCCON3  , 0xD1)
__SFR(ADCCON2  , 0xD2)
__SFR(ADCCON1  , 0xD3)
__SFR(ADCDATH  , 0xD4)
__SFR(ADCDATL  , 0xD5)
__SFR(RNGCTL   , 0xD6)
__SFR(RNGDAT   , 0xD7)
__SFR(ADCON    , 0xD8)
__SFR(W2SADR   , 0xD9)
__SFR(W2DAT    , 0xDA)
__SFR(COMPCON  , 0xDB)
__SFR(POFCON   , 0xDC)
__SFR(CCPDATIA , 0xDD)
__SFR(CCPDATIB , 0xDE)
__SFR(CCPDATO  , 0xDF)
__SFR(ACC      , 0xE0)
__SFR(W2CON1   , 0xE1)
__SFR(W2CON0   , 0xE2)
__SFR(SPIRCON0 , 0xE4)
__SFR(SPIRCON1 , 0xE5)
__SFR(SPIRSTAT , 0xE6)
__SFR(SPIRDAT  , 0xE7)
__SFR(RFCON    , 0xE8)
__SFR(MD0      , 0xE9)
__SFR(MD1      , 0xEA)
__SFR(MD2      , 0xEB)
__SFR(MD3      , 0xEC)
__SFR(MD4      , 0xED)
__SFR(MD5      , 0xEE)
__SFR(ARCON    , 0xEF)
__SFR(B        , 0xF0)
__SFR(FSR      , 0xF8)
__SFR(FPCR     , 0xF9)
__SFR(FCR      , 0xFA)
__SFR(SPIMCON0 , 0xFC)
__SFR(SPIMCON1 , 0xFD)
__SFR(SPIMSTAT , 0xFE)
__SFR(SPIMDAT  , 0xFF)

///////////////////////////////////////////
// Two-byte __sfrs
///////////////////////////////////////////
__SFR16(CC1   , 0xC3C2)
__SFR16(CC2   , 0xC5C4)
__SFR16(CC3   , 0xC7C6)
__SFR16(CRC   , 0xCBCA)
__SFR16(T0    , 0x8C8A)
__SFR16(T1    , 0x8D8B)
__SFR16(T2    , 0xCDCC)
__SFR16(S0REL , 0xBAAA)
__SFR16(ADCDAT, 0xD4D5)
__SFR16(MD10  , 0xEAE9)
__SFR16(MD32  , 0xECEB)
__SFR16(MD54  , 0xEEED)

///////////////////////////////////////////
// Four-byte __sfrs
///////////////////////////////////////////
__SFR32(MD3210, 0xECEBEAE9)

///////////////////////////////////////////
// __sbit defines for P0 register
///////////////////////////////////////////
__SBIT(P0_SB_D0, 0x80)
__SBIT(P0_SB_D1, 0x81)
__SBIT(P0_SB_D2, 0x82)
__SBIT(P0_SB_D3, 0x83)
__SBIT(P0_SB_D4, 0x84)
__SBIT(P0_SB_D5, 0x85)
__SBIT(P0_SB_D6, 0x86)
__SBIT(P0_SB_D7, 0x87)

///////////////////////////////////////////
// __sbit defines for TCON register
///////////////////////////////////////////
__SBIT(TCON_SB_IT0, 0x88)
__SBIT(TCON_SB_IE0, 0x89)
__SBIT(TCON_SB_IT1, 0x8A)
__SBIT(TCON_SB_IE1, 0x8B)
__SBIT(TCON_SB_TR0, 0x8C)
__SBIT(TCON_SB_TF0, 0x8D)
__SBIT(TCON_SB_TR1, 0x8E)
__SBIT(TCON_SB_TF1, 0x8F)

///////////////////////////////////////////
// __sbit defines for P1 register
///////////////////////////////////////////
__SBIT(P1_SB_D0, 0x90)
__SBIT(P1_SB_D1, 0x91)
__SBIT(P1_SB_D2, 0x92)
__SBIT(P1_SB_D3, 0x93)
__SBIT(P1_SB_D4, 0x94)
__SBIT(P1_SB_D5, 0x95)
__SBIT(P1_SB_D6, 0x96)
__SBIT(P1_SB_D7, 0x97)

///////////////////////////////////////////
// __sbit defines for S0CON register
///////////////////////////////////////////
__SBIT(S0CON_SB_RI0 , 0x98)
__SBIT(S0CON_SB_TI0 , 0x99)
__SBIT(S0CON_SB_RB80, 0x9A)
__SBIT(S0CON_SB_TB80, 0x9B)
__SBIT(S0CON_SB_REN0, 0x9C)
__SBIT(S0CON_SB_SM20, 0x9D)
__SBIT(S0CON_SB_SM1 , 0x9E)
__SBIT(S0CON_SB_SM0 , 0x9F)

///////////////////////////////////////////
// __sbit defines for P2 register
///////////////////////////////////////////
__SBIT(P2_SB_D0, 0xA0)
__SBIT(P2_SB_D1, 0xA1)
__SBIT(P2_SB_D2, 0xA2)
__SBIT(P2_SB_D3, 0xA3)
__SBIT(P2_SB_D4, 0xA4)
__SBIT(P2_SB_D5, 0xA5)
__SBIT(P2_SB_D6, 0xA6)
__SBIT(P2_SB_D7, 0xA7)

///////////////////////////////////////////
// __sbit defines for IEN0 register
///////////////////////////////////////////
__SBIT(IEN0_SB_IFP   , 0xA8)
__SBIT(IEN0_SB_T0    , 0xA9)
__SBIT(IEN0_SB_POFIRQ, 0xAA)
__SBIT(IEN0_SB_T1    , 0xAB)
__SBIT(IEN0_SB_UART  , 0xAC)
__SBIT(IEN0_SB_T2    , 0xAD)
__SBIT(IEN0_SB_GLOBAL, 0xAF)

///////////////////////////////////////////
// __sbit defines for P3 register
///////////////////////////////////////////
__SBIT(P3_SB_D0, 0xB0)
__SBIT(P3_SB_D1, 0xB1)
__SBIT(P3_SB_D2, 0xB2)
__SBIT(P3_SB_D3, 0xB3)
__SBIT(P3_SB_D4, 0xB4)
__SBIT(P3_SB_D5, 0xB5)
__SBIT(P3_SB_D6, 0xB6)
__SBIT(P3_SB_D7, 0xB7)

///////////////////////////////////////////
// __sbit defines for IEN1 register
///////////////////////////////////////////
__SBIT(IEN1_SB_RFRDY    , 0xB8)
__SBIT(IEN1_SB_RFIRQ    , 0xB9)
__SBIT(IEN1_SB_SPI_2WIRE, 0xBA)
__SBIT(IEN1_SB_WUOPIRQ  , 0xBB)
__SBIT(IEN1_SB_MISCIRQ  , 0xBC)
__SBIT(IEN1_SB_TICK     , 0xBD)
__SBIT(IEN1_SB_T2EXTRLD , 0xBF)

///////////////////////////////////////////
// __sbit defines for IRCON register
///////////////////////////////////////////
__SBIT(IRCON_SB_RFRDY    , 0xC0)
__SBIT(IRCON_SB_RFIRQ    , 0xC1)
__SBIT(IRCON_SB_SPI_2WIRE, 0xC2)
__SBIT(IRCON_SB_WUOPIRQ  , 0xC3)
__SBIT(IRCON_SB_MISCIRQ  , 0xC4)
__SBIT(IRCON_SB_TICK     , 0xC5)
__SBIT(IRCON_SB_TF2      , 0xC6)
__SBIT(IRCON_SB_EXF2     , 0xC7)

///////////////////////////////////////////
// __sbit defines for T2CON register
///////////////////////////////////////////
__SBIT(T2CON_SB_T2I0, 0xC8)
__SBIT(T2CON_SB_T2I1, 0xC9)
__SBIT(T2CON_SB_T2CM, 0xCA)
__SBIT(T2CON_SB_T2R0, 0xCB)
__SBIT(T2CON_SB_T2R1, 0xCC)
__SBIT(T2CON_SB_I2FR, 0xCD)
__SBIT(T2CON_SB_I3FR, 0xCE)
__SBIT(T2CON_SB_T2PS, 0xCF)

///////////////////////////////////////////
// __sbit defines for PSW register
///////////////////////////////////////////
__SBIT(PSW_SB_CY , 0xD7)
__SBIT(PSW_SB_AC , 0xD6)
__SBIT(PSW_SB_F0 , 0xD5)
__SBIT(PSW_SB_RS1, 0xD4)
__SBIT(PSW_SB_RS0, 0xD3)
__SBIT(PSW_SB_OV , 0xD2)
__SBIT(PSW_SB_F1 , 0xD1)
__SBIT(PSW_SB_P  , 0xD0)

///////////////////////////////////////////
// __sbit defines for ADCON register
///////////////////////////////////////////
__SBIT(ADCON_SB_BD, 0xDF)

///////////////////////////////////////////
// __sbit defines for RFCON register
///////////////////////////////////////////
__SBIT(RFCON_SB_RFCE  , 0xE8)
__SBIT(RFCON_SB_RFCSN , 0xE9)
__SBIT(RFCON_SB_RFCKEN, 0xEA)

///////////////////////////////////////////
// __sbit defines for FSR register
///////////////////////////////////////////
__SBIT(FSR_SB_RDISMB, 0xFA)
__SBIT(FSR_SB_INFEN , 0xFB)
__SBIT(FSR_SB_RDYN  , 0xFC)
__SBIT(FSR_SB_WEN   , 0xFD)
__SBIT(FSR_SB_STP   , 0xFE)
__SBIT(FSR_SB_ENDBG , 0xFF)

///////////////////////////////////////////
// Bit defines for PCON register
///////////////////////////////////////////
#define PCON_IDLE	0x01
#define PCON_STOP	0x02
#define PCON_GF0	0x04
#define PCON_GF1	0x08
#define PCON_PMW	0x10
#define PCON_GF2	0x20
#define PCON_GF3	0x40
#define PCON_SMOD	0x80

///////////////////////////////////////////
// Bit defines for PX registers
///////////////////////////////////////////
#define PX_D0	0x01
#define PX_D1	0x02
#define PX_D2	0x04
#define PX_D3	0x08
#define PX_D4	0x10
#define PX_D5	0x20
#define PX_D6	0x40
#define PX_D7	0x80

///////////////////////////////////////////
// Bit defines for PXDIR registers
///////////////////////////////////////////
#define PXDIR_D0	0x01
#define PXDIR_D1	0x02
#define PXDIR_D2	0x04
#define PXDIR_D3	0x08
#define PXDIR_D4	0x10
#define PXDIR_D5	0x20
#define PXDIR_D6	0x40
#define PXDIR_D7	0x80

///////////////////////////////////////////
// Bit defines for PXCON registers
///////////////////////////////////////////
#define PXCON_BIT_ADDR_SHIFT	0
#define PXCON_BIT_ADDR_MASK		(0x07 << PXCON_BIT_ADDR_SHIFT)
#define PXCON_READ_ADDR			0x08
#define PXCON_IN_OUT			0x10
#define PXCON_PINMODE_SHIFT		5
#define PXCON_PINMODE_MASK		(0x07 << PXCON_PINMODE_SHIFT)

///////////////////////////////////////////
// Bit defines for S0CON register
///////////////////////////////////////////
#define S0CON_RI0			0x01
#define S0CON_TI0			0x02
#define S0CON_RB80			0x04
#define S0CON_TB80			0x08
#define S0CON_REN0			0x10
#define S0CON_SM20			0x20
#define S0CON_MODE_SHIFT	6
#define S0CON_MODE_FIELD	(0x03 << S0CON_MODE_SHIFT)

///////////////////////////////////////////
// Bit defines for ADCON register
///////////////////////////////////////////
#define ADCON_BD	0x80

///////////////////////////////////////////
// Bit defines for RFCON register
///////////////////////////////////////////
#define RFCON_RFCE		0x01
#define RFCON_RFCSN		0x02
#define RFCON_RFCKEN	0x04

///////////////////////////////////////////
// Bit defines for SPIRCON register
///////////////////////////////////////////
#define SPIRCON1_MASK_IRQ_TX_FIFO_READY	0x01
#define SPIRCON1_MASK_IRQ_TX_FIFO_EMPTY	0x02
#define SPIRCON1_MASK_IRQ_RX_FIFO_READY	0x04
#define SPIRCON1_MASK_IRQ_RX_FIFO_FULL	0x08

///////////////////////////////////////////
// Bit defines for SPIRSTAT register
///////////////////////////////////////////
#define SPIRSTAT_IRQ_TX_FIFO_READY	0x01
#define SPIRSTAT_IRQ_TX_FIFO_EMPTY	0x02
#define SPIRSTAT_IRQ_RX_FIFO_READY	0x04
#define SPIRSTAT_IRQ_RX_FIFO_FULL	0x08

///////////////////////////////////////////
// Bit defines for IRCON register
///////////////////////////////////////////
#define IRCON_RFRDY		0x01
#define IRCON_RFIRQ		0x02
#define IRCON_SPI_2WIRE	0x04
#define IRCON_WUOPIRQ	0x08
#define IRCON_MISCIRQ	0x10
#define IRCON_TICK		0x20
#define IRCON_TF2		0x40
#define IRCON_EXF2		0x80

///////////////////////////////////////////
// Bit defines for INTEXP register
///////////////////////////////////////////
#define INTEXP_SSPI_COMPLETED_INT_ENABLE	0x01
#define INTEXP_MSPI_COMPLETED_INT_ENABLE	0x02
#define INTEXP_2WIRE_COMPLETED_INT_ENABLE	0x04
#define INTEXP_GP_INT0_ENABLE				0x08
#define INTEXP_GP_INT1_ENABLE				0x10
#define INTEXP_GP_INT2_ENABLE				0x20

///////////////////////////////////////////
// Bit defines for SPIMCON0 register
///////////////////////////////////////////
#define SPIMCON0_ENABLE			0x01
#define SPIMCON0_CPHA			0x02
#define SPIMCON0_CPOL			0x04
#define SPIMCON0_DATA_ORDER		0x08
#define SPIMCON0_CLK_DIV_SHIFT	4
#define SPIMCON0_CLK_DIV_MASK	(0x03 << SPIMCON0_CLK_DIV_SHIFT)

///////////////////////////////////////////
// Bit defines for SPIMCON1 register
///////////////////////////////////////////
#define SPIMCON1_INT_TX_FIFO_READY_DISABLE	0x01
#define SPIMCON1_INT_TX_FIFO_EMPTY_DISABLE	0x02
#define SPIMCON1_INT_RX_DATA_READY_DISABLE	0x04
#define SPIMCON1_INT_RX_DATA_FULL_DISABLE	0x08

///////////////////////////////////////////
// Bit defines for SPIMSTAT register
///////////////////////////////////////////
#define SPIMSTAT_INT_TX_FIFO_READY_FLAG	0x01
#define SPIMSTAT_INT_TX_FIFO_EMPTY_FLAG	0x02
#define SPIMSTAT_INT_RX_DATA_READY_FLAG	0x04
#define SPIMSTAT_INT_RX_DATA_FULL_FLAG	0x08

///////////////////////////////////////////
// Bit defines for SPISCON0 register
///////////////////////////////////////////
#define SPISCON0_ENABLE						0x01
#define SPISCON0_CPHA						0x02
#define SPISCON0_CPOL						0x04
#define SPISCON0_DATA_ORDER					0x08
#define SPISCON0_INT_SPI_SLAVE_DONE_DISABLE	0x10
#define SPISCON0_INT_CSN_LOW_DISABLE		0x20
#define SPISCON0_INT_CSN_HIGH_DISABLE		0x40

///////////////////////////////////////////
// Bit defines for SPISSTAT register
///////////////////////////////////////////
#define SPISSTAT_INT_SPI_SLAVE_DONE_FLAG	0x01
#define SPISSTAT_INT_CSN_LOW_FLAG			0x10
#define SPISSTAT_INT_CSN_HIGH_FLAG			0x20

///////////////////////////////////////////
// Bit defines for ADCCON1 register
///////////////////////////////////////////
#define ADCCON1_REF_SEL_SHIFT		0
#define ADCCON1_REF_SEL_MASK		(0x03 << ADCCON1_REF_SEL_SHIFT)
#define ADCCON1_CHAN_SEL_SHIFT		2
#define ADCCON1_CHAN_SEL_MASK		(0x0F << ADCCON1_CHAN_SEL_SHIFT)
#define ADCCON1_BUSY_FLAG			0x40
#define ADCCON1_POWER_UP			0x80

///////////////////////////////////////////
// Bit defines for ADCCON2 register
///////////////////////////////////////////
#define ADCCON2_ACQ_TIME_SHIFT			0
#define ADCCON2_ACQ_TIME_MASK			(0x03 << ADCCON2_ACQ_TIME_SHIFT)
#define ADCCON2_SAMPLING_RATE_SHIFT		2
#define ADCCON2_SAMPLING_RATE_MASK		(0x03 << ADCCON2_SAMPLING_RATE_SHIFT)
#define ADCCON2_CONTINUOUS_SAMPLING		0x20
#define ADCCON2_SAMPLING_MODE_SHIFT		6
#define ADCCON2_SAMPLING_MODE_MASK		(0x03 << ADCCON2_SAMPLING_MODE_SHIFT)

///////////////////////////////////////////
// Bit defines for ADCCON3 register
///////////////////////////////////////////
#define ADCCON3_RANGE_FLAG				0x04
#define ADCCON3_OVERFLOW_FLAG			0x08
#define ADCCON3_UNDERFLOW_FLAG			0x10
#define ADCCON3_RESULT_RIGHT_JUSTIFIED	0x20
#define ADCCON3_BIT_RES_SHIFT			6
#define ADCCON3_BIT_RES_MASK			(0x03 << ADCCON3_BIT_RES_SHIFT)

///////////////////////////////////////////
// Bit defines for TCON register
///////////////////////////////////////////
#define TCON_IT0	0x01
#define TCON_IE0	0x02
#define TCON_IT1	0x04
#define TCON_IE1	0x08
#define TCON_TR0	0x10
#define TCON_TF0	0x20
#define TCON_TR1	0x40
#define TCON_TF1	0x80

///////////////////////////////////////////
// Bit defines for TMOD register
///////////////////////////////////////////
#define TMOD_MODE0_SHIFT	0
#define TMOD_MODE0_MASK		(0x03 << TMOD_MODE0_SHIFT)
#define TMOD_CT0			0x04
#define TMOD_GATE0			0x08
#define TMOD_MODE1_SHIFT	4
#define TMOD_MODE1_MASK		(0x03 << TMOD_MODE1_SHIFT)
#define TMOD_CT1			0x40
#define TMOD_GATE1			0x80

///////////////////////////////////////////
// Bit defines for PWMCON register
///////////////////////////////////////////
#define PWMCON_PWM0_ENABLE		0x01
#define PWMCON_PWM1_ENABLE		0x02
#define PWMCON_PRESCALER_SHIFT	2
#define PWMCON_PRESCALER_MASK	(0x0F << PWMCON_PRESCALER_SHIFT)
#define PWMCON_BIT_WIDTH_SHIFT	6
#define PWMCON_BIT_WIDTH_MASK	(0x03 << PWMCON_BIT_WIDTH_SHIFT)

///////////////////////////////////////////
// Bit defines for RNGCTL register
///////////////////////////////////////////
#define RNGCTL_RESULT_READY			0x20
#define RNGCTL_CORRECTOR_ENABLE		0x40
#define RNGCTL_POWER_UP				0x80

///////////////////////////////////////////
// Bit defines for CLKCTRL register
///////////////////////////////////////////
#define CLKCTRL_CLK_FREQ_SHIFT					0
#define CLKCTRL_CLK_FREQ_MASK					(0x07 << CLKCTRL_CLK_FREQ_SHIFT)
#define CLKCTRL_XOSC16M_ACTIVE_WKUP_INT_ENABLE	0x08
#define CLKCTRL_XOSC16M_RCOSC16M_START_SHIFT	4
#define CLKCTRL_XOSC16M_RCOSC16M_START_MASK		(0x03 << CLKCTRL_XOSC16M_RCOSC16M_START_SHIFT)
#define CLKCTRL_CLK_SRC_PIN_XC1_OR_OSC			0x40
#define CLKCTRL_XOSC16M_IN_REG_RET_MODE_ENABLE	0x80

///////////////////////////////////////////
// Bit defines for CLKLFCTRL register
///////////////////////////////////////////
#define CLKLFCTRL_CLKLF_SRC_SHIFT		0
#define CLKLFCTRL_CLKLF_SRC_MASK		(0x07 << CLKLFCTRL_CLKLF_SRC_SHIFT)
#define CLKLFCTRL_IS_CLKLF_SRC_XOSC16M	0x08
#define CLKLFCTRL_IS_CLKLF_READY		0x40
#define CLKLFCTRL_CLKLF_VAL_READ		0x80

///////////////////////////////////////////
// Bit defines for PWRDWN register
///////////////////////////////////////////
#define PWRDWN_PWR_CNTL_SHIFT				0
#define PWRDWN_PWR_CNTL_MASK				(0x07 << PWRDWN_PWR_CNTL_SHIFT)
#define PWRDWN_PWR_IS_WAKE_FROM_COMPARATOR	0x20
#define PWRDWN_PWR_IS_WAKE_FROM_TICK		0x40
#define PWRDWN_PWR_IS_WAKE_FROM_PIN			0x80

///////////////////////////////////////////
// Bit defines for RSTREAS register
///////////////////////////////////////////
#define RSTREAS_RESET_REASON_SHIFT	0
#define RSTREAS_RESET_REASON_MASK	(0x07 << RSTREAS_RESET_REASON_SHIFT)

///////////////////////////////////////////
// Bit defines for OPMCON register
///////////////////////////////////////////
#define OPMCON_WATCHDOG_RESET_ENABLE	0x01
#define OPMCON_RETENTION_LATCH_CONTROL	0x02
#define OPMCON_WAKEUP_PINS_POLARITY		0x04

///////////////////////////////////////////
// Bit defines for POFCON register
///////////////////////////////////////////
#define POFCON_POF_WARNING			0x10
#define POFCON_POF_THRESHOLD_SHIFT	5
#define POFCON_POF_THRESHOLD_MASK	(0x03 << POFCON_POF_THRESHOLD_SHIFT)
#define POFCON_POF_ENABLE			0x80

///////////////////////////////////////////
// Bit defines for WUCON register
///////////////////////////////////////////
#define WUCON_WAKE_ON_MISCIRQ_SHIFT		0
#define WUCON_WAKE_ON_MISCIRQ_MASK		(0x03 << WUCON_WAKE_ON_MISCIRQ_SHIFT)
#define WUCON_WAKE_ON_WUOPIRQ_SHIFT		2
#define WUCON_WAKE_ON_WUOPIRQ_MASK		(0x03 << WUCON_WAKE_ON_WUOPIRQ_SHIFT)
#define WUCON_WAKE_ON_RTC2_TICK_SHIFT	4
#define WUCON_WAKE_ON_RTC2_TICK_MASK	(0x03 << WUCON_WAKE_ON_RTC2_TICK_SHIFT)
#define WUCON_WAKE_ON_RFIRQ_SHIFT		6
#define WUCON_WAKE_ON_RFIRQ_MASK		(0x03 << WUCON_WAKE_ON_RFIRQ_SHIFT)

///////////////////////////////////////////
// Bit defines for WUOPCX registers
///////////////////////////////////////////
#define WUOPCX_PIN0		0x01
#define WUOPCX_PIN1		0x02
#define WUOPCX_PIN2		0x04
#define WUOPCX_PIN3		0x08
#define WUOPCX_PIN4		0x10
#define WUOPCX_PIN5		0x20
#define WUOPCX_PIN6		0x40
#define WUOPCX_PIN7		0x80

///////////////////////////////////////////
// Bit defines for RTC2CON register
///////////////////////////////////////////
#define RTC2CON_ENABLE						0x01
#define RTC2CON_COMPARE_MODE_SHIFT			1
#define RTC2CON_COMPARE_MODE_MASK			(0x03 << RTC2CON_COMPARE_MODE_SHIFT)
#define RTC2CON_ENABLE_EXTERNAL_CAPTURE		0x04
#define RTC2CON_SFR_CAPTURE					0x10

///////////////////////////////////////////
// Bit defines for T2CON register
///////////////////////////////////////////
#define T2CON_INPUT_SEL_SHIFT		0
#define T2CON_INPUT_SEL_MASK		(0x03 << T2CON_INPUT_SEL_SHIFT)
#define T2CON_COMPARE_MODE			0x02
#define T2CON_RELOAD_MODE_SHIFT		3
#define T2CON_RELOAD_MODE_MASK		(0x03 << T2CON_RELOAD_MODE_SHIFT)
#define T2CON_INT2_EDGE				0x20
#define T2CON_INT3_EDGE				0x40
#define T2CON_PRESCALER				0x80

///////////////////////////////////////////
// Bit defines for CCEN register
///////////////////////////////////////////
#define CCEN_CRC_MODE_SHIFT			0
#define CCEN_CRC_MODE_MASK			(0x03 << CCEN_CRC_MODE_SHIFT)
#define CCEN_CC1_MODE_SHIFT			2
#define CCEN_CC1_MODE_MASK			(0x03 << CCEN_CC1_MODE_SHIFT)
#define CCEN_CC2_MODE_SHIFT			4
#define CCEN_CC2_MODE_MASK			(0x03 << CCEN_CC2_MODE_SHIFT)
#define CCEN_CC3_MODE_SHIFT			6
#define CCEN_CC3_MODE_MASK			(0x03 << CCEN_CC3_MODE_SHIFT)

///////////////////////////////////////////
// Bit defines for COMPCON register
///////////////////////////////////////////
#define COMPCON_ENABLE				0x01
#define COMPCON_REF_SEL				0x02
#define COMPCON_REF_SCALE_SHIFT		2
#define COMPCON_REF_SCALE_MASK		(0x03 << COMPCON_REF_SCALE_SHIFT)
#define COMPCON_POLARITY			0x10

///////////////////////////////////////////
// Bit defines for W2CON0 register
///////////////////////////////////////////
#define W2CON0_ENABLE							0x01
#define W2CON0_MASTER_SELECT					0x02
#define W2CON0_CLOCK_FREQUENCY_SHIFT			2
#define W2CON0_CLOCK_FREQUENCY_MASK				(0x03 << W2CON0_CLOCK_FREQUENCY_SHIFT)
#define W2CON0_MASTER_TX_START					0x10
#define W2CON0_SLAVE_DISABLE_IRQ_ON_ADDR_MATCH	0x10
#define W2CON0_MASTER_TX_STOP					0x20
#define W2CON0_SLAVE_DISABLE_IRQ_ON_STOP		0x20
#define W2CON0_SLAVE_CLOCK_STOP					0x40
#define W2CON0_SLAVE_BROADCAST_ENABLE			0x80

///////////////////////////////////////////
// Bit defines for W2CON1 register
///////////////////////////////////////////
#define W2CON1_DATA_READY						0x01
#define W2CON1_LAST_ACK							0x02
#define W2CON1_SLAVE_IRQ_DUE_TO_ADDR_MATCH		0x04
#define W2CON1_SLAVE_IRQ_DUE_TO_STOP			0x08
#define W2CON1_SLAVE_LAST_ADDR_WAS_BROADCAST	0x10
#define W2CON1_DISABLE_INTERRUPTS				0x20

///////////////////////////////////////////
// Bit defines for W2DAT register
///////////////////////////////////////////
#define W2DAT_DIRECTION			0x01
#define W2DAT_ADDRESS_SHIFT		1
#define W2DAT_ADDRESS_MASK		(0x7F << W2DAT_ADDRESS_SHIFT)

///////////////////////////////////////////
// Bit defines for MEMCON register
///////////////////////////////////////////
#define MEMCON_SRAM_DATA_RETENTIVE_MAPPING		0x01
#define MEMCON_SRAM_DATA_NON_RETENTIVE_MAPPING	0x02
#define MEMCON_SRAM_START_LOCATION				0x04

///////////////////////////////////////////
// Bit defines for FSR register
///////////////////////////////////////////
#define FSR_RDISMB	0x04
#define FSR_INFEN	0x08
#define FSR_RDYN	0x10
#define FSR_WEN		0x20
#define FSR_STP		0x40
#define FSR_ENDBG	0x80

///////////////////////////////////////////
// Bit defines for FPCR register
///////////////////////////////////////////
#define FPCR_NUPP_MASK	0x7F


///////////////////////////////////////////
// Additional defines
///////////////////////////////////////////
#define	CCLK_MAX_FREQ_HZ	16000000					//16 MHz clock frequency
#define	CCLK_MAX_FREQ_KHZ	(CCLK_MAX_FREQ_HZ / 1000)	//16 MHz clock frequency
#define	CCLK_MAX_FREQ_MHZ	(CCLK_MAX_FREQ_KHZ / 1000)	//16 MHz clock frequency
#define BIT_TRUE			1							//Simple define for 1
#define BIT_FALSE			0							//Simple define for 0

///////////////////////////////////////////
// define for bool type
///////////////////////////////////////////
#ifndef bool
#define bool uint8_t
#endif
#ifndef false
#define false 0
#endif
#ifndef true
#define true (!false)
#endif

////////////////////////////////////////////////
// Defenitions for nRF24LE1 packages
////////////////////////////////////////////////
#define NRF24LE1_PACKAGE_24_PIN 0
#define NRF24LE1_PACKAGE_32_PIN 1
#define NRF24LE1_PACKAGE_48_PIN 2

/////////////////////////////////////////////////////////////////
// Verify existence of __TARG_PACKAGE_TYPE
/////////////////////////////////////////////////////////////////
#ifndef __TARG_PACKAGE_TYPE
#error "You must define __TARG_PACKAGE_TYPE in the appropriate target_nrf24le1_sdk.h file to use this header file"
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
// Verify that __TARG_PACKAGE_TYPE has been set to an appropriate value
//////////////////////////////////////////////////////////////////////////////////////////////
#if (__TARG_PACKAGE_TYPE != NRF24LE1_PACKAGE_24_PIN) && (__TARG_PACKAGE_TYPE != NRF24LE1_PACKAGE_32_PIN) && (__TARG_PACKAGE_TYPE != NRF24LE1_PACKAGE_48_PIN)
#error "__TARG_PACKAGE_TYPE has been defined, but it has been defined to an incorrect value.  Please see #defines NR24LE1_PACKAGE_XX_PIN in include/reg24le1.h"
#endif


///////////////////////////////////////////////////////////////////////////////
// Simple instructions
///////////////////////////////////////////////////////////////////////////////
#ifndef __SDCC
#define nop() asm("nop")	//No-operation instruction wrapper
#else
#define nop() __asm nop __endasm	//No-operation instruction wrapper
#endif

///////////////////////////////////////////////////////////////////////////////
// Macros for waiting on bit values
///////////////////////////////////////////////////////////////////////////////
#define wait_for_bit_level_high(bit)	while(!(bit))					//Wait for a bit to go high
#define wait_for_bit_level_low(bit)	    while(bit)						//Wait for a bit to go low
#define wait_for_bit_edge_rising(bit)	wait_for_bit_level_low(bit);\
										wait_for_bit_level_high(bit)	//Wait for a bit to have a rising edge
#define wait_for_bit_edge_falling(bit)	wait_for_bit_level_high(bit);\
										wait_for_bit_level_low(bit)		//Wait for a bit to have a falling edge
#define wait_for_bit_toggle(bit)		if(bit)\
											while(bit);\
										else\
											while(!(bit))				//Wait for a bit to change in value

///////////////////////////////////////////////////////////////////////////////
// Macros for changing bits in registers
///////////////////////////////////////////////////////////////////////////////
#define reg_bits_set(reg, mask)			((reg) |= (mask))	//Set bit(s) in a register
#define reg_bits_clear(reg, mask)		((reg) &= ~(mask))	//Clear bit(s) in a register
#define reg_bits_complement(reg, mask)	((reg) ^= (mask))	//Complement bit(s) in a register

///////////////////////////////////////////////////////////////////////////////
// Macros for changing SBITs
///////////////////////////////////////////////////////////////////////////////
#ifndef __SDCC
// These are just for GCC to be able to parse the code and do syntax highlight
#define sbit_set(sbit_pin_set)               asm("setb " #sbit_pin_set)
#define sbit_clear(sbit_pin_clear)           asm("clr " #sbit_pin_clear)
#define sbit_complement(sbit_pin_complement) asm("cpl " #sbit_pin_complement)
#else
#define sbit_set(sbit_pin_set)					__asm setb _##sbit_pin_set __endasm			//Set an SBIT
#define sbit_clear(sbit_pin_clear)				__asm clr  _##sbit_pin_clear __endasm		//Clear an SBIT
#define sbit_complement(sbit_pin_complement)	__asm cpl  _##sbit_pin_complement __endasm	//Complement an SBIT
#endif

#endif
