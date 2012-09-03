/*==========================================================================*\
|                                                                            |
| LowLevelFunc430.c                                                          |
|                                                                            |
| Low Level Functions regarding user's hardware                              |
|----------------------------------------------------------------------------|
| Project:              MSP430 Replicator                                    |
| Developed using:      IAR Embedded Workbench 3.40B [Kickstart]             |
|             and:      Code Composer Eessentials 2.0                        |
|----------------------------------------------------------------------------|
| Author:               FRGR                                                 |
| Version:              1.6                                                  |
| Initial Version:      04-17-02                                             |
| Last Change:          05-24-12                                             |
|----------------------------------------------------------------------------|
| Version history:                                                           |
| 1.0 04/02 FRGR        Initial version.                                     |
| 1.1 04/02 FRGR        Included SPI mode to speed up shifting function by 2.|
| 1.2 06/02 ALB2        Formatting changes, added comments.                  |
| 1.3 08/02 ALB2        Initial code release with Lit# SLAA149.              |
| 1.4 09/05 SUN1        Software delays redesigned to use TimerA harware;    |
|                       see MsDelay() routine. Added TA setup                |
| 1.5 12/05 STO         Adapted for 2xx devices with SpyBiWire using 4JTAG   |
| 1.6 08/08 WLUT        Cleaned up InitTarget() for JTAG init sequence.      |
| 1.7 05/09 GC (Elprotronic)  Added support for the new hardware - REP430F   |
| 1.8 07/09 FB          Added support for Spy-Bi-Wire and function           |
|                        configure_IO_SBW( void )                            |
| 1.9 05/12 RL          Updated commentaries                                 |
|----------------------------------------------------------------------------|
| Designed 2002 by Texas Instruments Germany                                 |
\*==========================================================================*/
/*
 * 
 * Copyright (C) 2002 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

//! \file LowLevelFunc430.c
//! \brief Low Level Functions regarding user's Hardware
//! \author Wolfgang Lutsch
//! \date 09/16/2009
//! \version 1.9

/****************************************************************************/
/* INCLUDES                                                                 */
/****************************************************************************/

#include <linux/time.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>

#define spin_lock_w_flags(_l) unsigned long _flags;  spin_lock_irqsave((_l), _flags)
#define spin_relock_w_flags(_l) spin_lock_irqsave((_l), _flags)
#define spin_unlock_w_flags(_l) spin_unlock_irqrestore((_l), _flags)

#include "LowLevelFunc430.h"
#include "JTAGfunc430.h"

#define TDIO_GPIO 134
#define TCK_GPIO 136

byte current_tdio = 1;
byte current_tck = 1;

DEFINE_SPINLOCK(lock);

void SetTDIODir(byte val) {
    if(val) {
        gpio_direction_output(TDIO_GPIO, current_tdio);
    } else {
        gpio_direction_input(TDIO_GPIO);
    }
}

byte GetTDIO(void) {
    if(gpio_get_value(TDIO_GPIO)) {
        return 1;
    }  else {
        return 0;
    }
}

void SetTDIO(byte val) {
    if(val) {
        current_tdio = 1;
        gpio_set_value(TDIO_GPIO, 1);
    } else {
        current_tdio = 0;
        gpio_set_value(TDIO_GPIO, 0);
    }
}

void SetTCKDir(byte val) {
    if(val) {
        gpio_direction_output(TCK_GPIO, current_tck);
    } else {
        gpio_direction_input(TCK_GPIO);
    }
}

void SetTCK(byte val) {
    if(val) {
        current_tck = 1;
        gpio_set_value(TCK_GPIO, 1);
    } else {
        current_tck = 0;
        gpio_set_value(TCK_GPIO, 0);
    }
}

void    usDelay(word microeconds) {
    udelay(microeconds);
}

void    MsDelay(word milliseconds) {
    msleep(milliseconds);
}


/****************************************************************************/
/* Macros and Pin-to-Signal assignments which have to be programmed         */
/* by the user. This implementation assumes use of an MSP430F5437 as the    */
/* host controller and the corresponding hardware given in the MSP430       */
/* Programming Via the JTAG Interface User's Guide (SLAU320).               */
/*                                                                          */
/* The following MSP430 example acts as a hint of how to generally          */
/* implement a micro-controller programmer solution for the MSP430 flash-   */
/* based devices.                                                           */
/****************************************************************************/

// I/O Level translators (SN74LVC1T45) direction setup

#define TMS             0x20
#define TDI             0x80
#define TDO             0x40
#define TCK             0x10
#define TEST            0x04
#define RST             0x08
#define TCLK            TDI

/****************************************************************************/
/* Macros to control Spy-Bi-Wire-IF                                         */
/****************************************************************************/

//! \brief JTAG data_out pin in SBW mode -separate pin in MSP430F5437 - common IO translator
#define   SBWDATO   TDI

//! \brief SBW macro: set TMS signal
#define   TMSH    {SetTDIO(1); { spin_lock_w_flags(&lock); SetTCK(0); SetTCK(1); spin_unlock_w_flags(&lock); } }
//! \brief SBW macro: clear TMS signal
#define   TMSL    {SetTDIO(0); { spin_lock_w_flags(&lock); SetTCK(0); SetTCK(1); spin_unlock_w_flags(&lock); } }
//! \brief SBW macro: clear TMS signal and immediately set it high again in
//! the SBWTCK low phase to enter the TDI slot with a high signal 
//! \details Only used to clock TCLK (=TDI for SBW) in Run-Test/IDLE mode of
//! the JTAG FSM 
#define   TMSLDH  {SetTDIO(0); { spin_lock_w_flags(&lock); SetTCK(0); SetTDIO(1); SetTCK(1); spin_unlock_w_flags(&lock); } }
//! \brief SBW macro: Set TDI = 1
#define   TDIH    {SetTDIO(1);  { spin_lock_w_flags(&lock); SetTCK(0); SetTCK(1); spin_unlock_w_flags(&lock); } }
//! \brief SBW macro: clear TDI signal
#define   TDIL    {SetTDIO(0); { spin_lock_w_flags(&lock); SetTCK(0); SetTCK(1); spin_unlock_w_flags(&lock); } }
//! \brief SBW macro: TDO cycle without reading TDO
#define   TDOsbw  {SetTDIODir(0); { spin_lock_w_flags(&lock); SetTCK(0); SetTCK(1); spin_unlock_w_flags(&lock); } SetTDIODir(1);}
//! \brief SBW macro: TDO cycle with TDO read
#define   TDO_RD  {SetTDIODir(0); { spin_lock_w_flags(&lock); SetTCK(0); tdo_bit = GetTDIO(); SetTCK(1); spin_unlock_w_flags(&lock); } SetTDIODir(1);}


//! \brief SBW macro: set TCK signal
void SetSBWTCK(void) {
    SetTCK(1);
}
//! \brief SBW macro: clear TCK signal
void ClrSBWTCK(void) {
    SetTCK(0);
}
//! \brief SBW macro: set TDIO signal
void SetSBWTDIO(void) {
    SetTDIO(1);
}
//! \brief SBW macro: clear TDIO signal
void ClrSBWTDIO(void) {
    SetTDIO(0);
}

//! \brief SBW macro: clear TCLK signal
void ClrTCLK(void) {
    ClrTCLK_sbw();
}
//! \brief SBW macro: set TCLK signal
void SetTCLK(void) {
    SetTCLK_sbw();
}

//! \brief SBW macro: set RST signal
void SetRST(void) {
    SetTDIO(1);
}
//! \brief SBW macro: clear RST signal
void ClrRST(void) {
    SetTDIO(0);
}
//! \brief SBW macro: release RST signal (pro forma)
void ReleaseRST(void) {
}
//! \brief SBW macro: set TEST pin signal
void SetTST(void) {
    SetTCK(1);
}
//! \brief SBW macro: clear TEST pin signal
void ClrTST(void) {
    SetTCK(0);
}
 
/****************************************************************************/
/* GLOBAL VARIABLES                                                         */
/****************************************************************************/

//! \brief Holds the value of TDO-bit
byte tdo_bit;
//! \brief Holds the last value of TCLK before entering a JTAG sequence
byte TCLK_saved = 1;

/****************************************************************************/
/* FUNCTIONS                                                                */
/****************************************************************************/

byte IsTCKHigh(void) {
    return (TCLK_saved & 1);
}

// Combinations of sbw-cycles (TMS, TDI, TDO)
//----------------------------------------------------------------------------
//! \brief Combination of SBW macros: TMS low, TDI low, no TDO read
void TMSL_TDIL(void)
{
    TMSL  TDIL  TDOsbw
}
//----------------------------------------------------------------------------
//! \brief Combination of SBW macros: TMS high, TDI low, no TDO read
void TMSH_TDIL(void)
{
    TMSH  TDIL  TDOsbw
}
//----------------------------------------------------------------------------
//! \brief Combination of SBW macros: TMS low, TDI high, no TDO read
void TMSL_TDIH(void)
{
    TMSL  TDIH  TDOsbw
}
//----------------------------------------------------------------------------
//! \brief Combination of SBW macros: TMS high, TDI high, no TDO read
void TMSH_TDIH(void)
{
    TMSH  TDIH  TDOsbw
}
//----------------------------------------------------------------------------
//! \brief Combination of SBW macros: TMS low, TDI high, TDO read
void TMSL_TDIH_TDOrd(void)
{
    TMSL  TDIH  TDO_RD
}
//----------------------------------------------------------------------------
//! \brief Combination of SBW macros: TMS low, TDI low, TDO read
void TMSL_TDIL_TDOrd(void)
{
    TMSL  TDIL  TDO_RD
}
//----------------------------------------------------------------------------
//! \brief Combination of SBW macros: TMS high, TDI high, TDO read
void TMSH_TDIH_TDOrd(void)
{
    TMSH  TDIH  TDO_RD
}
//----------------------------------------------------------------------------
//! \brief Combination of SBW macros: TMS high, TDI low, TDO read
void TMSH_TDIL_TDOrd(void)
{
    TMSH  TDIL  TDO_RD
}

//----------------------------------------------------------------------------
//! \brief Clear TCLK in Spy-Bi-Wire mode
//! \details enters with TCLK_saved and exits with TCLK = 0
void ClrTCLK_sbw(void)
{
    if (IsTCKHigh() )
    {
        TMSLDH
    }
    else
    {
        TMSL
    }

    SetTDIO(0);

    TDIL TDOsbw    //ExitTCLK
    TCLK_saved = 0;
}

//----------------------------------------------------------------------------
//! \brief Set TCLK in Spy-Bi-Wire mode
//! \details enters with TCLK_saved and exits with TCLK = 1
void SetTCLK_sbw(void)
{
    if (IsTCKHigh() )
    {
        TMSLDH
    }
    else
    {
        TMSL
    }

   SetTDIO(1);

   TDIH TDOsbw    //ExitTCLK
   TCLK_saved = 1;
}

//----------------------------------------------------------------------------
//! \brief Shift a value into TDI (MSB first) and simultaneously shift out a 
//! value from TDO (MSB first).
//! \param[in] Format (number of bits shifted, 8 (F_BYTE), 16 (F_WORD), 
//! 20 (F_ADDR) or 32 (F_LONG))
//! \param[in] Data (data to be shifted into TDI)
//! \return unsigned long (scanned TDO value)
word Shift(word Format, word Data)
{
   word TDOword = 0x0000;
   word MSB = 0x0000;
   word i;

   (Format == F_WORD) ? (MSB = 0x8000) : (MSB = 0x80);
   for (i = Format; i > 0; i--)
   {
        if (i == 1)                     // last bit requires TMS=1; TDO one bit before TDI
        {
          ((Data & MSB) == 0) ? TMSH_TDIL_TDOrd() : TMSH_TDIH_TDOrd();
        }
        else
        {
          ((Data & MSB) == 0) ? TMSL_TDIL_TDOrd() : TMSL_TDIH_TDOrd();
        }
        Data <<= 1;
        if (tdo_bit)
            TDOword++;
        if (i > 1)
            TDOword <<= 1;               // TDO could be any port pin
   }
   TMSH_TDIH();                         // update IR
   if (IsTCKHigh() )
   {
        TMSL_TDIH();
   }
   else
   {
        TMSL_TDIL();
   }
   return(TDOword);
}

void TCLKstrobes(word Amount)
{
   word i;

   if (IsTCKHigh())
   {
        TMSLDH
   }                         // TDI = 1 with rising sbwclk
   else
   {
        TMSL
   }

   for (i = Amount; i > 0; i--)
   {
       SetTDIO(0);
       SetTDIO(1);
   }

   TDIH TDOsbw    //ExitTCLK
   TCLK_saved = 1;
}

//----------------------------------------------------------------------------
//! \brief Set the direction for the TDO pin
//! \param dir (0 = IN - from target to REP430F, !0 = OUT)
void TDOI_dir(word dir)
{
    SetTDIODir(dir);
}

//----------------------------------------------------------------------------
//! \brief Set the direction for the TCK pin
//! \param dir (0 = IN - from target to REP430F, !0 = OUT)
void TCK_dir(word dir)
{
    SetTCKDir(dir);
}

//----------------------------------------------------------------------------
//! \brief Set SBW pins to output direction - from REP430F to target
void configure_IO_SBW(void)
{
    TDOI_dir( 1 );
    TCK_dir( 1 );
}

//----------------------------------------------------------------------------
//! \brief Set all JTAG pins to input direction - from target to REP430F
void IO_3state(void)
{
    TDOI_dir( 0 );
    TCK_dir( 0 );
}

//----------------------------------------------------------------------------
//! \brief Set up I/O pins for JTAG communication
void DrvSignals(void)
{
    current_tdio = 1;
    current_tck = 1;
    IO_3state();
    configure_IO_SBW();
}

//----------------------------------------------------------------------------
//! \brief Release I/O pins
void RlsSignals(void)
{
    IO_3state();
}

//----------------------------------------------------------------------------
//! \brief Initialization of the Target Board (switch voltages on, preset JTAG 
//! pins)
//! \details For devices with normal 4wires JTAG  (JTAG4SBW=0)\n
//! For devices with Spy-Bi-Wire to work in 4wires JTAG (JTAG4SBW=1)
byte InitTarget(void)
{
    if(gpio_request(TDIO_GPIO, "tdio") != 0) goto error;
    if(gpio_request(TCK_GPIO, "tck") != 0) goto error;

    DrvSignals();

    return 1;

error:
    ReleaseTarget();
    return 0;
}

//----------------------------------------------------------------------------
//! \brief Release Target Board (switch voltages off, JTAG pins are HI-Z)
void ReleaseTarget(void)
{
    RlsSignals();

    spin_lock_init(&lock);

    gpio_free(TDIO_GPIO);
    gpio_free(TCK_GPIO);
}


/****************************************************************************/
/*                         END OF SOURCE FILE                               */
/****************************************************************************/
