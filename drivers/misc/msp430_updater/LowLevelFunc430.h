#ifndef LOWLEVELFUNC430_H
#define LOWLEVELFUNC430_H
/*==========================================================================*\
|                                                                            |
| LowLevelFunc430.h                                                          |
|                                                                            |
| Low Level function prototypes, macros, and pin-to-signal assignments       |
| regarding to user's hardware                                               |
|----------------------------------------------------------------------------|
| Project:              MSP430 Replicator                                    |
| Developed using:      IAR Embedded Workbench 3.40B [Kickstart]             |
|             and:      Code Composer Eessentials 2.0                        |
|----------------------------------------------------------------------------|
| Author:               FRGR                                                 |
| Version:              1.9                                                  |
| Initial Version:      04-17-02                                             |
| Last Change:          05-24-12                                             |
|----------------------------------------------------------------------------|
| Version history:                                                           |
| 1.0 04/02 FRGR        Initial version.                                     |
| 1.1 04/02 FRGR        Included SPI mode to speed up shifting function by 2.|
|                       (JTAG control now on Port5)                          |
| 1.2 06/02 ALB2        Formatting changes, added comments. Removed code used|
|                       for debug purposes during development.               |
| 1.3 08/02 ALB2        Initial code release with Lit# SLAA149.              |
| 1.4 09/05 SUN1        Software delays redesigned to use TimerA harware;    |
|                       see MsDelay() routine. Added TA constant.            |
| 1.5 12/05 STO         Added RESET pin definition                           |
| 1.6 08/08 WLUT        Added DrvSignals and RlsSignals macros for clean     |
|                       JTAG init sequence.                                  |
| 1.7 05/09 GC (Elprotronic)  Added support for the new hardware - REP430F   |
| 1.8 07/09 FB          Added macros for Spy-Bi-Wire support                 |
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

//! \file LowLevelFunc430.h
//! \brief Low Level function prototypes, macros, and pin-to-signal assignments regarding to user's hardware
//! \author Florian Berenbrinker
//! \author Gregory Czajkowski (Elprotronic)
//! \date 05/24/2011
//! \version 1.9

/****************************************************************************/
/* INCLUDES                                                                 */
/****************************************************************************/

#include "Config430.h"        // High-level user input

/****************************************************************************/
/* DEFINES & CONSTANTS                                                      */
/****************************************************************************/

//! \brief JTAG interface
#define JTAG_IF             1
//! \brief JTAG interface on a device that supports JTAG and SBW
#define SPYBIWIREJTAG_IF    2
//! \brief Spy-Bi-Wire interface
#define SPYBIWIRE_IF        3

#if( INTERFACE == SPYBIWIRE_IF )
#define SPYBIWIRE_MODE 
#endif

#ifndef __DATAFORMATS__
#define __DATAFORMATS__
#define F_BYTE                     8
#define F_WORD                     16
#define F_ADDR                     20
#define F_LONG                     32
#endif

// Constants for runoff status
//! \brief return 0 = error
#define STATUS_ERROR     0      // false
//! \brief return 1 = no error
#define STATUS_OK        1      // true
//! \brief GetDevice returns this if the security fuse is blown
#define STATUS_FUSEBLOWN 2

//! \brief Replicator is active
#define STATUS_ACTIVE    3
//! \brief Replicator is idling
#define STATUS_IDLE      4

/*----------------------------------------------------------------------------
   Definition of global variables
*/
extern byte TCLK_saved;      // holds the last value of TCLK before entering a JTAG sequence

/****************************************************************************/
/* TYPEDEFS                                                                 */
/****************************************************************************/

#ifndef __BYTEWORD__
#define __BYTEWORD__
typedef unsigned short word;
typedef unsigned char byte;
#endif

/****************************************************************************/
/* FUNCTION PROTOTYPES                                                      */
/****************************************************************************/

void    ClrTCLK(void);
void    SetTCLK(void);
void    ClrTST(void);
void    SetTST(void);
void    ClrRST(void);
void    SetRST(void);

byte    IsTCKHigh(void);
void    ClrTCLK_sbw(void);
void    SetTCLK_sbw(void);
void    TMSL_TDIL(void);
void    TMSH_TDIL(void);
void    TMSL_TDIH(void);
void    TMSH_TDIH(void);
void    TMSL_TDIH_TDOrd(void);
void    TMSL_TDIL_TDOrd(void);
void    TMSH_TDIH_TDOrd(void);
void    TMSH_TDIL_TDOrd(void);
void    MsDelay(word milliseconds);      // millisecond delay loop, uses Timer_A
void    usDelay(word microeconds);       // microsecond delay loop, uses nops
void    InitController(void);
byte    InitTarget(void);
void    ReleaseTarget(void);
word    Shift(word Format, word Data);   // used for IR- as well as DR-shift
void    TDOisInput(void);
void    TCLKstrobes(word Amount);
void    DrvSignals( void );
void    RlsSignals( void );
void    TDI_dir( word dir );
void    TDOI_dir( word dir );
void    TEST_dir( word dir );
void    TMS_dir( word dir );
void    RST_dir( word dir );
void    TCK_dir( word dir );

void    configure_IO_JTAG( void );
void    configure_IO_BSL( void );
void    IO_3state( void );
void    configure_IO_SBW( void );

#endif //LOWLEVELFUNC430_H
