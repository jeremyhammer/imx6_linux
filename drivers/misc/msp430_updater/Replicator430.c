/*==========================================================================*\
|                                                                            |
| Replicator430.c                                                            |
|                                                                            |
| JTAG Replicator for the MSP430 Flash-based family devices                  |
|                                                                            |
| Key features:                                                              |
| � Supports JTAG communication to all MSP430 Flash Devices                  |
| � Max. code size of target program: 57kB                                   |
| � Programming speed: ~60kB/10sec (~6kB/sec)                                |
| � Fast Verification and Erase Check: ~60kB/350ms                           |
| � Supports Programming of JTAG Access Protection Fuse                      |
|                                                                            |
|----------------------------------------------------------------------------|
| Project:              MSP430 Replicator                                    |
| Developed using:      IAR Embedded Workbench 3.40B [Kickstart]             |
|             and:      Code Composer Eessentials 2.0                        |
|----------------------------------------------------------------------------|
| Author:               FRGR                                                 |
| Version:              2.0                                                  |
| Initial Version:      04-17-02                                             |
| Last Change:          07-23-09                                             |
|----------------------------------------------------------------------------|
| Version history:                                                           |
| 1.0 04/02 FRGR        Initial version.                                     |
| 1.1 04/02 FRGR        Included SPI mode to speed up shifting function by 2.|
| 1.2 06/02 ALB2        Formatting changes, added comments.                  |
| 1.3 06/02 ALB2        DEVICE ID info added.                                |
| 1.4 08/02 ALB2        Initial code release with Lit# SLAA149.              |
| 1.5 01/03 ALB2        Unnecessary lines have been commented out(see below).|
| 1.6 09/05 JDI         converted to IAR EW 3.30. F2xxx support added        |
|           SUN1        Software delays redesigned to use TimerA harware;    |
|                       see MsDelay() routine.                               |
|           ALB2        Added 2xx Info memory B, C, & D erase & check        |
| 1.7 12/05 STO         Adapted for 2xx devices with SpyBiWire               |
| 1.8 02/06 STO         Minor cosmetic changes                               |
| 1.9 05/06 STO         Minor cosmetic changes                               |
| 2.0 04/07 WLUT        Minor changes at declarations and calls              |
|                       according to use srec_cat.exe                        |
| 2.1 07/09 FB          Added loop for polling P1.6 / added support for      |
|                       replicator                                           |
| 2.2 07/12 RL          Updated commentaries                                 |
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

//! \file Replicator430.c
//! \brief JTAG Replicator for the MSP430 Flash-based family.
//! \author Florian Berenbrinker
//! \date 07/03/2012
//! \version 2.2

/****************************************************************************/
/* Main section of Replicator program: User can modify/insert code as needed*/
/****************************************************************************/
/* Main function:
   Upon execution completion, LED blinks for F1xx & F4xx device target socket
   boards when used as the target system (P1.0 and P5.1 drive LEDs).

   Note: All High Level JTAG Functions are applied here.

   A number of lines have been commented out which are not strictly required
   to successfully program a flash-based MSP430 device using the Replicator
   concept. Please note that these lines have been commented out in order
   to provide reference examples for calling such functions.
   Uncommented lines below represent the minimum requirements to successfully
   program an MSP430 flash device using the Replicator.
   
    The basic routine consists of the following steps:
   
   1.  | Initialize the host MSP430 on the Replicator board
   2.  | Connect to the target device
   3.  | Control the target peripherals directly (optional)
   4.  | Perform a write + read + verify in the target RAM
   5.  | Operations in the device's main memory
   6.  | Blow JTAG access fuse (optional)
   7.  | Release the device from JTAG control and wait for the user to press button "S1"
*/

/****************************************************************************/
/* Includes                                                                 */
/****************************************************************************/

#include "msp430_updater.h"

#include "JTAGfunc430.h"           // JTAG functions
#include "LowLevelFunc430.h"       // low level functions
#include "Devices430.h"            // holds Device specific information

/****************************************************************************/
/* FUNCTIONS                                                                */
/****************************************************************************/

uint32_t saved_status = STATUS_IDLE;

uint32_t GetUpdateStatus(void) {
    return saved_status;
}

void SetUpdateStatus(word status, int line)
{
    saved_status = status;
    switch (status)
    {
        case STATUS_ERROR:
            printk("Status(%d): ERROR\n", line);
            break;
        case STATUS_ACTIVE:
            printk("Status(%d): ACTIVE\n", line);
            break;
        case STATUS_OK:
            printk("Status(%d): OK\n", line);
            break;
        case STATUS_IDLE:
            printk("Status(%d): IDLE\n", line);
            break;
        default:
            printk("Status(%d): UNKOWN - %d\n", line, status);
            break;
    }
}

#define SetStatus(s) SetUpdateStatus(s, __LINE__)



//! \brief The basic Replicator routine
//! \details This function is executed once at startup and can be restarted 
//! by pressing button "S1" on the REP430F board.
void do_msp430_update(msp430_update_data_t* update_data)
{
    //! \brief Data pointer
    word p;
    //! \brief Buffer, used for memory read with ReadMemQuick()
    word* ReadArray = vmalloc(8192);

/*------------------------------------------------------------------------------------------------------*/
/*  1. | Initialize host MSP430 (on Replicator board) & target board                                    */
/*------------------------------------------------------------------------------------------------------*/    
    
    SetStatus(STATUS_ACTIVE);         // Switch both LEDs on to indicate operation.

    InitTarget();                         // Initialize target board

/*------------------------------------------------------------------------------------------------------*/
/*  2. | Connect to the target device                                                                   */
/*------------------------------------------------------------------------------------------------------*/    
    
    if (GetDevice() != STATUS_OK)         // Set DeviceId
    {
        SetStatus(STATUS_ERROR);      // stop here if invalid JTAG ID or
        goto end;
    }                                     // time-out. (error: red LED is ON)
    
/*------------------------------------------------------------------------------------------------------*/
/*  3. | Control the target peripherals directly                                                        */
/*------------------------------------------------------------------------------------------------------*/

    // Remove the following comments to toggle Pin 1.0 (i.e flash the LED for MSP430 target socket boards)
    /*{
        word k;
        
        WriteMem(F_BYTE, 0x21, 0x01);         // P1.0 for F1xx,2xx devices
        WriteMem(F_BYTE, 0x31, 0x02);         // P5.1 for F4xx devices
    
        for(k = 0; k < 3; k++)
        {
            WriteMem(F_BYTE, 0x22, 0x01);
            WriteMem(F_BYTE, 0x32, 0x02);
            MsDelay(500);                     // LED on for 0.5s
            WriteMem(F_BYTE, 0x21, 0x00);
            WriteMem(F_BYTE, 0x31, 0x00);
            MsDelay(500);                     // LED off for 0.5s    
        }
    }*/

/*------------------------------------------------------------------------------------------------------*/
/*  4. | Perform a write + read + verify in the target RAM                                              */
/*------------------------------------------------------------------------------------------------------*/

    // Communication test: Write 2 RAM bytes
    WriteMem(F_BYTE, 0x0200, 0x34);
    WriteMem(F_BYTE, 0x0201, 0x12);
    // Read back word
    if (ReadMem(F_WORD, 0x0200) != 0x1234)
    {
        SetStatus(STATUS_ERROR);
        goto end;
    }
    
    // Write RAM word
    WriteMem(F_WORD, 0x0202, 0x5678);
    // Read back 2 bytes
    if (ReadMem(F_BYTE, 0x0202) != 0x78)
    {
        SetStatus(STATUS_ERROR);
        goto end;
    }
    if (ReadMem(F_BYTE, 0x0203) != 0x56)
    {
        SetStatus(STATUS_ERROR);
        goto end;
    }

    // Is the Read/WriteQuick for current device possible?
    if (DeviceHas_DataQuick())
    {
        word test_data[0x10] = {0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009, 0x000A};
        // Write RAM block
        WriteMemQuick (0x0200, 0x0010, (word*)&test_data[0]);
        // Verify(PSA) RAM of target
        if (!VerifyMem(0x0200, 0x0010, (word*)&test_data[0]))
        {
            SetStatus(STATUS_ERROR);
            goto end;
        }
        // Read RAM block
        ReadMemQuick  (0x0200, 0x0010, &ReadArray[0]);
        // Verify(word-for-word) RAM of target
        for (p = 0; p < 0x0010; p++)
        {
            if (ReadArray[p] != test_data[p])
            {
                SetStatus(STATUS_ERROR);
                goto end;
            }
        }
    }
    
/*------------------------------------------------------------------------------------------------------*/
/*  5. | Operations in the device's main memory                                                         */
/*------------------------------------------------------------------------------------------------------*/
    
    // The following section is not required and included only for reference as to the
    // manner in which a single flash memory segment can be erased. The more common
    // "mass erase" is used to prepare the target device for replicator programming.
    /*{
        // Segment 0 erase Flash (all types)
        EraseFLASH(ERASE_SGMT, 0xFE00);
        // Check segment 0 memory erasure
        if (!EraseCheck(0xFE00, 0x0100))
        {
            SetStatus(STATUS_ERROR);
            goto end;
        }
    }*/

    // Perform a mass erase  
    if (DeviceHas_CpuX())
    {
        EraseFLASH(ERASE_GLOB, 0xFE00);     // Global-Erase Flash
    }                                       // (for all devices with CPU-X)
    else
    {
        EraseFLASH(ERASE_MASS, 0xFE00);     // Mass-Erase Flash (all types)
        // NOTE: the INFO memory in F2xx device will be not erased,
        // if the memory is locked. For more info See EraseFLASH() in JTAGfunc430.c
    }

    //if (!EraseCheck(0x1000, 0x0040))      // Check info memory erasure (Fxx2..9)
    //{
    //    SetStatus(STATUS_ERROR);
    //    goto end;
    //}

    if (!EraseCheck(0xF800, 0x0400))        // Check main memory erasure (Fxx2..9)
    {
        SetStatus(STATUS_ERROR);
        goto end;
    }

    /* For all 1xx & 4xx & 2xx devices, where ALL Flash has been erased*/
    EraseFLASH(ERASE_SGMT, 0xFE00);
    if (!EraseCheck(0xfe00, 0x0100))      // Check part of main memory erasure (Fxx2..9)
    {
        SetStatus(STATUS_ERROR);
        goto end;
    }

    // For all 2xx devices, where Info A was not erased (Info A keeps calibration data)
    /*
    // Check info memory erasure (Fxx2..9)
    if (!EraseCheck(0x1000, 0x0080))
    {  
        SetStatus(STATUS_ERROR);
        goto end;
    }
    */

    // Following section shows how to erase Info-Segments on 2xx Devices selectively
    // Info A will not be erased as it contains calibration data
    {
        // Info B erase (2xx)
        EraseFLASH(ERASE_SGMT, 0x1080);
        // Check Info B memory erasure (2xx)
        if (!EraseCheck(0x1080, 0x0020))
        {
            SetStatus(STATUS_ERROR);
            goto end;
        }
        
        // Info C erase (2xx)
        EraseFLASH(ERASE_SGMT, 0x1040);
        // Check Info C memory erasure (2xx)
        if (!EraseCheck(0x1040, 0x0020))
        {
            SetStatus(STATUS_ERROR);
            goto end;
        }
        
        // Info D erase (2xx)
        EraseFLASH(ERASE_SGMT, 0x1000);
        // Check Info D memory erasure (2xx)
        if (!EraseCheck(0x1000, 0x0020))
        {
            SetStatus(STATUS_ERROR);
            goto end;
        }
    }

    // Program target code
    if (!WriteFLASHallSections(update_data->data, Device_MainStart(), update_data->length))
    {
        SetStatus(STATUS_ERROR);
        goto end;
    }

    printk("Data written, verifying\n");
    // Read RAM block
    ReadMemQuick(Device_MainStart(), update_data->length, &ReadArray[0]);
    // Verify(word-for-word) RAM of target
    for (p = 0; p < update_data->length; p++)
    {
        printk("%04X == %04X\n", ReadArray[p], update_data->data[p]);
        if (ReadArray[p] != update_data->data[p])
        {
            printk("Failed to verify\n");
            SetStatus(STATUS_ERROR);
            goto end;
        }
    }

    printk("Data verified\n");

/*------------------------------------------------------------------------------------------------------*/
/*  6. | Blow the JTAG access protection fuse                                                           */
/*------------------------------------------------------------------------------------------------------*/
     
    // Remove following comments to enable the JTAG fuse blow routine.
    // This makes the MSP430 device permanently inaccessible via JTAG

    /*if (!BlowFuse())              // ***Action is permanent***
    {
        SetStatus(STATUS_ERROR);
        goto end;
        }*/

    /*------------------------------------------------------------------------------------------------------*/
    /*  7. | Release the target device from JTAG control and wait for the user to press button "S1"         */
    /*------------------------------------------------------------------------------------------------------*/

    ReleaseDevice(V_RESET);         // Perform Reset, release CPU from JTAG control
    // Target board LED should start blinking
    SetStatus(STATUS_OK);       // OK: green LED is ON
    
end:
    vfree(ReadArray);
    ReleaseTarget();
}

/****************************************************************************/
/*                         END OF SOURCE FILE                               */
/****************************************************************************/
