/****************************************************************************
 *
 * MODULE:             Utilities Code
 *
 * COMPONENT:          $RCSfile: Utils.c,v $
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.2 $
 *
 * DATED:              $Date: 2008-05-22 13:51:28 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:
 *
 * DESCRIPTION:
 *
 *
 * LAST MODIFIED BY:   $Author: thayd $
 *                     $Modtime: $
 *
 ****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2008. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include "jendefs.h"
#include <AppHardwareApi.h>
#include "Utils.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/


/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vUtils_Init
 *
 * DESCRIPTION:
 * Initialises UART
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * void, never returns
 *
 ****************************************************************************/
PUBLIC void vUtils_Init(void)
{
    vAHI_UartEnable(UART);
    vAHI_UartReset(UART, TRUE, TRUE);
    vAHI_UartReset(UART, FALSE, FALSE);
    vAHI_UartSetClockDivisor(UART, UART_BAUD_RATE);
}

/****************************************************************************
 *
 * NAME: vUtils_WriteChar
 *
 * DESCRIPTION:
 * Sends a character to the UART
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8chr           R   Character to send
 *
 * RETURNS:
 * void, never returns
 *
 ****************************************************************************/
PUBLIC void vUtils_WriteChar(char u8chr)
{
    vAHI_UartWriteData(UART,u8chr);
    while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
}
/****************************************************************************
 *
 * NAME: vUtils_DisplayMsg
 *
 * DESCRIPTION:
 * Used to display text plus number
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcMessage       R   Message to display
 *                  u32Data         R   Data to display
 *
 * RETURNS:
 * void, never returns
 *
 ****************************************************************************/
PUBLIC void vUtils_DisplayMsg(char *pcMessage, uint32 u32Data)
{
    vUtils_Debug(pcMessage);
    vUtils_DisplayHex(u32Data, 8);
}

/****************************************************************************
 *
 * NAME: vUtils_Debug
 *
 * DESCRIPTION:
 * Sends a string to UART0 using the hardware API with CR
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcMessage       R   Null-terminated message to send
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUtils_Debug(char *pcMessage)
{
    vUtils_String(pcMessage);

    vAHI_UartWriteData(UART, '\r');
    while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);

    vAHI_UartWriteData(UART, '\n');
    while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
}

/****************************************************************************
 *
 * NAME: vUtils_String
 *
 * DESCRIPTION:
 * Sends a string to UART0 using the hardware API without CR
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcMessage       R   Null-terminated message to send
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUtils_String(char *pcMessage)
{
    while (*pcMessage)
    {
        while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
        vAHI_UartWriteData(UART, *pcMessage);
        pcMessage++;
    }
}

/****************************************************************************
 *
 * NAME: vUtils_DisplayHex
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32Data         R   Value to send
 *                  iSize           R   Length of value
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUtils_DisplayHex(uint32 u32Data, int iSize)
{
    char acValue[9];

    vUtils_ValToHex(acValue, u32Data, 8);
    vUtils_Debug(acValue);
}

/****************************************************************************
 *
 * NAME: vUtils_DisplayDec
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32Data         R   Value to send
 *                  iSize           R   Length of value
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUtils_DisplayDec(uint8 u8Data)
{
    char acValue[3];

    vUtils_ValToDec(acValue, u8Data);
    vUtils_Debug(acValue);
}

/****************************************************************************
 *
 * NAME: vUtils_DisplayHex
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32Data         R   Value to send
 *                  iSize           R   Length of value
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUtils_ValToHex(char *pcString, uint32 u32Data, int iSize)
{
    uint8 u8Nybble;
    int i, j;

    j = 0;
    for (i = (iSize << 2) - 4; i >= 0; i -= 4)
    {
        u8Nybble = (uint8)((u32Data >> i) & 0x0f);
        u8Nybble += 0x30;
        if (u8Nybble > 0x39)
            u8Nybble += 7;

        *pcString = u8Nybble;
        pcString++;
    }
    *pcString = '\0';
}

/****************************************************************************
 *
 * NAME: vUtils_ValToDec
 *
 * DESCRIPTION:
 * Converts an 8-bit value to a string of the textual decimal representation.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcOutString     R   Location for new string
 *                  u8Value         R   Value to convert
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUtils_ValToDec(char *pcOutString, uint8 u8Value)
{
    static const uint8 au8Digits[3] = {100, 10, 1};
    uint8 u8Digit;
    uint8 u8DigitIndex;
    uint8 u8Count;
    bool_t boPreviousDigitPrinted = FALSE;

    for (u8DigitIndex = 0; u8DigitIndex < 3; u8DigitIndex++)
    {
        u8Count = 0;
        u8Digit = au8Digits[u8DigitIndex];
        while (u8Value >= u8Digit)
        {
            u8Value -= u8Digit;
            u8Count++;
        }

        if ((u8Count != 0) || (boPreviousDigitPrinted == TRUE)
            || (u8DigitIndex == 2))
        {
            *pcOutString = '0' + u8Count;
            boPreviousDigitPrinted = TRUE;
            pcOutString++;
        }
    }
    *pcOutString = '\0';
}

PUBLIC void vUtils_DisplayBytes(uint8 *pcOutString, uint8 u8Num)
{
uint8 chr;
    while(u8Num--)
    {
        chr=(*pcOutString >> 4)+0x30;
        if(chr > 0x39)
        {
            chr+=0x07;
        }
        vAHI_UartWriteData(UART,chr);
        while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
        chr=(*pcOutString & 0x0f)+0x30;
        if(chr > 0x39)
        {
            chr+=0x07;
        }
        vAHI_UartWriteData(UART,chr);
        while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
        vAHI_UartWriteData(UART,' ');
        while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
        pcOutString++;
    }
    vAHI_UartWriteData(UART, '\r');
    while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
    vAHI_UartWriteData(UART, '\n');
    while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
}
/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
