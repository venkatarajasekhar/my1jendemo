/****************************************************************************
 *
 * MODULE:             Utils
 *
 * COMPONENT:          $RCSfile: Utils.h,v $
 *
 * VERSION:            $Name: $
 *
 * REVISION:           $Revision: 1.2 $
 *
 * DATED:              $Date: 2008-05-22 13:51:28 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             GPfef
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

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif  //__cplusplus

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <AppHardwareApi.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Define which of the two available hardware UARTs and what baud rate to use */
#define UART                    E_AHI_UART_0

#define UART_BAUD_RATE          E_AHI_UART_RATE_19200

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void vUtils_Init(void);
PUBLIC void vUtils_DisplayHex(uint32 u32Data, int iSize);
PUBLIC void vUtils_DisplayDec(uint8 u8Data);
PUBLIC void vUtils_Debug(char *pcMessage);
PUBLIC void vUtils_DisplayMsg(char *pcMessage, uint32 u32Data);
PUBLIC void vUtils_String(char *pcMessage);
PUBLIC void vUtils_ValToHex(char *pcOutString, uint32 u32Data, int iSize);
PUBLIC void vUtils_ValToDec(char *pcOutString, uint8 u8Value);
PUBLIC void vUtils_DisplayBytes(uint8 *pcOutString, uint8 u8Num);
PUBLIC void vUtils_WriteChar(char u8chr);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#ifdef __cplusplus
}
#endif  //__cplusplus

#endif  // UTILS_H

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/


