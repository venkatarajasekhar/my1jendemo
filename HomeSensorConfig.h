/*****************************************************************************
 *
 * MODULE:              JenNet Home Sensor Demo
 *
 * COMPONENT:           $RCSfile: HomeSensorConfig.h,v $
 *
 * VERSION:             $Name: $
 *
 * REVISION:            $Revision: 1.5 $
 *
 * DATED:               $Date: 2008-05-22 13:51:28 $
 *
 * STATUS:              $State: Exp $
 *
 * AUTHOR:              CJG
 *
 * DESCRIPTION:
 * Configuration settings for demo system
 *
 * LAST MODIFIED BY:    $Author: thayd $
 *                      $Modtime: $
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

#ifndef  DEMO_CONFIG_INCLUDED
#define  DEMO_CONFIG_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "jendefs.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Number of points in graph of previous values. Must be multiple of 2 */
#define DEMO_HISTORY_LEN                  32
/* Number of endpoints in system */
#define DEMO_ENDPOINTS                    4
/* First byte of payload from coordinator, as frame identity */
#define DEMO_BEACON_IDENTIFIER            0xb5
/* First byte of payload from endpoint, as frame identity */
#define DEMO_ENDPOINT_MESSAGE_ID          0x5b
#define DEMO_ENDPOINT_JOIN_ID             0x5c
/* PAN ID on which demo operates */
#define DEMO_PAN_ID                       0x0e1c
/* Coordinator short address */
#define DEMO_COORD_ADDR                   0x0e00
/* Endpoint short address base */
#define DEMO_ENDPOINT_ADDR_BASE           0x0e01
/* Channels available */
#define CHANNEL_MIN                       11
#define CHANNEL_MID                       18
#define CHANNEL_MAX                       26
#define DEMO_CHANNEL_BITMAP               0x7fff800

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum
{
    E_SENSOR_TEMP = 0,
    E_SENSOR_HTS,
    E_SENSOR_ALS,
    DEMO_SENSOR_LIST_LEN
} teSensor;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* DEMO_CONFIG_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

