/****************************************************************************
 *
 * MODULE:             JenNet Home Sensor Demo
 *
 * COMPONENT:          $RCSfile: HomeSensorRouter.c,v $
 *
 * VERSION:            $Name: $
 *
 * REVISION:           $Revision: 1.6 $
 *
 * DATED:              $Date: 2009-08-06 09:27:46 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             $Author: pjtw $
 *
 * DESCRIPTION:
 *
 * LAST MODIFIED BY:   thayd
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

#include <jendefs.h>
#include <AppHardwareApi.h>
#include <string.h>
#include "AlsDriver.h"
#include "HtsDriver.h"
#include "HomeSensorConfig.h"
#include "Button.h"
#include "LedControl.h"
#include "Jenie.h"
#include "Utils.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Timing values */
#define RUNNING_FLASH_RATE      4
#define RUNNING_TRANSMIT_RATE   5
#define REGISTER_FLASH_RATE     10
#define DELAY_PERIOD            3200

/* define LED positions  */
#define LED1                    0
#define LED2                    1

/* define if using high power modules */
/* #define HIGH_POWER */

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/* Button values */
typedef enum
{
    E_KEY_0 = BUTTON_0_MASK,
    E_KEY_1 = BUTTON_1_MASK
} teKeyValues;

/* All application data with scope within the entire file is kept here, */
typedef struct
{
    uint64 u64DestAddr;
    uint64 u64ParentAddr;
    bool_t bAppTimerStarted;
    bool_t bStackReady;
    uint8 eAppState;
} tsHomeData;

typedef enum
{
    E_STATE_OFF,
    E_STATE_REGISTER,
    E_STATE_RUNNING
}teAppState;
/* All variables with scope throughout module are in one structure */
typedef struct
{
    /* Transceiver (basically anything TX/RX not covered elsewhere) */
    struct
    {
        uint8   u8CurrentTxHandle;
        uint8   u8PrevRxBsn;
    } sTransceiver;

    /* Controls (switch, light level alarm) */
    struct
    {
        uint8   u8Switch;
        uint8   u8LightAlarmLevel;
    } sControls;

    /* Sensor data, stored between read and going out in frame */
    struct
    {
        uint8   u8TempResult;
        uint8   u8HtsResult;
        uint8   u8AlsResult;
    } sSensors;

    /* Settings specific to light sensor */
    struct
    {
        uint16 u16Hi;
        uint16 u16Lo;
    } sLightSensor;

    /* System (state, assigned address, channel) */
    struct
    {
        teAppState eState;
        uint16  u16ShortAddr;
        uint8   u8ThisNode;
        uint8   u8Channel;
    } sSystem;
} tsDemoData;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/* File scope data */
PRIVATE tsHomeData sHomeData;
PRIVATE tsDemoData sDemoData;

PRIVATE bool_t bTimeOut;

/* Routing table storage */
PRIVATE tsJenieRoutingTable asRoutingTable[100];

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vProcessTxData(void);
PRIVATE void vTxRegister(void);
PRIVATE void vInitEndpoint(void);
PRIVATE void vProcessRead(void);
PRIVATE uint8 u8FindMin(uint8 u8Val1, uint8 u8Val2);

/* Stack to application callback functions */
/****************************************************************************
 *
 * NAME: vJenie_ConfigureNetwork
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Initialises system.
 *
 * RETURNS:
 * Nothing
 *
 ****************************************************************************/
PUBLIC void vJenie_CbConfigureNetwork(void)
{
    /* Set PAN_ID and other network stuff or defaults will be used */
    gJenie_NetworkApplicationID =   0xdeaddead;
    gJenie_PanID                =   DEMO_PAN_ID;

    /* Configure stack with routing table data */
    gJenie_RoutingEnabled    = TRUE;
    gJenie_RoutingTableSize  = 100;
    gJenie_RoutingTableSpace = (void *)asRoutingTable;
}

PUBLIC void vJenie_CbInit(bool_t bWarmStart)
{

    vUtils_Init();

    if(bWarmStart==FALSE)
    {
        (void)u32AHI_Init();
        sHomeData.bStackReady=FALSE;
        /* Initialise buttons, LEDs and program variables */
        vInitEndpoint();
        /* Set DIO for buttons and LEDs */
        vLedControl(LED1, FALSE);
        vLedControl(LED2, FALSE);
        vLedInitRfd();
        vButtonInitRfd();

        vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_1, TRUE);

        /* Set SW1(dio9) to input */
        vAHI_DioSetDirection(E_AHI_DIO9_INT, 0);
        /* set interrupt for DIO9 to occur on button release - rising edge */
        vAHI_DioInterruptEdge(E_AHI_DIO9_INT, 0);
        /* enable interrupt for DIO9 */
        vAHI_DioInterruptEnable(E_AHI_DIO9_INT, 0);

        /* Set SW2(dio10) to input */
        vAHI_DioSetDirection(E_AHI_DIO10_INT, 0);
        /* set interrupt for DIO9 to occur on button release - rising edge */
        vAHI_DioInterruptEdge(E_AHI_DIO10_INT, 0);
        /* enable interrupt for DIO9 */
        vAHI_DioInterruptEnable(E_AHI_DIO10_INT, 0);

        /* Set up peripheral hardware */
        vALSreset();
        vHTSreset();

        /* Start ALS now: it automatically keeps re-sampling after this */
        vALSstartReadChannel(0);

        sHomeData.eAppState = E_STATE_REGISTER;
        switch(eJenie_Start(E_JENIE_ROUTER))        /* Start network as router */
        {
        case E_JENIE_SUCCESS:
            #ifdef DEBUG
                vUtils_Debug("Jenie Started");
            #endif
            #ifdef HIGH_POWER
                /* Set high power mode */
                eJenie_RadioPower(18, TRUE);
            #endif
            break;

        case E_JENIE_ERR_UNKNOWN:
        case E_JENIE_ERR_INVLD_PARAM:
        case E_JENIE_ERR_STACK_RSRC:
        case E_JENIE_ERR_STACK_BUSY:

        default:
            /* Do something on failure?? */
            break;
        }
    }
    else
    {
        #ifdef DEBUG
            vUtils_String("Warm Start?");
        #endif
    }
}

/****************************************************************************
 *
 * NAME: vJenie_Main
 *
 * DESCRIPTION:
 * Main user routine. This is called by the Basic Operating System (BOS)
 * at regular intervals.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vJenie_CbMain(void)
{
    static bool phase=0;
    static uint32 loop_count;

    /* regular watchdog reset */
    #ifdef WATCHDOG_ENABLED
       vAHI_WatchdogRestart();
    #endif

    if(sHomeData.bStackReady && bTimeOut)       // Stack up and running and waiting for us to do something
    {
        switch(sHomeData.eAppState)
        {
        case E_STATE_REGISTER:
            if(loop_count % REGISTER_FLASH_RATE == 0)
            {
                vTxRegister();
                vLedControl(LED1,phase);
                phase ^= 1;
            }
            break;

        case E_STATE_RUNNING:
            vProcessRead();
            if(loop_count % RUNNING_FLASH_RATE == 0)
            {
                vLedControl(LED1,phase);
                phase ^= 1;
            }
            if(loop_count % RUNNING_TRANSMIT_RATE == 0)
            {
                vProcessTxData();
            }
            break;

        default:
            #ifdef DEBUG
                vUtils_Debug("Unknown State");
            #endif
            break;
        }
        loop_count--;

        vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_1, DELAY_PERIOD);
        bTimeOut = FALSE;
    }


}

/****************************************************************************
 *
 * NAME: vJenie_StackMgmtEvent
 *
 * DESCRIPTION:
 * Used to receive stack management events
 *
 * PARAMETERS:      Name                    RW  Usage
 *                  *psStackMgmtEvent       R   Pointer to event structure
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vJenie_CbStackMgmtEvent(teEventType eEventType, void *pvEventPrim)
{
    switch(eEventType)
    {
    case E_JENIE_NETWORK_UP:
        sHomeData.u64ParentAddr = ((tsNwkStartUp*)pvEventPrim)->u64ParentAddress;
        #ifdef DEBUG
            vUtils_DisplayMsg("New parent:",(uint32)sHomeData.u64ParentAddr);
            vUtils_Debug("Network Up");
        #endif
        sHomeData.bStackReady = TRUE;
        eJenie_SetPermitJoin(TRUE);
        bTimeOut=TRUE;
        break;

    case E_JENIE_REG_SVC_RSP:
        break;

    case E_JENIE_SVC_REQ_RSP:
        break;

    case E_JENIE_POLL_CMPLT:
        break;

    case E_JENIE_PACKET_SENT:
        break;

    case E_JENIE_PACKET_FAILED:
        break;

    case E_JENIE_CHILD_JOINED:
        #ifdef DEBUG
            vUtils_DisplayMsg("Child Joined: ",(uint32)(((tsChildJoined*)pvEventPrim)->u64SrcAddress));
        #endif
        break;

    case E_JENIE_CHILD_LEAVE:
        #ifdef DEBUG
            vUtils_DisplayMsg("Child Left: ",(uint32)(((tsChildLeave*)pvEventPrim)->u64SrcAddress));
        #endif
        break;

    case E_JENIE_STACK_RESET:
        #ifdef DEBUG
            vUtils_Debug("Stack Reset");
        #endif
        sHomeData.bStackReady = FALSE;
        sHomeData.eAppState = E_STATE_REGISTER;
        eJenie_SetPermitJoin(FALSE);
        break;

    default:
        /* Unknown management event type */
        #ifdef DEBUG
            vUtils_Debug("Unknown Management Event");
        #endif
        break;
    }
}





/****************************************************************************
 *
 * NAME: vJenie_StackDataEvent
 *
 * DESCRIPTION:
 * Used to receive stack data events
 *
 * PARAMETERS:      Name                    RW  Usage
 *                  *psStackDataEvent       R   Pointer to data structure
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vJenie_CbStackDataEvent(teEventType eEventType, void *pvEventPrim)
{
    switch(eEventType)
    {
    case E_JENIE_DATA:
        break;

    case E_JENIE_DATA_TO_SERVICE:
        break;

    case E_JENIE_DATA_ACK:
        /* Update current state on success*/
        if (sHomeData.eAppState == E_STATE_REGISTER)
        {
            #ifdef DEBUG
                vUtils_Debug("Registered");
            #endif
            sHomeData.eAppState = E_STATE_RUNNING;
        }
    break;

    case E_JENIE_DATA_TO_SERVICE_ACK:
        break;

    default:
        /*Unknown data event type */
        #ifdef DEBUG
            vUtils_Debug("Unknown Data Event");
        #endif
        break;
    }
}


/****************************************************************************
 *
 * NAME: vProcessTxData
 *
 * DESCRIPTION:
 * Assembles and requests transmission of sensor data
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vProcessTxData(void)
{
    uint8 au8Payload[8];
    au8Payload[0] = DEMO_ENDPOINT_MESSAGE_ID;
    au8Payload[1] = sDemoData.sTransceiver.u8PrevRxBsn;
    au8Payload[2] = sDemoData.sControls.u8Switch;
    au8Payload[3] = sDemoData.sSensors.u8TempResult;
    au8Payload[4] = sDemoData.sSensors.u8HtsResult;
    au8Payload[5] = sDemoData.sSensors.u8AlsResult;
    au8Payload[6] = 0;
    au8Payload[7] = 0;
    eJenie_SendData(0ULL,au8Payload,8,0);

}


/****************************************************************************
 *
 * NAME: vTxRegister
 *
 * DESCRIPTION:
 * Requests transmission of registration message
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vTxRegister(void)
{
    uint8 au8Payload[1];
    au8Payload[0] = DEMO_ENDPOINT_JOIN_ID;
    eJenie_SendData(0ULL,au8Payload,1,TXOPTION_ACKREQ);
}

/****************************************************************************
 *
 * NAME: vJenie_HwEvent
 *
 * DESCRIPTION:
 * Adds events to the hardware event queue.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32Device       R   Peripheral responsible for interrupt e.g DIO
 *                  u32ItemBitmap   R   Source of interrupt e.g. DIO bit map
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vJenie_CbHwEvent(uint32 u32DeviceId,uint32 u32ItemBitmap)
{
    /* Not used in this application */
    if ((u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & E_AHI_DIO9_INT))
    {
        sDemoData.sControls.u8Switch = 0;
    }
    else if((u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & E_AHI_DIO10_INT))
    {
        sDemoData.sControls.u8Switch = 1;
    }
    else if ( (u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & E_AHI_SYSCTRL_WK1_MASK) )
    {
        bTimeOut = TRUE;
    }
}

/****************************************************************************
 *
 * NAME: vInitEndpoint
 *
 * DESCRIPTION:
 * Initialises the nodes state
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vInitEndpoint(void)
{
    /* Set defaults for software */
    sDemoData.sControls.u8Switch = 0;
    sDemoData.sControls.u8LightAlarmLevel = 0;
    sDemoData.sSensors.u8TempResult = 0;
    sDemoData.sSensors.u8HtsResult = 0;
    sDemoData.sSensors.u8AlsResult = 0;
    sDemoData.sSystem.eState = E_STATE_OFF;
    sDemoData.sSystem.u16ShortAddr = 0xffff;
    sDemoData.sSystem.u8ThisNode = 0;

    /* Set light sensor values to 'wrong' ends of range, so the first time
       a value is read they will get updated */
    sDemoData.sLightSensor.u16Hi = 0;
    sDemoData.sLightSensor.u16Lo = 65535;
}

/****************************************************************************
 *
 * NAME: vProcessRead
 *
 * DESCRIPTION:
 * Gets the current readings from each sensor. If the light level causes the
 * low light alarm to be triggered, an LED is illuminated.
 *
 * RETURNS:
 * void
 *
 * NOTES:
 * This is not an efficient way to read the sensors as much time is wasted
 * waiting for the humidity and temperature sensor to complete. The sensor
 * pulls a DIO line low when it is ready, and this could be used to generate
 * an interrupt to indicate when data is ready to be read.
 *
 ****************************************************************************/
PRIVATE void vProcessRead(void)
{
    uint16 u16LightSensor;
    uint16 u16Diff;


    /* Read light level, adjust to range 0-6. This sensor automatically starts
       a new conversion afterwards so there is no need for a 'start read' */
    u16LightSensor = u16ALSreadChannelResult();
    /* Adjust the high and low values if necessary, and obtain the
       difference between them */

    if (sDemoData.sLightSensor.u16Hi < u16LightSensor)
    {
        sDemoData.sLightSensor.u16Hi = u16LightSensor;
    }

    if (sDemoData.sLightSensor.u16Lo > u16LightSensor)
    {
        sDemoData.sLightSensor.u16Lo = u16LightSensor;
    }

    u16Diff = sDemoData.sLightSensor.u16Hi - sDemoData.sLightSensor.u16Lo;

    /* Work out the current value as a value between 0 and 6 within the
       range of values that have been seen previously */
    if (u16Diff)
    {
        sDemoData.sSensors.u8AlsResult = (uint8)(((uint32)(u16LightSensor - sDemoData.sLightSensor.u16Lo) * 6) / (uint32)u16Diff);
    }
    else
    {
        sDemoData.sSensors.u8AlsResult = 3;
    }

    /* Set LED 1 based on light level */
    if ((sDemoData.sSensors.u8AlsResult <= sDemoData.sControls.u8LightAlarmLevel)
        && (sDemoData.sControls.u8LightAlarmLevel < 7))
    {
        vLedControl(LED2, TRUE);
    }
    else
    {
        vLedControl(LED2, FALSE);
    }

    /* Read temperature, 0-52 are acceptable. Polls until result received */
    vHTSstartReadTemp();
    sDemoData.sSensors.u8TempResult = u8FindMin((uint8)u16HTSreadTempResult(), 52);


    /* Read humidity, 0-104 are acceptable. Polls until result received */
    vHTSstartReadHumidity();
    sDemoData.sSensors.u8HtsResult = u8FindMin((uint8)u16HTSreadHumidityResult(), 104);
}

/****************************************************************************
 *
 * NAME: u8FindMin
 *
 * DESCRIPTION:
 * Returns the smallest of two values.
 *
 * PARAMETERS:      Name    RW  Usage
 *                  u8Val1  R   First value to compare
 *                  u8Val2  R   Second value to compare
 *
 * RETURNS:
 * uint8, lowest of two input values
 *
 ****************************************************************************/
PRIVATE uint8 u8FindMin(uint8 u8Val1, uint8 u8Val2)
{
    if (u8Val1 < u8Val2)
    {
        return u8Val1;
    }
    return u8Val2;
}
