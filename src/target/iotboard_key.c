/******************************************************************************

 @file       board_key.c

 @brief This file contains the interface to the SRF06EB Key Service.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2014-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/pin/PINCC26XX.h>

#ifdef USE_ICALL
#include <icall.h>
#endif

#include <inc/hw_ints.h>

#include "util.h"
#include "iotboard_key.h"
#include "board.h"
#include "inc/sdi_task.h"

/*********************************************************************
 * TYPEDEFS
 */
#define BTN_PRESSED_SHORT       10              // 1 second
#define BTN_PRESSED_MEDIUM      30              // 3 second
#define BTN_PRESSED_LONG        50              // 5 second
/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void Board_keyChangeHandler(UArg a0);
static void Board_keyCallback(PIN_Handle hPin, PIN_Id pinId);

static void Board_keyPressedHandler(UArg a0);

/*******************************************************************************
 * EXTERNAL VARIABLES
 */
extern Clock_Struct periodicClock;
/*********************************************************************
 * LOCAL VARIABLES
 */
bool POWER_OnOff_FLAG = false;
// Value of keys Pressed
static uint8_t pressedTimes = 0;

// Key debounce clock
static Clock_Struct keyChangeClock,powerOnClock;

// Pointer to application callback
keysPressedCB_t appKeyChangeHandler = NULL;

// Memory for the GPIO module to construct a Hwi
Hwi_Struct callbackHwiKeys;

// PIN configuration structure to set all KEY pins as inputs with pullups enabled
PIN_Config keyPinsCfg[] =
{
#if defined(CC2640R2MOD_RGZ)
    Board_BTN1          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
    Board_BTN2          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
#elif defined(CC2640R2MOD_RSM)
    Board_KEY_IN        | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,                   /* Button is active low */
    Board_POWER_OUT     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN, 
    Board_S_LED         | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* Board power initially ON */
#endif
    PIN_TERMINATE
};

PIN_State  keyPins;
PIN_Handle hKeyPins;

 /* Delay */
#define delay_ms(i) ( CPUdelay(12000*(i)) )
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void BLED_TogglePutputValue(void)
{
    PIN_setOutputValue(hKeyPins, Board_S_LED, !PIN_getOutputValue(Board_S_LED));
}

void BLED_SetPutputValue(bool flag)
{
    PIN_setOutputValue(hKeyPins, Board_S_LED, flag);
}
/*********************************************************************
 * @fn      Board_initKey_PowerOn
 *
 * @brief   Enable interrupts for keys on GPIOs.
 *
 * @param   appKeyCB - application key pressed callback
 *
 * @return  none
*/
void Board_initKey_PowerOn(void)
{
  // Initialize KEY pins. Enable int after callback registered
  hKeyPins = PIN_open(&keyPins, keyPinsCfg);
  if(PIN_getInputValue(Board_KEY_IN) == 0)
  {
       delay_ms(KEY_DELAY_POWERON_TIMEOUT);
       PIN_setOutputValue(hKeyPins, Board_S_LED, 0);
       PIN_setOutputValue(hKeyPins, Board_POWER_OUT, 1);
       POWER_OnOff_FLAG = true;  //Power ON
  }
}

void Board_PowerOff(void)
{
    Util_stopClock(&periodicClock);
    for(uint8_t t = 0; t < 6; t++)
    {
        BLED_TogglePutputValue();
        delay_ms(300);
    }
    PIN_setOutputValue(hKeyPins, Board_POWER_OUT, 0); 
}

/*********************************************************************
 * @fn      Board_initKeys
 *
 * @brief   Enable interrupts for keys on GPIOs.
 *
 * @param   appKeyCB - application key pressed callback
 *
 * @return  none
 */
void Board_initKeys(keysPressedCB_t appKeyCB)
{
  // Initialize KEY pins. Enable int after callback registered
//  hKeyPins = PIN_open(&keyPins, keyPinsCfg);
  PIN_registerIntCb(hKeyPins, Board_keyCallback);
  PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_KEY_IN        | PIN_IRQ_BOTHEDGES);
  
  // Setup keycallback for keys
  Util_constructClock(&keyChangeClock, Board_keyPressedHandler,
                      KEY_DEBOUNCE_TIMEOUT, KEY_DEBOUNCE_TIMEOUT, false, 0);

  // Set the application callback
  appKeyChangeHandler = appKeyCB;
}

/*********************************************************************
 * @fn      Board_keyCallback
 *
 * @brief   Interrupt handler for Keys
 *
 * @param   none
 *
 * @return  none
 */
static void Board_keyCallback(PIN_Handle hPin, PIN_Id pinId)
{
    static uint8_t cnt = 0;
    uint8_t pressedType = 0;
    static bool runPeriodicClock = false;
    
    Clock_Handle handle = Clock_handle(&keyChangeClock);
    bool runKeyChangeClock = Clock_isActive(handle);  
    SDITask_PrintfToUART("Start Periodic Clock! FLAG:%d \r\n",runPeriodicClock);
    if(!runKeyChangeClock){
        if(PIN_getInputValue(Board_KEY_IN) == 0){
            pressedTimes = 0;
            runPeriodicClock = Clock_isActive(Clock_handle(&periodicClock));
            if(runPeriodicClock){
                Util_stopClock(&periodicClock);
            }
            PIN_setOutputValue(hKeyPins, Board_S_LED, 1);
            Util_startClock(&keyChangeClock);
            SDITask_PrintfToUART("Start KeyChange Clock! CNT:%d \r\n",cnt++);
            SDITask_PrintfToUART("-----------------------------------------------\r\n");
        }
    }
    else{
        if(PIN_getInputValue(Board_KEY_IN) != 0){
            PIN_setOutputValue(hKeyPins, Board_S_LED, 0);
            if(runPeriodicClock){
                Util_startClock(&periodicClock);
            }
            Util_stopClock(&keyChangeClock);
            SDITask_PrintfToUART("Stop KeyChange Clock! CNT:%d \r\n",cnt--);  
            SDITask_PrintfToUART("KeyChange! Pressed Times:%d \r\n",pressedTimes);
            SDITask_PrintfToUART("-----------------------------------------------\r\n");
            
            if(pressedTimes < BTN_PRESSED_SHORT)
                pressedType = BTN_PRESSED_SHORT;
            else
                pressedType = BTN_PRESSED_MEDIUM;
 
            // Notify the application
            if (appKeyChangeHandler != NULL){
                (*appKeyChangeHandler)(pressedType);
            }
        }
    }
}

/*********************************************************************
 * @fn      Board_keyChangeHandler
 *
 * @brief   Handler for key change
 *
 * @param   UArg a0 - ignored
 *
 * @return  none
 */
static void Board_keyPressedHandler(UArg a0)
{
    ++pressedTimes;
//    if(pressedTimes % 5 == 0){   // Toggle the Tip LED per 5 second
//        BLED_TogglePutputValue();
//    }
    if(pressedTimes >= BTN_PRESSED_LONG){
        Util_stopClock(&keyChangeClock);
        // Notify the application
        if (appKeyChangeHandler != NULL){
            (*appKeyChangeHandler)(BTN_PRESSED_LONG);
        }
    }
}

/*********************************************************************
 * @fn      Board_keyChangeHandler
 *
 * @brief   Handler for key change
 *
 * @param   UArg a0 - ignored
 *
 * @return  none

static void Board_keyChangeHandler(UArg a0)
{
  if (appKeyChangeHandler != NULL)
  {
    keysPressed = 0;

    #if defined(CC2640R2MOD_RGZ)
      if ( PIN_getInputValue(Board_BTN1) == 0 )
      {
        keysPressed |= KEY_BTN1;
      }

      if ( PIN_getInputValue(Board_BTN2) == 0 )
      {
        keysPressed |= KEY_BTN2;
      }
    #endif     
    
    // Notify the application
    (*appKeyChangeHandler)(keysPressed);
  }
} */
/*********************************************************************
*********************************************************************/
