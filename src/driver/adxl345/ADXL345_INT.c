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
#include "ADXL345_INT.h"
#include "board.h"

// PIN configuration structure to set all KEY pins as inputs with pullups enabled
PIN_Config intPinsCfg[] =
{
    Board_TEST_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,         /* Test pin initially low */
    Board_ADXL345_INT1          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
    PIN_TERMINATE
};

PIN_State  intPins;
PIN_Handle hIntPins;

// Pointer to application callback
ADXL355INT_CB_t appADXL355_Handler = NULL;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void PIN_togglePutputValue(PIN_Id pinId)
{
  PIN_setOutputValue(hIntPins, pinId, !PIN_getOutputValue(pinId));
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
static void ADXL345_INT_Callback(PIN_Handle hPin, PIN_Id pinId)
{
  PIN_togglePutputValue(Board_TEST_PIN);
  // Notify the application
  (*appADXL355_Handler)();
}
/*********************************************************************
 * @fn      ADXL355_initDataReadyINT
 *
 * @brief   Enable interrupts for keys on GPIOs.
 *
 * @param   appKeyCB - application key pressed callback
 *
 * @return  none
 */
void ADXL345_initDataReadyINT(ADXL355INT_CB_t app_CB)
{
  // Initialize ADXL_INT pins. Enable int after callback registered
  hIntPins = PIN_open(&intPins, intPinsCfg);
  PIN_registerIntCb(hIntPins, ADXL345_INT_Callback);
  PIN_setConfig(hIntPins, PIN_BM_IRQ, Board_ADXL345_INT1        | PIN_IRQ_NEGEDGE);
  
    // Set the application callback
  appADXL355_Handler = app_CB;
}

