/**
  ******************************************************************************
  * @file    ano_uart.c
  * @author  CHENG Ziqiang
  * @version V0.1.0
  * @date    2022/03
  * @brief   user frame implementation for ANO UPPer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2022 CHENG</center></h2>
  *
  *
  ******************************************************************************
 */

 /* Includes ------------------------------------------------------------------*/
#include "ano_uart.h"
#include "inc/sdi_task.h"
#include "hal_assert.h"
#include "bcomdef.h"
#include "uarttrans_service.h"
#include "peripheral.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BYTE0(dwTemp)		(*(char *)(&dwTemp))
#define BYTE1(dwTemp)		(*((char *)(&dwTemp)+1))
#define BYTE2(dwTemp)		(*((char *)(&dwTemp)+2))
#define BYTE3(dwTemp)		(*((char *)(&dwTemp)+3))

//frame define
#define ANOHEAD         0xAA
#define ANOADDR         0xFF
#define ANOFUCID        0xF1
#define ANOLENTH        0x06

//user define frame
#define ANOIDF1   0xF1
#define ANOIDF2   0xF2
#define ANOIDF3   0xF3
#define ANOIDF4   0xF4
#define ANOIDF5   0xF5
#define ANOIDF6   0xF6
#define ANOIDF7   0xF7
#define ANOIDF8   0xF8
#define ANOIDF9   0xF9
#define ANOIDFA   0xFA

extern gaprole_States_t gapProfileState;

/***************************************************************************//**
 * @brief Send acceleration data to the anonymous upper computer.
 *
 * @param *acc - Pointer of the acceleration data.
 *
 * @return NULL.
*******************************************************************************/
void ANO_SendUART_Accelerate(int16_t *acc)
{
    uint8_t index = 0;
    uint8_t sumcheck = 0;//sum check byte
    uint8_t addcheck = 0;//additional check byte
    uint8_t dataFramBuffer[20];

/*****************************ANO Upper Head Frame*********************************/    
    dataFramBuffer[index++] = ANOHEAD;
    dataFramBuffer[index++] = ANOADDR;
    dataFramBuffer[index++] = ANOFUCID;
    dataFramBuffer[index++] = ANOLENTH;
    dataFramBuffer[index++] = BYTE0(acc[0]);
    dataFramBuffer[index++] = BYTE1(acc[0]);
    dataFramBuffer[index++] = BYTE0(acc[1]);
    dataFramBuffer[index++] = BYTE1(acc[1]);
    dataFramBuffer[index++] = BYTE0(acc[2]);
    dataFramBuffer[index++] = BYTE1(acc[2]);    
    
    for(uint8_t i=0; i < (dataFramBuffer[3] + 4); i++) 
    {
        sumcheck += dataFramBuffer[i]; 
        addcheck += sumcheck; 
    }
    dataFramBuffer[index++] = sumcheck;
    dataFramBuffer[index++] = addcheck;
//    SDITask_sendToUART(dataFramBuffer, index);
    //Send the notification
    if((gapProfileState == GAPROLE_CONNECTED) || (gapProfileState == GAPROLE_CONNECTED_ADV))
        SerialPortService_SetParameter(SERIALPORTSERVICE_CHAR_DATA, index, dataFramBuffer);
}



