/**
  ******************************************************************************
  * @file    ano_uart.h
  * @author  CHENG Ziqiang
  * @version V0.1.0
  * @date    2022/3
  * @brief   ANO uart header file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2022 CHENG</center></h2>
  *
  *
  ******************************************************************************
 */
#ifndef __ANO_UART_H_
#define __ANO_UART_H_

#include "board.h"

/*! Send acceleration data to the anonymous upper computer. */
void ANO_SendUART_Accelerate(int16_t *acc);

#endif
/************************ (C) COPYRIGHT CHENG *****END OF FILE****/