/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver for dsPIC33FJ128MC706A.
 *   @author CHENG Ziqiang (ziqiang.cheng@qq.com)
********************************************************************************
 * Copyright 2022(c) Maker Spaces, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 000
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "Communication.h"

/*********************************************************************
 * LOCAL PARAMETER
 */
//#define Slave_Addr        0x1D
#define Slave_Addr        0x53

I2C_Handle I2CHandle;
I2C_Params I2Cparams;

/***************************************************************************//**
 * @brief Initializes the I2C communication peripheral.
 *
 * @param void.
 *                    
 * @return void.
 *                  
*******************************************************************************/
void BLE_I2C_Init(void)
{
    I2C_init();
    I2C_Params_init(&I2Cparams);
    I2Cparams.bitRate = I2C_400kHz;
    I2Cparams.custom = NULL;
    I2Cparams.transferCallbackFxn = NULL;
    I2Cparams.transferMode = I2C_MODE_BLOCKING;
  
    I2CHandle = I2C_open(Board_I2C0,&I2Cparams);
}

/***************************************************************************//**
 * @brief Writes data to a slave device.
 *
 * @param regAddr - Adress of the slave register.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 *
 * @return status - True or False if the slave address was not
 *                  acknowledged by the device.
*******************************************************************************/
bool BLE_I2C_Write(uint8_t regAddr, uint8_t dataBuf)
{
    bool status = false;
    uint8_t buffer[2] = {regAddr,dataBuf};
    
    I2C_Transaction i2cTransaction;
    i2cTransaction.writeBuf = buffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    i2cTransaction.slaveAddress = Slave_Addr;
    i2cTransaction.arg = NULL;
    status = I2C_transfer(I2CHandle, &i2cTransaction);
    
    return status;
}

/***************************************************************************//**
 * @brief Reads data from a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuf - Pointer to a buffer that will store the received data.
 * @param bytesNum - Number of bytes to read.
 *
 * @return status - True or False if the slave address was not was not
 *                  acknowledged by the device.
*******************************************************************************/
bool BLE_I2C_Read(uint8_t regAddr, uint8_t* dataBuf, uint8_t bytesNum)
{
    bool status = false;
    
    I2C_Transaction i2cTransaction;
    i2cTransaction.writeBuf = &regAddr;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = dataBuf;
    i2cTransaction.readCount = bytesNum;
    i2cTransaction.slaveAddress = Slave_Addr;
    i2cTransaction.arg = NULL;
    status = I2C_transfer(I2CHandle, &i2cTransaction);
    
    return status;    
}
