/* Copyright 2018, Pedro Wozniak
 * All rights reserved.
 *
 * This file is part of Workspace.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "main.h"

/**
 * @brief	SH1106 initialization
 * @param	none
 * @return	Nothing
 */
void SH1106_Init(void)
{
  SH1106_Comando(SH1106_SET_REMAP_NORMAL);

  SH1106_Comando(SH1106_SET_CPM);
  SH1106_Comando(SH1106_CPM_ALTERNATIVE);

  SH1106_Comando(SH1106_SET_COSD_NORMAL);

  SH1106_Comando(SH1106_SET_MUX_RATIO);
  SH1106_Comando(SH1106_MUX_RATIO_VALUE);

  SH1106_Comando(SH1106_SET_DIV_OSC);
  SH1106_Comando(SH1106_DIV_OSC_VALUE);

  SH1106_Comando(SH1106_SET_VCOM);
  SH1106_Comando(SH1106_VCOM_VALUE);

  SH1106_Comando(SH1106_SET_CONTRAST);
  SH1106_Comando(SH1106_CONTRAST_VALUE);

  SH1106_Comando(SH1106_SET_VPP);

  SH1106_Comando(SH1106_SET_DCDC);
  SH1106_Comando(SH1106_DCDC_ON);
  vTaskDelay(100 / portTICK_RATE_MS);

  SH1106_Comando(SH1106_SET_DISPLAY_ON);
  vTaskDelay(150 / portTICK_RATE_MS);

  SH1106_Comando(SH1106_SET_START_LINE);

  SH1106_Comando(SH1106_SET_PAGE);

  SH1106_Comando(SH1106_SET_COLUMN_LOW | 2);
  SH1106_Comando(SH1106_SET_COLUMN_HIGH);
}

/**
 * @brief	Escritura por SPI
 * @param	none
 * @return	Nothing
 */
void SH1106_Write(uint8_t data)
{
  I2C_XFER_T xfer;
  uint8_t wbuffer[2];

  wbuffer[0] = 0x40;  // control byte
  wbuffer[1] = data;

  xfer.slaveAddr = DISPLAY_ADDRESS;
  xfer.txBuff = wbuffer;
  xfer.txSz = 2;
  xfer.rxBuff = 0;
  xfer.rxSz = 0;
  xfer.status = 0;

  Chip_I2C_MasterTransfer(I2C_PORT, &xfer);
}

/**
 * @brief	Control por SPI
 * @param	none
 * @return	Nothing
 */
void SH1106_Comando(uint8_t comando)
{
  I2C_XFER_T xfer;
  uint8_t wbuffer[2];

  wbuffer[0] = 0x00;  // control byte
  wbuffer[1] = comando;

  xfer.slaveAddr = DISPLAY_ADDRESS;
  xfer.txBuff = wbuffer;
  xfer.txSz = 2;
  xfer.rxBuff = 0;
  xfer.rxSz = 0;
  xfer.status = 0;

  Chip_I2C_MasterTransfer(I2C_PORT, &xfer);
}
