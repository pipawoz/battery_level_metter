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

/*==================[inclusions]=============================================*/

#include "board.h"
#include "ff.h"
#include "main.h"

/*==================[macros and definitions]=================================*/

#define FILENAME "log.txt"

/*==================[internal data declaration]==============================*/

static FATFS fs; /**< FatFs work area needed for each volume */
static FIL fp;   /**< File object needed for each open file */

/*==================[internal functions declaration]=========================*/

/*==================[internal functions definition]==========================*/

/**
 * @brief	SD initialization
 * @param	none
 * @return	Nothing
 */

void initSD(void)
{
  /* SPI configuration */
  Board_SSP_Init(LPC_SSP1);
  Chip_SSP_Init(LPC_SSP1);
  Chip_SSP_Enable(LPC_SSP1);

  /*Configuro SSEL como salida*/
  Chip_IOCON_PinMux(LPC_IOCON, PORT0, PIN_SD_SSEL, IOCON_MODE_PULLUP, IOCON_FUNC0);
  Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT0, PIN_SD_SSEL);

  if (f_mount(&fs, "", 0) != FR_OK)
  {
    /*Deberia poder montar siempre el sistema de archivos*/
  }
}

/**
 * @brief	Set led of SD
 * @param	port	: GPIO port to set
 * @param	pin		: GPIO pin to set
 * @param	state	: on for high, off for low
 * @return	Nothing
 */
void SD_LED_SET(uint8_t port, uint8_t pin, bool state)
{
  Chip_GPIO_WritePortBit(LPC_GPIO, port, pin, state);
}

/**
 * @brief	Rutina para grabar en SD segun el tipo de evento recibido
 * @param	eventType:  Evento a grabar
 * @return	Nothing
 */
void save_log(uint8_t eventType)
{
  RTC logRTC;
  getTimeRTC(&logRTC);
  uint32_t nivelBateria;
  nivelBateria = getValueADC();

  // Board_LED_Set(0,1);
  /*se utilizara para el led del sd*/
  SD_LED_SET(PORT0, PIN_SD_LED, ON);

  if (logRTC.year >= 2016)  // Si la fecha no fue configurada entonces no graba el log
  {
    /* Usamos f_printf para mayor facilidad. Para velocidad usar f_wirte
    pero en este caso como la tarea no es critica no nos importa algunos
    ms mas de tiempo de escritura en la sd, tener en cuenta esto*/
    if (f_open(&fp, FILENAME, FA_WRITE | FA_OPEN_APPEND) == FR_OK)

    {
      switch (eventType)
      {
        case DATE_RECEIVED:
          f_printf(&fp,
                   "%02d/%02d/%04d %02d:%02d:%02d Date configured\r\n",
                   logRTC.day,
                   logRTC.month,
                   logRTC.year,
                   logRTC.hour,
                   logRTC.minutes,
                   logRTC.seconds);
          break;

        case BATTERY_LEVEL:
          f_printf(&fp,
                   "%02d/%02d/%04d %02d:%02d:%02d Battery Level: %02d%% Normal Level\r\n",
                   logRTC.day,
                   logRTC.month,
                   logRTC.year,
                   logRTC.hour,
                   logRTC.minutes,
                   logRTC.seconds,
                   nivelBateria);

          break;

        case LEVEL_CRITICAL:
          f_printf(&fp,
                   "%02d/%02d/%04d %02d:%02d:%02d Battery Level: %02d%% Critial Level\r\n",
                   logRTC.day,
                   logRTC.month,
                   logRTC.year,
                   logRTC.hour,
                   logRTC.minutes,
                   logRTC.seconds,
                   nivelBateria);

          f_printf(&fp,
                   "%02d/%02d/%04d %02d:%02d:%02d Battery Level: %02d%% Alarm on\r\n",
                   logRTC.day,
                   logRTC.month,
                   logRTC.year,
                   logRTC.hour,
                   logRTC.minutes,
                   logRTC.seconds,
                   nivelBateria);
          break;

        case DISCONNECT_220:
          f_printf(&fp,
                   "%02d/%02d/%04d %02d:%02d:%02d Power line disconection\r\n",
                   logRTC.day,
                   logRTC.month,
                   logRTC.year,
                   logRTC.hour,
                   logRTC.minutes,
                   logRTC.seconds);
          break;

        case CONNECT_220:
          f_printf(&fp,
                   "%02d/%02d/%04d %02d:%02d:%02d Power line connection\r\n",
                   logRTC.day,
                   logRTC.month,
                   logRTC.year,
                   logRTC.hour,
                   logRTC.minutes,
                   logRTC.seconds);
          break;

        case CONNECT_CHARGER:
          f_printf(&fp,
                   "%02d/%02d/%04d %02d:%02d:%02d Charger connected\r\n",
                   logRTC.day,
                   logRTC.month,
                   logRTC.year,
                   logRTC.hour,
                   logRTC.minutes,
                   logRTC.seconds);
          break;

        case DISCONNECT_CHARGER:

          f_printf(&fp,
                   "%02d/%02d/%04d %02d:%02d:%02d Charger disconected\r\n",
                   logRTC.day,
                   logRTC.month,
                   logRTC.year,
                   logRTC.hour,
                   logRTC.minutes,
                   logRTC.seconds);
          break;

        case ROUTER_OFF:

          f_printf(&fp,
                   "%02d/%02d/%04d %02d:%02d:%02d Critical Level emergency shutdown\r\n",
                   logRTC.day,
                   logRTC.month,
                   logRTC.year,
                   logRTC.hour,
                   logRTC.minutes,
                   logRTC.seconds);
          break;
      }
    }

    f_close(&fp);
  }

  /*se utilizara para el led del sd*/
  SD_LED_SET(PORT0, PIN_SD_LED, OFF);
}

/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
