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

/** @brief This is an SD card write example using the FatFs library
 */

/** \addtogroup spi SD card write via SPI
 ** @{ */

/*==================[inclusions]=============================================*/

#include "board.h"
#include "uart.h"
#include "rtc.h"
#include <string.h>


/*#include <NXP/crp.h>
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
*/
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal functions definition]==========================*/
/**
 * @brief	Funcion para parsear los datos de la UART
 * @param	Buffer de recepcion
 * @return	Nothing
 */
void uartParse(uint8_t* rx_buf)
{
  RTC fechaAux;

  /*Esto se mantiene solo si respetamos el formato de envio de la fecha por UART.
   * Caso contrario hay que volver a armar la funcion
   */
  char rx_day[2], rx_month[2], rx_year[4], rx_hour[2], rx_minutes[2], rx_seconds[2];

  strncpy(rx_day, &rx_buf[6], 2);
  fechaAux.day = atoi(rx_day);

  strncpy(rx_month, &rx_buf[8], 2);
  fechaAux.month = atoi(rx_month);

  strncpy(rx_year, &rx_buf[10], 4);
  fechaAux.year = atoi(rx_year);
  fechaAux.year = fechaAux.year / 100;

  strncpy(rx_hour, &rx_buf[14], 2);
  fechaAux.hour = atoi(rx_hour);

  strncpy(rx_minutes, &rx_buf[16], 2);
  fechaAux.minutes = atoi(rx_minutes);

  strncpy(rx_seconds, &rx_buf[18], 2);
  fechaAux.seconds = atoi(rx_seconds);

  setTimeRTC(&fechaAux);
}
/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
