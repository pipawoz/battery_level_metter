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

#ifndef _MAIN_H_
#define _MAIN_H_

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include "board.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "sd.h"
#include "rtc.h"
#include "uart.h"
#include "adc.h"
#include "SH1106.h"
#include "Display.h"
/*==================[cplusplus]==============================================*/

/*==================[macros]=================================================*/
#define OFF 0
#define ON 1
#define I2C_PORT I2C1

/*Port*/
#define PORT0 0
#define PORT1 1
#define PORT2 2

/*pines UART*/
#define PIN_UART_TX 2
#define PIN_UART_RX 3

/*pines SD*/
#define PIN_SD_MOSI 9
#define PIN_SD_MISO 8
#define PIN_SD_SCK 7
#define PIN_SD_SSEL 6
#define PIN_SD_LED 0

/*pines ADC*/
#define PIN_AD0 23

/*pines de entradas*/
#define PIN_S_LINE 24
#define PIN_S_CARGADOR 25
#define PIN_EN_RUOTER 26

/*pines BUZZER*/
#define PIN_BUZZER 21

#define PIN_P1 11
#define PORT_P GPIOINT_PORT0

#define LOW_ALARM_RATE_ms 2000
#define CRITICAL_ALARM_RATE_ms 500


// Estados de la máquina
#define NO_OPRIMIDO 0
#define DEBOUNCE 1
#define VALIDAR 2
#define OPRIMIDO 3
#define BOTON_NO_PRESIONADO 0
#define BOTON_PRESIONADO 1

#define SCAN_RATE_ms 150
#define TIEMPO_DE_DEBOUNCE_ms 20

// unsigned int 	codigo;

/*==================[typedef]================================================*/

typedef enum
{
  DISCONNECT_220,
  DISCONNECT_CHARGER,
  CONNECT_CHARGER,
  CONNECT_220,
  BATTERY_LEVEL,
  LEVEL_LOW,
  LEVEL_CRITICAL,
  ROUTER_OFF,
  ROUTER_ON,
  DATE_RECEIVED
} eventos;

/*==================[external data declaration]==============================*/

extern xSemaphoreHandle xSemaphoreADC;

/*==================[external functions declaration]=========================*/

/** @brief main function
 * @return main function should never return
 */
int main(void);

/*==================[cplusplus]==============================================*/

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _MAIN_H_ */
