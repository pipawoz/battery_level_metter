/* Copyright 2018, Pedro Wozniak
 * All rights reserved.
 *
 * This file is part of lpc1769_template.
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

#include "main.h"

/*==================[macros and definitions]=================================*/
/**
 * @brief Enable debug in UART
 */
#define DEBUG_ENABLED ON
/**
 * @brief Event queue size
 */
#define mainQUEUE_LENGTH (10)
/**
 * @brief ADC queue
 */
#define adcQUEUE_LENGTH (1)
/**
 * @brief Stack size
 */
#define STACK_SIZE 512

/*==================[internal data declaration]==============================*/
/**
 * @brief ADC Semaphore
 */
xSemaphoreHandle xSemaphoreADC;

/**
 * @brief Event queue
 */
xQueueHandle xEventQueue = NULL;

/**
 * @brief Semaphore struct
 */
typedef struct
{
  xSemaphoreHandle xSemaphoreConexion220;
  xSemaphoreHandle xSemaphoreDesconexion220;
  xSemaphoreHandle xSemaphoreConexionCargador;
  xSemaphoreHandle xSemaphoreDesconexionCargador;
  xSemaphoreHandle xSemaphoreRouter;
} taskParam_t;

/**
 * @brief Semaphore param_t
 */
static taskParam_t param_t1;

/**
 * @brief Alarm flag
 */
static char flag_alarma = 0;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void)
{
  Board_Init();
  SystemCoreClockUpdate();

  initUART();
  initRTC();
  initADC();
  initSD();  // Ok

  Chip_GPIOINT_Init(LPC_GPIOINT);

  Chip_IOCON_PinMux(LPC_IOCON, PORT0, PIN_S_LINE, IOCON_MODE_PULLUP, IOCON_FUNC0);
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, PORT0, PIN_S_LINE);

  Chip_IOCON_PinMux(LPC_IOCON, PORT0, PIN_S_CARGADOR, IOCON_MODE_PULLUP, IOCON_FUNC0);
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, PORT0, PIN_S_CARGADOR);

  Chip_IOCON_PinMux(LPC_IOCON, PORT0, PIN_EN_RUOTER, IOCON_MODE_PULLUP, IOCON_FUNC0);
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, PORT0, PIN_EN_RUOTER);

  Chip_GPIOINT_SetIntFalling(
    LPC_GPIOINT, GPIOINT_PORT0, (1 << PIN_S_LINE) | (1 << PIN_S_CARGADOR) | (1 << PIN_EN_RUOTER));
  Chip_GPIOINT_SetIntRising(
    LPC_GPIOINT, GPIOINT_PORT0, (1 << PIN_S_LINE) | (1 << PIN_S_CARGADOR) | (1 << PIN_EN_RUOTER));

  NVIC_EnableIRQ(EINT3_IRQn);

  Chip_IOCON_PinMux(LPC_IOCON, PORT0, PIN_BUZZER, IOCON_MODE_PULLUP, IOCON_FUNC0);
  Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT0, PIN_BUZZER);

  Chip_IOCON_PinMux(LPC_IOCON, PORT0, PIN_EN_RUOTER, IOCON_MODE_PULLUP, IOCON_FUNC0);
  Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT0, PIN_EN_RUOTER);
  Chip_GPIO_SetPinState(LPC_GPIO, PORT0, PIN_EN_RUOTER, OFF);

  Board_I2C_Init(I2C1);
  Chip_I2C_SetClockRate(I2C1, 100000);
  Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);
}
/**
 * @brief	Task to print data in the display
 * @param	none
 * @return	Nothing
 */
void vDisplayTask(void *pvParametros)
{
  RTC logRTC;
  uint8_t p = 0;
  uint8_t l = 0;
  SH1106_Init();
  draw_bat();

  uint32_t valor = 0;

  while (1)
  {
    getTimeRTC(&logRTC);


    print_date(logRTC.day, logRTC.month, logRTC.year % 100);
    print_time(logRTC.hour, logRTC.minutes);
    for (p = 0; p < SH1106_PAGES; p++)
    {
      SH1106_Comando(SH1106_SET_PAGE | p);
      for (l = 0; l < SH1106_WIDTH; l++)
      {
        SH1106_Write(display_mem[p][l]);
      }
    }
  }
}


/**
 * @brief	GPIO Task
 * 			Semaphore handle and event control
 * @return	Nothing
 */
static void vTaskGPIO(void *p)
{
  eventos eventType;

  taskParam_t *t = ( taskParam_t * )p;
  xSemaphoreHandle s;
  xQueueSetHandle set;

  set = xQueueCreateSet(5);

  xQueueAddToSet(t->xSemaphoreConexion220, set);
  xQueueAddToSet(t->xSemaphoreDesconexion220, set);
  xQueueAddToSet(t->xSemaphoreConexionCargador, set);
  xQueueAddToSet(t->xSemaphoreDesconexionCargador, set);
  xQueueAddToSet(t->xSemaphoreRouter, set);

  while (1)
  {
    s = xQueueSelectFromSet(set, portMAX_DELAY);

    if (s == t->xSemaphoreConexion220)
    {
      eventType = CONNECT_220;
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "220 Connect\r\n", 14);
    }

    if (s == t->xSemaphoreDesconexion220)
    {
      eventType = DISCONNECT_220;
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "220 Disconnect\r\n", 17);
    }

    if (s == t->xSemaphoreConexionCargador)
    {
      eventType = CONNECT_CHARGER;
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "Charger Connect\r\n", 19);
    }

    if (s == t->xSemaphoreDesconexionCargador)
    {
      eventType = DISCONNECT_CHARGER;
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "Charger Disconnect\r\n", 22);
    }


    xQueueSendToBack(xEventQueue, &eventType, 0);
    xSemaphoreTake(s, 0);
  }
}

/**
 * @brief	Alarm Task
 * 			Modify the alarm level according to the battery level.
 * @return	Nothing
 */
static void vTaskAlarma(void *p)
{
  portTickType xMeDesperte = xTaskGetTickCount();

  while (1)
  {
    if (flag_alarma == 0)
    {
      Chip_GPIO_SetPinState(LPC_GPIO, PORT0, PIN_BUZZER, OFF);
    }
    else if (flag_alarma == 1)
    {
      Chip_GPIO_SetPinToggle(LPC_GPIO, PORT0, PIN_BUZZER);
      vTaskDelayUntil(&xMeDesperte, LOW_ALARM_RATE_ms / portTICK_RATE_MS);
    }
    else if (flag_alarma == 2)
    {
      Chip_GPIO_SetPinToggle(LPC_GPIO, PORT0, PIN_BUZZER);
      vTaskDelayUntil(&xMeDesperte, CRITICAL_ALARM_RATE_ms / portTICK_RATE_MS);
    }
    else
    {
      Chip_GPIO_SetPinState(LPC_GPIO, PORT0, PIN_BUZZER, ON);
      vTaskDelayUntil(&xMeDesperte, CRITICAL_ALARM_RATE_ms / portTICK_RATE_MS);
    }
  }
}


/**
 * @brief	UART Task
 *          UART data processing
 * @return	Nothing
 */
static void vTaskUART(void *a)
{
  eventos eventType;

  uint8_t rx_bytes = 0;
  uint8_t rx_buf[RXBUF_SIZE] = { 0 };

  portTickType xMeDesperte = xTaskGetTickCount();

  while (1)
  {
    rx_bytes = RingBuffer_GetCount(&rx_rb);

    /*La fecha recibida es DDMMAA27102018182435*/

    if (rx_bytes > 15)
    {
      Chip_UART_ReadRB(LPC_UART0, &rx_rb, rx_buf, RXBUF_SIZE);

      uartParse(rx_buf);

      Chip_UART_SendRB(LPC_UART0, &tx_rb, "Date Received\r\n", 16);

      eventType = DATE_RECEIVED;

      xQueueSendToBack(xEventQueue, &eventType, 0);
    }

    vTaskDelayUntil(&xMeDesperte, 1000 / portTICK_RATE_MS);
  }
}


/**
 * @brief	ADC Task
 * 			Se utiliza DMA para obtener el valor actual de bateria.
 * @return	Nothing
 */
static void vTaskADC(void *a)
{
  eventos eventType;
  int valor = 0;
  startDMATransfer();
  portTickType xMeDesperte = xTaskGetTickCount();
#if DEBUG_ENABLED
  char buffer[5];
#endif
  while (1)
  {
    valor = getValueADC();

    print_lvl(valor);
    if (valor >= 80)
    {
      flag_alarma = 0;
      eventType = BATTERY_LEVEL;
      print_text("Normal", 6);
#if DEBUG_ENABLED
      snprintf(buffer, sizeof(buffer), "%02d", valor);
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "Battery Level: ", 15);
      Chip_UART_SendRB(LPC_UART0, &tx_rb, buffer, 3);
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "% Normal\r\n", 10);
#endif
    }
    else if (valor >= 74 && valor < 80)
    {
      flag_alarma = 1;
      eventType = LEVEL_LOW;
      print_text("Low", 4);
#if DEBUG_ENABLED
      snprintf(buffer, sizeof(buffer), "%02d", valor);
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "Battery Level: ", 15);
      Chip_UART_SendRB(LPC_UART0, &tx_rb, buffer, 2);
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "% Low\r\n", 8);
#endif
    }
    else if (valor >= 71 && valor < 74)
    {
      flag_alarma = 2;
      eventType = LEVEL_CRITICAL;
      print_text("Critical", 7);
#if DEBUG_ENABLED
      snprintf(buffer, sizeof(buffer), "%02d", valor);
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "Battery Level: ", 15);
      Chip_UART_SendRB(LPC_UART0, &tx_rb, buffer, 2);
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "% Critical\r\n", 11);
#endif
    }
    else
    {
      flag_alarma = 3;
      eventType = ROUTER_OFF;
      print_text("Disconnected", 12);
      Chip_GPIO_SetPinState(LPC_GPIO, PORT0, PIN_EN_RUOTER, ON);
      //	xSemaphoreGive(param_t1.xSemaphoreRouter);


#if DEBUG_ENABLED
      Chip_UART_SendRB(LPC_UART0, &tx_rb, "Critical level emergency shutdown\r\n", 45);
      Board_LED_Toggle(0);
#endif
    }

    xQueueSendToBack(xEventQueue, &eventType, 0);
    startDMATransfer();

    vTaskDelayUntil(&xMeDesperte, 1000 / portTICK_RATE_MS);
  }
}

/**
 * @brief	SD Task
 *          Send queue events to SD memory
 * @return	Nothing
 */
static void vTaskSD(void *a)
{
  eventos eventType;

  while (1)
  {
    if (xQueueReceive(xEventQueue, &eventType, portMAX_DELAY) == pdTRUE)
    {
      save_log(eventType);
    }
  }
}

/** @brief UART initialization function
 *         Initialize UART in mode 8-N-1 115200
 *	@return none
 */
void initUART(void)
{
  Chip_IOCON_PinMux(LPC_IOCON, PORT0, PIN_UART_TX, IOCON_MODE_INACT, IOCON_FUNC1);
  Chip_IOCON_PinMux(LPC_IOCON, PORT0, PIN_UART_RX, IOCON_MODE_INACT, IOCON_FUNC1);

  // USART peripheral configuration
  Chip_UART_Init(LPC_UART0);  // 8-N-1 and FIFOs enabled
  Chip_UART_SetBaud(LPC_UART0, 115200);
  Chip_UART_TXEnable(LPC_UART0);

  // Enable USART RBR and THRE interrupt
  Chip_UART_IntEnable(LPC_UART0, (UART_IER_RBRINT | UART_IER_THREINT));

  // Enable USART interrupt in the NVIC
  NVIC_ClearPendingIRQ(UART0_IRQn);
  NVIC_EnableIRQ(UART0_IRQn);

  // Initialize USART2 Rx and Tx ring buffers
  RingBuffer_Init(&rx_rb, rx_rb_buf, 1, UART_RB_SIZE);
  RingBuffer_Init(&tx_rb, tx_rb_buf, 1, UART_RB_SIZE);
#if DEBUG_ENABLED
  Chip_UART_SendRB(LPC_UART0, &tx_rb, "Device initialized\r\n", 26);
#endif
}


/*==================[external functions definition]==========================*/

/** @brief MAIN Function
 *	@return none
 */
int main(void)
{
  eventos eventType;

  param_t1.xSemaphoreConexion220 = xSemaphoreCreateBinary();
  param_t1.xSemaphoreDesconexion220 = xSemaphoreCreateBinary();
  param_t1.xSemaphoreConexionCargador = xSemaphoreCreateBinary();
  param_t1.xSemaphoreDesconexionCargador = xSemaphoreCreateBinary();
  param_t1.xSemaphoreRouter = xSemaphoreCreateBinary();
  xSemaphoreADC = xSemaphoreCreateBinary();

  initHardware();

  xTaskCreate(
    vTaskUART, (const signed char *const) "vTaskUART", STACK_SIZE, 0, tskIDLE_PRIORITY + 1, 0);

  xTaskCreate(
    vTaskSD, (const signed char *const) "vTaskSD", STACK_SIZE, 0, tskIDLE_PRIORITY + 1, 0);

  xTaskCreate(
    vTaskADC, (const signed char *const) "vTaskADC", STACK_SIZE, 0, tskIDLE_PRIORITY + 1, 0);

  xTaskCreate(vTaskGPIO,
              (const signed char *const) "vTaskGPIO",
              STACK_SIZE,
              &param_t1,
              tskIDLE_PRIORITY + 1,
              0);

  xTaskCreate(
    vTaskAlarma, (const signed char *const) "vTaskAlarma", STACK_SIZE, 0, tskIDLE_PRIORITY + 1, 0);
    
  xTaskCreate(vDisplayTask, ( signed char * )"Display", 512, NULL, (tskIDLE_PRIORITY + 1), NULL);

  xEventQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(eventType));

  uint8_t p = 0, l = 0;

  for (p = 0; p < SH1106_PAGES; p++)
  {
    for (l = 0; l < SH1106_WIDTH; l++)
    {
      display_mem[p][l] = 0x00;
    }
  }


  vTaskStartScheduler();

  while (1)
  {
  }

  return -1; /* Nunca deberia llegar aca */
}

/** @brief Interrupt Handler
 *          Verify different inputs and free the corresponding semaphore
 *	@return none
 */
void EINT3_IRQHandler(void)
{
  static portBASE_TYPE xTaskSwitchRequired = pdFALSE;

  if (Chip_GPIOINT_GetStatusFalling(LPC_GPIOINT, GPIOINT_PORT0) & (1 << PIN_S_LINE))
  {
    Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, GPIOINT_PORT0, 1 << PIN_S_LINE);
    xSemaphoreGiveFromISR(param_t1.xSemaphoreConexion220, &xTaskSwitchRequired);
  }

  if (Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, GPIOINT_PORT0) & (1 << PIN_S_LINE))
  {
    Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, GPIOINT_PORT0, 1 << PIN_S_LINE);
    xSemaphoreGiveFromISR(param_t1.xSemaphoreDesconexion220, &xTaskSwitchRequired);
  }

  if (Chip_GPIOINT_GetStatusFalling(LPC_GPIOINT, GPIOINT_PORT0) & (1 << PIN_S_CARGADOR))
  {
    Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, GPIOINT_PORT0, 1 << PIN_S_CARGADOR);
    xSemaphoreGiveFromISR(param_t1.xSemaphoreConexionCargador, &xTaskSwitchRequired);
  }

  if (Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, GPIOINT_PORT0) & (1 << PIN_S_CARGADOR))
  {
    Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, GPIOINT_PORT0, 1 << PIN_S_CARGADOR);
    xSemaphoreGiveFromISR(param_t1.xSemaphoreDesconexionCargador, &xTaskSwitchRequired);
  }


  portEND_SWITCHING_ISR(xTaskSwitchRequired);
}

/** @brief DMA Interrupt Handler
 *      DMA interrupt handler triggered after N_SAMPLES conversions of ADC0_1
 *	@return none
 */
void DMA_IRQHandler(void)
{
  static portBASE_TYPE xTaskSwitchRequired = pdFALSE;

  if (Chip_GPDMA_Interrupt(LPC_GPDMA, dma_ch_adc) == SUCCESS)
  {
    xSemaphoreGiveFromISR(xSemaphoreADC, &xTaskSwitchRequired);
    dma_adc_transfer_done = true;
  }

  portEND_SWITCHING_ISR(xTaskSwitchRequired);
}

/** @brief 10ms tickhook
 *  @warning this is called by an interrupt handler!
 */
void vApplicationTickHook(void)
{
  static portBASE_TYPE xTaskSwitchRequired = pdFALSE;
  disk_timerproc();  // RTOS cada 10ms por config
  portEND_SWITCHING_ISR(xTaskSwitchRequired);
}

/** @brief UART Interrupt Handler
 *	@return none
 */
void UART0_IRQHandler(void)
{
  static portBASE_TYPE xTaskSwitchRequired = pdFALSE;
  Chip_UART_IRQRBHandler(LPC_UART0, &rx_rb, &tx_rb);
  portEND_SWITCHING_ISR(xTaskSwitchRequired);
}

/** @brief FreeRTOS application idle hook
 *	@return none
 */
void vApplicationIdleHook(void)
{
  /* Best to sleep here until next systick */
  __WFI();
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
