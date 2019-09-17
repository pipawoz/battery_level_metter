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
#include "main.h"

/*#include <NXP/crp.h>
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
*/
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal functions definition]==========================*/


/**
 * @brief	Initialize ADC in DMA mode
 * @return	Nothing
 */

void initADC(void)
{
  ADC_CLOCK_SETUP_T adc_setup;

  Chip_IOCON_PinMux(LPC_IOCON, PORT0, PIN_AD0, IOCON_MODE_INACT, IOCON_FUNC1);

  // Setup ADC0: 10-bit, 100kSPS, burst mode [UM:47.6.1]
  Chip_ADC_Init(LPC_ADC, &adc_setup);
  Chip_ADC_SetSampleRate(LPC_ADC, &adc_setup, 100000);
  Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);

  Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);      // [UM:47.6.1]
  Chip_ADC_Int_SetChannelCmd(LPC_ADC, ADC_CH0, ENABLE);  // [UM:47.6.3]

  /* DMA controller configuration */
  Chip_GPDMA_Init(LPC_GPDMA);  // [UM:21.6]

  // Get a free channel for a ADC0->Memory DMA transfer (8 available)
  dma_ch_adc = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_ADC);

  // Enable DMA interrupt in the NVIC [UM:9.7]
  NVIC_ClearPendingIRQ(DMA_IRQn);
  NVIC_EnableIRQ(DMA_IRQn);

  startDMATransfer();
}

/**
 * @brief	Return the ADC value in % from 0-100
 * @return	uint32_t
 */
uint32_t getValueADC(void)
{
  uint8_t i = 0;
  uint32_t promedio = 0, suma = 0;
  uint8_t valorDescartado = 0;
  /* After the DMA transfers N_SAMPLES values from ADC0_CH1 to
   * dma_buf[], copy them to adc_buf[] and start a new DMA transfer */
  if (xSemaphoreTake(xSemaphoreADC, portMAX_DELAY) == pdTRUE)
  {
    for (i = 0; i < N_SAMPLES; i++)
    {
      adc_buf[i] = ADC_DR_RESULT(dma_buf[i]);
      suma += adc_buf[i];
    }
    promedio = (suma / N_SAMPLES);
    promedio = promedio * 100 / 4096;  // 3.3v -> 100%
    startDMATransfer();
  }

  return promedio;
}

/**
 * @brief	DMA Transfer function
 * @return	Nothing
 */
void startDMATransfer(void)
{
  dma_adc_transfer_done = false;

  Chip_GPDMA_Transfer(LPC_GPDMA,
                      dma_ch_adc,
                      GPDMA_CONN_ADC,
                      ( uint32_t )&dma_buf,
                      GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
                      N_SAMPLES);  // [UM:21.6.15]
}

/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
