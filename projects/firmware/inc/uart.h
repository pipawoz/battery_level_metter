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

#ifndef _UART_H_
#define _UART_H_

/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

//#ifdef __cplusplus
// extern "C" {
//#endif

/*==================[macros]=================================================*/

#define UART_RB_SIZE 64  // Rx and Tx ring buffers size (must be power of 2)
#define RXBUF_SIZE 26

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

static RINGBUFF_T rx_rb, tx_rb;
static uint8_t rx_rb_buf[UART_RB_SIZE], tx_rb_buf[UART_RB_SIZE];

/*==================[external functions declaration]=========================*/
/** @brief hardware initialization function
 *	@return none
 */
void initUART(void);

void uartParse(uint8_t*);

/*==================[cplusplus]==============================================*/

//#ifdef __cplusplus
//}
//#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _UART_H_ */
