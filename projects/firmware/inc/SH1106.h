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

#ifndef PROJECTS_DISPLAY_INC_SH1106_H_
#define PROJECTS_DISPLAY_INC_SH1106_H_

#include "Display.h"
// Libreria para Display Driver SH1106

/*
1 - Seleccionar el Segment Re-Map           6   ok
2 - COM model (Sequential or alternative    17  ok
3 - COM Scand direction                     13  ok
4 - Multiplex ratio mode                    9   ok
5 - Display divide Ratio/oscillator         15  ok
6 - VCOM deselect level set                 18  ok
7 - Contrast Set                            5   ok
8 - VPP value                               3   ok
9 - Clear interna RAM (0x00)
10 - DC-DC contro SET (0x8B recomendado)    10  ok
11 - Delay 100ms and Display ON (0xAF)      11  ok
12 - Delay 150ms                                ok
13 - Display Start line                     4   ok
14 - Page address set                       12  ok
15 - Column address set                     1,2
Finalmente mandar los datos
*/

#define SH1106_SET_REMAP_NORMAL 0xA0
#define SH1106_SET_REMAP_INVERSE 0xA1

#define SH1106_SET_CPM 0xDA
#define SH1106_CPM_SEQUENTIAL 0x02
#define SH1106_CPM_ALTERNATIVE 0x12

#define SH1106_SET_COSD_NORMAL 0xC0
#define SH1106_SET_COSD_INVERSE 0xC8

#define SH1106_SET_MUX_RATIO 0xA8
#define SH1106_MUX_RATIO_VALUE 0x3F

#define SH1106_SET_DIV_OSC 0xD5
#define SH1106_DIV_OSC_VALUE 0x50

#define SH1106_SET_VCOM 0xDB
#define SH1106_VCOM_VALUE 0x35

#define SH1106_SET_CONTRAST 0x81
#define SH1106_CONTRAST_VALUE 0x80

#define SH1106_SET_VPP 0x30

#define SH1106_SET_DCDC 0xAD
#define SH1106_DCDC_OFF 0x8A
#define SH1106_DCDC_ON 0x8B

#define SH1106_SET_DISPLAY_ON 0xAF
#define SH1106_SET_DISPLAY_OFF 0xAE

#define SH1106_SET_START_LINE 0x40  // 0x60

#define SH1106_SET_PAGE 0xB0

#define SH1106_SET_COLUMN_LOW 0x00
#define SH1106_SET_COLUMN_HIGH 0x10

#define SH1106_PAGES 8
#define SH1106_WIDTH 132

#define DISPLAY_ADDRESS 0x3C

// extern


// - Prototipos -
void SH1106_Comando(uint8_t comando);
void SH1106_Init(void);
void test_display2(void);
void SH1106_Write(uint8_t data);
void vDisplayTask(void *pvParametros);

// extern xSemaphoreHandle xDisplaySemaphore;

#endif /* PROJECTS_DISPLAY_INC_SH1106_H_ */
