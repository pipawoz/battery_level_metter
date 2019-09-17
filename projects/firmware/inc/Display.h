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

#ifndef PROJECTS_DISPLAY_LPC_INC_DISPLAY_H_
#define PROJECTS_DISPLAY_LPC_INC_DISPLAY_H_

#include "SH1106.h"
#include "board.h"


#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"


// - Defines para el caracteres -
#define NUMBER 48
#define PORCENT 37
#define BAR 47
#define POINTS 58
#define FLECHA_UP 31
#define FLECHA_DOWN 30
#define FONT_LENGTH 8
// - -

// - Defines para lo que se va a mostrar -
// Para la fecha
#define DATE_OFFSET 0
#define DATE_LENGTH 8
// Para la hour
#define TIME_LENGTH 5
#define TIME_OFFSET 11
// - -

// - Datos a escribir -
#define W_FECHA 0
#define W_HORA 1
#define W_LVL 2
#define W_TEXTO 3
// - -
typedef struct
{
  uint8_t datatype;    // Tipo de dato a escribir en el display
  uint8_t numbers[3];  // toma los datos para la hour, fecha o nivel
  uint8_t textlength;  // Largo del texto a escribir
  char texto[16];      // Texto a escribir
} ToDisplay;

// extern uint8_t display_mem;
// - Cola -

#define Q_DISPLAY_SIZE 10;
// - -

extern uint8_t display_mem[8][128];


uint8_t display_mem[8][128];

static uint8_t font[128][8] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0000 (nul)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0001
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0002
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0003
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0004
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0005
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0006
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0007
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0008
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0009
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+000A
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+000B
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+000C
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+000D
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+000E
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+000F
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0010
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0011
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0012
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0013
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0014
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0015
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0016
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0017
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0018
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+0019
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+001A
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+001B
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+001C
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // U+001D
  { 0x00, 0x08, 0x10, 0x20, 0x7F, 0x20, 0x10, 0x08 },  // 30 flecha down
  { 0x00, 0x08, 0x04, 0x02, 0x7F, 0x02, 0x04, 0x08 },  // 31 flecha up
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  //    32 ' '
  { 0x00, 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00 },  //    33 '!'
  { 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00 },  //    34 '"'
  { 0x00, 0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00 },  //    35 '#'
  { 0x00, 0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00 },  //    36 '$'
  { 0x00, 0x02, 0x45, 0x32, 0x08, 0x26, 0x51, 0x20 },  //    37 '%'
  { 0x00, 0x32, 0x4D, 0x49, 0x51, 0x20, 0x50, 0x00 },  //    38 '&'
  { 0x00, 0x00, 0x00, 0x04, 0x03, 0x01, 0x00, 0x00 },  //    39 '''
  { 0x00, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00 },  //    40 '('
  { 0x00, 0x00, 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00 },  //    41 ')'
  { 0x00, 0x04, 0x44, 0x28, 0x1F, 0x28, 0x44, 0x04 },  //    42 '*'
  { 0x00, 0x08, 0x08, 0x08, 0x7F, 0x08, 0x08, 0x08 },  //    43 '+'
  { 0x00, 0x00, 0x80, 0x60, 0x20, 0x00, 0x00, 0x00 },  //    44 ','
  { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08 },  //    45 '-'
  { 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00 },  //    46 '.'
  { 0x00, 0x00, 0xC0, 0x30, 0x0C, 0x03, 0x00, 0x00 },  //    47 '/'
  { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x41, 0x3E, 0x00 },  //    48 '0'
  { 0x00, 0x00, 0x00, 0x02, 0x7F, 0x00, 0x00, 0x00 },  //    49 '1'
  { 0x00, 0x42, 0x61, 0x51, 0x49, 0x45, 0x42, 0x00 },  //    50 '2'
  { 0x00, 0x22, 0x41, 0x49, 0x49, 0x49, 0x36, 0x00 },  //    51 '3'
  { 0x00, 0x10, 0x18, 0x14, 0x52, 0x7F, 0x50, 0x00 },  //    52 '4'
  { 0x00, 0x4F, 0x49, 0x49, 0x49, 0x49, 0x31, 0x00 },  //    53 '5'
  { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x49, 0x30, 0x00 },  //    54 '6'
  { 0x00, 0x01, 0x01, 0x41, 0x31, 0x0D, 0x03, 0x00 },  //    55 '7'
  { 0x00, 0x36, 0x49, 0x49, 0x49, 0x49, 0x36, 0x00 },  //    56 '8'
  { 0x00, 0x06, 0x49, 0x49, 0x49, 0x29, 0x1E, 0x00 },  //    57 '9'
  { 0x00, 0x00, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00 },  //    58 ':'
  { 0x00, 0x00, 0x80, 0x66, 0x26, 0x00, 0x00, 0x00 },  //    59 ';'
  { 0x00, 0x08, 0x08, 0x14, 0x14, 0x22, 0x22, 0x00 },  //    60 '<'
  { 0x00, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24 },  //    61 '='
  { 0x00, 0x22, 0x22, 0x14, 0x14, 0x08, 0x08, 0x00 },  //    62 '>'
  { 0x00, 0x00, 0x02, 0x01, 0x51, 0x09, 0x06, 0x00 },  //    63 '?'
  { 0x00, 0x1C, 0x22, 0x49, 0x55, 0x49, 0x12, 0x0C },  //    64 '@'
  { 0x00, 0x40, 0x70, 0x1C, 0x17, 0x1C, 0x70, 0x40 },  //    65 'A'
  { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x49, 0x49, 0x36 },  //    66 'B'
  { 0x00, 0x1C, 0x22, 0x41, 0x41, 0x41, 0x41, 0x22 },  //    67 'C'
  { 0x00, 0x7F, 0x41, 0x41, 0x41, 0x41, 0x22, 0x1C },  //    68 'D'
  { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x49, 0x41, 0x41 },  //    69 'E'
  { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x09, 0x01, 0x01 },  //    70 'F'
  { 0x00, 0x1C, 0x22, 0x41, 0x41, 0x49, 0x49, 0x7A },  //    71 'G'
  { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x7F },  //    72 'H'
  { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x41, 0x41, 0x00 },  //    73 'I'
  { 0x00, 0x30, 0x40, 0x40, 0x41, 0x41, 0x3F, 0x00 },  //    74 'J'
  { 0x00, 0x7F, 0x08, 0x08, 0x14, 0x22, 0x41, 0x00 },  //    75 'K'
  { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00 },  //    76 'L'
  { 0x00, 0x7F, 0x02, 0x04, 0x08, 0x04, 0x02, 0x7F },  //    77 'M'
  { 0x00, 0x7F, 0x02, 0x04, 0x08, 0x10, 0x20, 0x7F },  //    78 'N'
  { 0x00, 0x1C, 0x22, 0x41, 0x41, 0x41, 0x22, 0x1C },  //    79 'O'
  { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x09, 0x09, 0x06 },  //    80 'P'
  { 0x00, 0x1C, 0x22, 0x41, 0x41, 0xC1, 0xA2, 0x9C },  //    81 'Q'
  { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x19, 0x29, 0x46 },  //    82 'R'
  { 0x00, 0x26, 0x49, 0x49, 0x49, 0x49, 0x49, 0x32 },  //    83 'S'
  { 0x00, 0x01, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x01 },  //    84 'T'
  { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x40, 0x40, 0x3F },  //    85 'U'
  { 0x00, 0x01, 0x07, 0x18, 0x60, 0x18, 0x07, 0x01 },  //    86 'V'
  { 0x00, 0x7F, 0x20, 0x10, 0x08, 0x10, 0x20, 0x7F },  //    87 'W'
  { 0x00, 0x41, 0x22, 0x14, 0x08, 0x14, 0x22, 0x41 },  //    88 'X'
  { 0x00, 0x01, 0x02, 0x04, 0x78, 0x04, 0x02, 0x01 },  //    89 'Y'
  { 0x00, 0x41, 0x61, 0x51, 0x49, 0x45, 0x43, 0x41 },  //    90 'Z'
  { 0x00, 0x00, 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },  //    91 '['
  { 0x00, 0x00, 0x03, 0x0C, 0x30, 0xC0, 0x00, 0x00 },  //    92 '\'
  { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00 },  //    93 ']'
  { 0x00, 0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00 },  //    94 '^'
  { 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 },  //    95 '_'
  { 0x00, 0x00, 0x00, 0x01, 0x03, 0x04, 0x00, 0x00 },  //    96 '`'
  { 0x00, 0x38, 0x44, 0x44, 0x44, 0x44, 0x24, 0x7C },  //    97 'a'
  { 0x00, 0x7F, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38 },  //    98 'b'
  { 0x00, 0x38, 0x44, 0x44, 0x44, 0x44, 0x44, 0x00 },  //    99 'c'
  { 0x00, 0x38, 0x44, 0x44, 0x44, 0x44, 0x44, 0x7F },  //   100 'd'
  { 0x00, 0x38, 0x54, 0x54, 0x54, 0x54, 0x54, 0x18 },  //   101 'e'
  { 0x00, 0x04, 0x7E, 0x05, 0x05, 0x01, 0x01, 0x00 },  //   102 'f'
  { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0xA4, 0xA4, 0x7C },  //   103 'g'
  { 0x00, 0x7F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x78 },  //   104 'h'
  { 0x00, 0x00, 0x44, 0x44, 0x7D, 0x40, 0x40, 0x00 },  //   105 'i'
  { 0x00, 0x00, 0x80, 0x84, 0x84, 0x84, 0x7D, 0x00 },  //   106 'j'
  { 0x00, 0x7F, 0x10, 0x10, 0x28, 0x44, 0x44, 0x00 },  //   107 'k'
  { 0x00, 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00 },  //   108 'l'
  { 0x00, 0x7C, 0x04, 0x04, 0x38, 0x04, 0x04, 0x78 },  //   109 'm'
  { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x04, 0x04, 0x78 },  //   110 'n'
  { 0x00, 0x38, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38 },  //   111 'o'
  { 0x00, 0xFC, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38 },  //   112 'p'
  { 0x00, 0x38, 0x44, 0x44, 0x44, 0x44, 0x44, 0xFC },  //   113 'q'
  { 0x00, 0x00, 0x7C, 0x08, 0x04, 0x04, 0x04, 0x00 },  //   114 'r'
  { 0x00, 0x48, 0x54, 0x54, 0x54, 0x54, 0x24, 0x00 },  //   115 's'
  { 0x00, 0x04, 0x04, 0x3F, 0x44, 0x44, 0x44, 0x00 },  //   116 't'
  { 0x00, 0x3C, 0x40, 0x40, 0x40, 0x40, 0x20, 0x7C },  //   117 'u'
  { 0x00, 0x04, 0x0C, 0x30, 0x40, 0x30, 0x0C, 0x04 },  //   118 'v'
  { 0x00, 0x3C, 0x40, 0x40, 0x38, 0x40, 0x40, 0x3C },  //   119 'w'
  { 0x00, 0x44, 0x44, 0x28, 0x10, 0x28, 0x44, 0x44 },  //   120 'x'
  { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0xA0, 0x7C, 0x00 },  //   121 'y'
  { 0x00, 0x44, 0x64, 0x54, 0x54, 0x4C, 0x44, 0x00 },  //   122 'z'
  { 0x00, 0x00, 0x00, 0x08, 0x36, 0x41, 0x00, 0x00 },  //   123 '{'
  { 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00 },  //   124 '|'
  { 0x00, 0x00, 0x00, 0x41, 0x36, 0x08, 0x00, 0x00 },  //   125 '}'
  { 0x00, 0x06, 0x01, 0x01, 0x02, 0x04, 0x04, 0x03 },  //   126 '~'
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

// - Prototipos -
void print_time(uint8_t hh, uint8_t mm);
void print_date(uint8_t dd, uint8_t mm, uint8_t aa);
void draw_bat(void);
void print_lvl(uint8_t lvl);
void print_text(char *texto, uint8_t largo);
// - -

#endif /* PROJECTS_DISPLAY_LPC_INC_DISPLAY_H_ */
