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


xQueueHandle xDisplayQueue;
xSemaphoreHandle xDisplaySemaphore;
/**
 * @brief	Print the current time in the display
 * @param	Current Hour, Current Minute
 * @return	Nothing
 */

void print_time(uint8_t hh, uint8_t mm)
{
  uint8_t i = 0, n = 0, a = 0;
  uint8_t hour[TIME_LENGTH - 1];

  hour[0] = (hh / 10) % 10;
  hour[1] = hh % 10;
  hour[2] = (mm / 10) % 10;
  hour[3] = mm % 10;


  for (n = 0; n < TIME_LENGTH; n++)
  {
    if (n == 2)
    {
      for (i = 0; i < FONT_LENGTH; i++)
      {
        display_mem[0][(TIME_OFFSET * FONT_LENGTH) + (n * FONT_LENGTH) + i] = font[':'][i];
      }
      a++;
    }
    else
    {
      for (i = 0; i < FONT_LENGTH; i++)
      {
        display_mem[0][(TIME_OFFSET * FONT_LENGTH) + (n * FONT_LENGTH) + i] =
          font[hour[n - a] + NUMBER][i];
      }
    }
  }
}

/**
 * @brief	Funcion para escribir la fecha
 * @param	Dia, Mes, Año
 * @return	Nothing
 */
void print_date(uint8_t dd, uint8_t mm, uint8_t aa)
{
  uint8_t fecha[DATE_LENGTH - 2];
  uint8_t n = 0, i = 0, a = 0;

  fecha[0] = (dd / 10) % 10;
  fecha[1] = dd % 10;
  fecha[2] = (mm / 10) % 10;
  fecha[3] = mm % 10;
  fecha[4] = (aa / 10) % 10;
  fecha[5] = aa % 10;

  for (n = 0; n < DATE_LENGTH; n++)
  {
    if (n == 2 || n == 5)
    {
      for (i = 0; i < FONT_LENGTH; i++)
      {
        display_mem[0][(DATE_OFFSET * FONT_LENGTH) + (n * FONT_LENGTH) + i] = font['/'][i];
      }
      a++;
    }
    else
    {
      for (i = 0; i < FONT_LENGTH; i++)
      {
        display_mem[0][(DATE_OFFSET * FONT_LENGTH) + (n * FONT_LENGTH) + i] =
          font[fecha[(n - a)] + NUMBER][i];
      }
    }
  }
}

/**
 * @brief	Dibuja la forma de la bateria.
 * @param	none
 * @return	Nothing
 */

void draw_bat(void)
{
  uint8_t i = 0;
  for (i = 0; i < 52; i++)
  {
    display_mem[2][38 + i] = 0x30;
    display_mem[5][38 + i] = 0x0C;
  }
  for (i = 0; i < 2; i++)
  {
    display_mem[2][38 + i] = 0xF0;
    display_mem[5][38 + i] = 0x0F;
    display_mem[3][38 + i] = 0xFF;
    display_mem[4][38 + i] = 0xFF;
    display_mem[2][88 + i] = 0xF0;
    display_mem[5][88 + i] = 0x0F;
    display_mem[3][88 + i] = 0xFF;
    display_mem[4][88 + i] = 0xFF;
  }
  for (i = 0; i < 4; i++)
  {
    display_mem[3][90 + i] = 0xF0;
    display_mem[4][90 + i] = 0x0F;
  }
}
/**
 * @brief	Funcion para escribir el nivel de bateria
 * @param	Battery Level
 * @return	Nothing
 */
void print_lvl(uint8_t lvl)
{
  uint8_t nivel[3];
  uint8_t i = 0, n = 0;
  uint8_t dig = 3;
  uint8_t offset = 0;

  if (lvl < 100)
  {
    dig--;
    if (lvl < 10)
    {
      dig--;
    }
  }

  for (i = 0; i < 3; i++)
  {
    nivel[2 - i] = lvl % 10;
    lvl = lvl / 10;
  }

  offset = (128 / 2) - (((dig + 1) / 2) * FONT_LENGTH) - (((dig + 1) % 2) * (FONT_LENGTH / 2));

  for (n = 0; n < dig; n++)
  {
    for (i = 0; i < FONT_LENGTH; i++)
    {
      display_mem[3][offset + (n * FONT_LENGTH) + i] =
        (font[nivel[(3 - dig) + n] + NUMBER][i] << 4);
      display_mem[4][offset + (n * FONT_LENGTH) + i] =
        (font[nivel[(3 - dig) + n] + NUMBER][i] >> 4);
    }
  }
  for (i = 0; i < FONT_LENGTH; i++)
  {
    display_mem[3][offset + (dig * FONT_LENGTH) + i] = (font['%'][i] << 4);
    display_mem[4][offset + (dig * FONT_LENGTH) + i] = (font['%'][i] >> 4);
  }
}
/**
 * @brief	Imprime el estado en pantalla
 * @param	none
 * @return	Nothing
 */
////https://github.com/achilikin/bdfe/blob/master/font88.h
void print_text(char *texto, uint8_t largo)
{
  uint8_t offset = 0;

  offset = (128 / 2) - ((largo / 2) * FONT_LENGTH) - ((largo % 2) * (FONT_LENGTH / 2));
  uint8_t n, i;
  for (n = 0; n < 128; n++)
  {
    display_mem[7][n] = 0x00;
  }
  for (n = 0; n < largo; n++)
  {
    for (i = 0; i < FONT_LENGTH; i++)
    {
      display_mem[7][offset + (n * FONT_LENGTH) + i] = font[texto[n]][i];
    }
  }
}
