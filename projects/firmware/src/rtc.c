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

#include "chip.h"
#include "board.h"
#include "rtc.h"

/**
 * @brief	RTC initialization
 * @param	none
 * @return	Nothing
 */

void initRTC(void)
{
  RTC logRTC;
  getTimeRTC(&logRTC);

  if (logRTC.year >= 2016)
  {
    /** Do nothing */
  }
  else
  {
    RTC_TIME_T fecha;
    Chip_RTC_Init(LPC_RTC);

    fecha.time[RTC_TIMETYPE_SECOND] = 0;
    fecha.time[RTC_TIMETYPE_MINUTE] = 10;
    fecha.time[RTC_TIMETYPE_HOUR] = 21;
    fecha.time[RTC_TIMETYPE_DAYOFMONTH] = 21;
    fecha.time[RTC_TIMETYPE_MONTH] = 12;
    fecha.time[RTC_TIMETYPE_YEAR] = 2015;

    Chip_RTC_SetFullTime(LPC_RTC, &fecha);
    Chip_RTC_Enable(LPC_RTC, ENABLE);
  }
}

/**
 * @brief	Retrieve RTC data
 * @param	*rtc : return value
 * @return	Nothing
 */
void getTimeRTC(RTC *rtc)
{
  RTC_TIME_T auxRTC;

  Chip_RTC_GetFullTime(LPC_RTC, &auxRTC);

  rtc->seconds = auxRTC.time[RTC_TIMETYPE_SECOND];
  rtc->minutes = auxRTC.time[RTC_TIMETYPE_MINUTE];
  rtc->hour = auxRTC.time[RTC_TIMETYPE_HOUR];
  rtc->day = auxRTC.time[RTC_TIMETYPE_DAYOFMONTH];
  rtc->month = auxRTC.time[RTC_TIMETYPE_MONTH];
  rtc->year = auxRTC.time[RTC_TIMETYPE_YEAR];
}

/**
 * @brief	Set new date in RTC
 * @param	*rtc : get new values
 * @return	Nothing
 */
void setTimeRTC(const RTC *rtc)
{
  RTC_TIME_T auxRTC;

  auxRTC.time[RTC_TIMETYPE_SECOND] = rtc->seconds;
  auxRTC.time[RTC_TIMETYPE_MINUTE] = rtc->minutes;
  auxRTC.time[RTC_TIMETYPE_HOUR] = rtc->hour;
  auxRTC.time[RTC_TIMETYPE_DAYOFMONTH] = rtc->day;
  auxRTC.time[RTC_TIMETYPE_MONTH] = rtc->month;
  auxRTC.time[RTC_TIMETYPE_YEAR] = rtc->year;

  Chip_RTC_SetFullTime(LPC_RTC, &auxRTC);
}
