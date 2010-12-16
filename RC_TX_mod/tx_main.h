/*===================================================================
 * Project: simple-flight-controller 
 *
 * Main Module 
 * 
 * Copyright (c) 2010 Thanh H Tran (thanhthd@gmail.com)
 *
 *  This file is part of simple-flight-controller project
 *
 * Simple-Flight-Controller is free software: 
 * you can redistribute it and /or modify 
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * Simple-Flight-Controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with simple-flight-controller.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *===================================================================
*/

#ifndef MAIN_INC_H
#define MAIN_INC_H


#define INIT_DELAY_TIME_100MS    (50)     // wait for 5 seconds (50 x 0.1S) after powering up 
                                          // before initializing gyros
#define MAX_PPM_CHANNELS		   7

extern CAPTURE_CCB ppm_ccb;
extern UINT16 ppm_pulses[MAX_PPM_CHANNELS];

extern UINT8   woken_up_by;
#define WOKEN_UP_BY_UNKNOWN   0
#define WOKEN_UP_BY_ADC       (1<<1)
#define WOKEN_UP_BY_TIMER     (1<<2)
#define WOKEN_UP_BY_CMD       (1<<3)
#define WOKEN_UP_BY_WIRELESS  (1<<4)
#define WOKEN_UP_BY_PPM_FRAME (1<<5)

extern UINT16 ail_pulse;
extern UINT16 pit_pulse;
extern UINT16 thr_pulse;
extern UINT16 rud_pulse;
extern UINT16 tx_gain_pulse; 
extern UINT16 num_ppm_pulses;

#endif
