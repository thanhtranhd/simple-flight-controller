/*===================================================================
 * Project: simple-flight-controller 
 *
 * This code handles the mixing of different controls and inputs
 * from gyros so that the multicopter can fly. 
 * 
 * Copyright (c) 2010 Thanh H Tran (thanhthd@gmail.com)
 *
 * This file is part of simple-flight-controller project
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

#include "types.h"

// various flags used in the mixer. 
extern UINT16 mixer_flags;

// possible values for mixer_flags
#define MIXER_CALIBRATE_ON	   (1<<0)        // used to calibrate all ESC throtle range
#define MIXER_DISC_INPUT_ON    (1<<1)        // disconnect the receiver signals
#define MIXER_IS_ARMED         (1<<2)        // arming the board.
#define MIXER_GYRO_TEST_ON	   (1<<3)        // use this to test gyro direction.         
#define MIXER_GYRO_FILTER_ON   (1<<5)        // filters gyro outputs
#define MIXER_PRINT_DEBUG_ON   (1<<15)       // turn on debugging


extern UINT16 adc_roll_gyro_val;            // gyro values from ADC inputs
extern UINT16 adc_pitch_gyro_val;
extern UINT16 adc_yaw_gyro_val;

extern UINT16 adc_roll_prev_gyro_val;       // previous gyro values 
extern UINT16 adc_pitch_prev_gyro_val;
extern UINT16 adc_yaw_prev_gyro_val;

// gyro values in their neutral points
extern UINT16 neutral_roll_gyro_val;
extern UINT16 neutral_pitch_gyro_val;
extern UINT16 neutral_yaw_gyro_val;

// some constants that deal with radio signal
#define CENTER_PULSE_VAL          1520
#define OFF_PULSE_VAL              900 
#define MIN_PULSE_ON	          1000
#define MAX_PULSE_ON	          2200

// trial and error shows that gain needs to be around 0.3 to 4.
// use fixed point arithmetic of base 256 gain val goes from 76 - 1000.
// tx_gain_pulse is a standard servo pulse, so its range is from 1000 - 2000.
#define TX_GAIN_OFFSET            (900)	 // 1000 - 100 // for wii gyro.  
                                         // gain from (1000-900)/256 - (2000-900)/256
                                         // 0.39 - 4.3 



void mix_mixing_quad  (INT16 ail, INT16 pit, INT16 thr, INT16 rud, 
                       INT16* fr,  INT16* bk,  INT16* al,  INT16* ar);
