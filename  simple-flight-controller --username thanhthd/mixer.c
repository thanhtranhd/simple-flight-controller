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
 * along with Simple-Flight-Controller.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *===================================================================
*/

#include "types.h"
#include "mixer.h"
#include "hal.h"
#include "captures.h"
#include "quad_main.h"
#include "utils.h"

//------------------------------------------------------------------
// initial values
//------------------------------------------------------------------
UINT16 adc_roll_gyro_val  = 0;
UINT16 adc_pitch_gyro_val = 0;
UINT16 adc_yaw_gyro_val   = 0;

UINT16 adc_roll_prev_gyro_val  = 0;
UINT16 adc_pitch_prev_gyro_val = 0;
UINT16 adc_yaw_prev_gyro_val   = 0;

UINT16 neutral_roll_gyro_val  = 0;
UINT16 neutral_pitch_gyro_val = 0;
UINT16 neutral_yaw_gyro_val   = 0;

UINT16 mixer_flags = 0;

//------------------------------------------------------------------
// function to do the mixing to control the flying of quad copter.
// inputs: 
//     ail, pit, thr, rud: inputs from the receiver
// outputs: 
//     fr, bk, al, ar: the front, back, left, and right motors
//
// external gyro values and various settings & flags are global 
// and used here.
//  
//------------------------------------------------------------------
void mix_mixing_quad  (INT16 ail, INT16 pit, INT16 thr, INT16 rud, 
                       INT16* fr,  INT16* bk,  INT16* al,  INT16* ar)
{  
   INT32 roll_gyro_rate;
   INT32 pitch_gyro_rate;
   INT32 yaw_gyro_rate;
   INT32 gain_val;
   
   static UINT16 last_fr;
   static UINT16 last_bk;
   static UINT16 last_al;
   static UINT16 last_ar;

   INT16 yaw              = 0;

   // get the active gyro rates by substracting the neutral values
   // without the (INT32) type cast here, somehow the multipilcation & right shift bellow
   // produce wrong result when the gain is not a power of 2 number.
   roll_gyro_rate   = (INT32)(adc_roll_gyro_val)  - (INT32)(neutral_roll_gyro_val);
   pitch_gyro_rate  = (INT32)(adc_pitch_gyro_val) - (INT32)(neutral_pitch_gyro_val);
   yaw_gyro_rate    = (INT32)(adc_yaw_gyro_val)   - (INT32)(neutral_yaw_gyro_val);

   // Add tx_gain_pulse to a number to get the desired gain value
   gain_val = tx_gain_pulse - TX_GAIN_OFFSET;      

   roll_gyro_rate  = (roll_gyro_rate  * (INT32)(gain_val+copter_config_data.gain_x_bias)) >> 8;
   pitch_gyro_rate = (pitch_gyro_rate * (INT32)(gain_val+copter_config_data.gain_y_bias)) >> 8;
   yaw_gyro_rate   = (yaw_gyro_rate   * (INT32)(gain_val+copter_config_data.gain_z_bias)) >> 8;

   //printU16(roll_gyro_rate);
   //tx_string("\n\r",2);

   // logging some stats: max gyro rates
   if (adc_roll_gyro_val > copter_config_data.max_x_gyro_rate) 
      copter_config_data.max_x_gyro_rate = adc_roll_gyro_val;
      
   if (adc_pitch_gyro_val> copter_config_data.max_y_gyro_rate) 
       copter_config_data.max_y_gyro_rate = adc_pitch_gyro_val;
      
   if (adc_yaw_gyro_val > copter_config_data.max_z_gyro_rate) 
      copter_config_data.max_z_gyro_rate = adc_yaw_gyro_val;
      
   // logging some stats: min gyro rates
   if (adc_roll_gyro_val < copter_config_data.min_x_gyro_rate) 
      copter_config_data.min_x_gyro_rate = adc_roll_gyro_val;
      
   if (adc_pitch_gyro_val < copter_config_data.min_y_gyro_rate) 
      copter_config_data.min_y_gyro_rate = adc_pitch_gyro_val;
      
   if (adc_yaw_gyro_val < copter_config_data.min_z_gyro_rate) 
      copter_config_data.min_z_gyro_rate = adc_yaw_gyro_val;
      
      
   // adjust gyro direction based on configuration.
   if (copter_config_data.gyro_dir & ROLL_GYRO_DIR)  roll_gyro_rate  = 0 - (INT16)roll_gyro_rate;
   if (copter_config_data.gyro_dir & PITCH_GYRO_DIR) pitch_gyro_rate = 0 - (INT16)pitch_gyro_rate ;      
   if (copter_config_data.gyro_dir & YAW_GYRO_DIR)   yaw_gyro_rate   = 0 - (INT16)yaw_gyro_rate;
                  
   yaw = rud - CENTER_PULSE_VAL + yaw_gyro_rate;
      
   // actual mixing
   if (!(mixer_flags & MIXER_CALIBRATE_ON)) 
   { 
      *fr = thr - (pit - CENTER_PULSE_VAL) - yaw + (INT16)(pitch_gyro_rate);
      *bk = thr + (pit - CENTER_PULSE_VAL) - yaw - (INT16)(pitch_gyro_rate);
      *al = thr - (ail - CENTER_PULSE_VAL) + yaw - (INT16)(roll_gyro_rate);
      *ar = thr + (ail - CENTER_PULSE_VAL) + yaw + (INT16)(roll_gyro_rate);

      if (mixer_flags & MIXER_PRINT_DEBUG_ON)
      {     
         tx_string("al=",3);printU16(*al);tx_string("ar=",3);printU16(*ar);
         tx_string("\n\r",2);
      }

      // turn off all motors if GYRO test mode is not turned on
      // of if the throtle is < 1000uS.
      if ((thr < 1000) && 
          !(mixer_flags & MIXER_GYRO_TEST_ON))
      {
         *fr = OFF_PULSE_VAL;
         *bk = OFF_PULSE_VAL;
         *al = OFF_PULSE_VAL;
         *ar = OFF_PULSE_VAL;
         
         return;
      }
               

      // logging motor pulse values
      // logging max
      if (*fr > copter_config_data.max_m1_pulse) 
         copter_config_data.max_m1_pulse = *fr;
      if (*ar > copter_config_data.max_m2_pulse) 
         copter_config_data.max_m2_pulse = *ar;
      if (*bk > copter_config_data.max_m3_pulse) 
         copter_config_data.max_m3_pulse = *bk; 
      if (*al > copter_config_data.max_m4_pulse) 
         copter_config_data.max_m4_pulse = *al; 
          
      // logging min
      if (*fr < copter_config_data.min_m1_pulse) 
         copter_config_data.min_m1_pulse = *fr;
      if (*ar < copter_config_data.min_m2_pulse) 
         copter_config_data.min_m2_pulse = *ar; 
      if (*bk < copter_config_data.min_m3_pulse) 
         copter_config_data.min_m3_pulse = *bk; 
      if (*al < copter_config_data.min_m4_pulse) 
         copter_config_data.min_m4_pulse = *al; 

      if (!(mixer_flags & MIXER_GYRO_TEST_ON))
      {
         // range checking...
         if ((*fr < MIN_PULSE_ON) || (*fr > MAX_PULSE_ON)) *fr = last_fr; else last_fr = *fr; 
         if ((*bk < MIN_PULSE_ON) || (*bk > MAX_PULSE_ON)) *bk = last_bk; else last_bk = *bk;
         if ((*al < MIN_PULSE_ON) || (*al > MAX_PULSE_ON)) *al = last_al; else last_al = *al; 
         if ((*ar < MIN_PULSE_ON) || (*ar > MAX_PULSE_ON)) *ar = last_ar; else last_ar = *ar;
      }             
   }
   else 
   {
      // mixer is off, calibration is on: output same throttle for all outputs
      *fr = thr;
      *bk = thr;
      *al = thr;
      *ar = thr;
   }
}

