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

extern UINT16 front_motor;
extern UINT16 back_motor;
extern UINT16 left_motor;
extern UINT16 right_motor;

#define ROLL_GYRO_DIR            (1<<0)
#define PITCH_GYRO_DIR           (1<<1)
#define YAW_GYRO_DIR             (1<<2)
#define DEFAULT_GYRO_GAIN         (900)   // adjust this value for gyro gain 
                                          // if no remote gain is used (channel 5 
                                          // is not connected to the controller)
#define INIT_DELAY_TIME_100MS    (50)     // wait for 5 seconds (50 x 0.1S) after powering up 
                                          // before initializing gyros

#define NUM_GYRO_SAMPLES         16       // this value is the number of gyro 
                                          // sample taken before averaging it out 
                                          // for calculating the gyro neutral values

extern CAPTURE_CCB ail_ccb;
extern CAPTURE_CCB pit_ccb;
extern CAPTURE_CCB thr_ccb;
extern CAPTURE_CCB rud_ccb;
extern CAPTURE_CCB tx_gain_ccb;

extern UINT16 ail_pulse;
extern UINT16 pit_pulse;
extern UINT16 thr_pulse;
extern UINT16 rud_pulse;
extern UINT16 tx_gain_pulse;

void read_config_from_flash();


typedef struct 
{
   UINT8    gyro_dir;
   INT16    gyro_gain;
   INT16    gain_x_bias;
   INT16    gain_y_bias;
   INT16    gain_z_bias;
   
   // the following are statistic logging data
   UINT16   min_x_gyro_rate;
   UINT16   min_y_gyro_rate;
   UINT16   min_z_gyro_rate;   

   UINT16   max_x_gyro_rate;
   UINT16   max_y_gyro_rate;
   UINT16   max_z_gyro_rate;
      
   UINT16   min_m1_pulse;
   UINT16   max_m1_pulse;
   UINT16   min_m2_pulse;
   UINT16   max_m2_pulse;
   UINT16   min_m3_pulse;
   UINT16   max_m3_pulse;
   UINT16   min_m4_pulse;
   UINT16   max_m4_pulse;
} COPTER_CONFIG_DATA;

extern COPTER_CONFIG_DATA	copter_config_data;


extern UINT8   woken_up_by;
#define WOKEN_UP_BY_UNKNOWN   0
#define WOKEN_UP_BY_ADC       (1<<1)
#define WOKEN_UP_BY_TIMER     (1<<2)
#define WOKEN_UP_BY_CMD       (1<<3)
#define WOKEN_UP_BY_WIRELESS  (1<<4)

#endif
