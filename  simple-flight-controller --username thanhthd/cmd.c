/*===================================================================
 * Project: simple-flight-controller 
 *
 * Command Line Processing module
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
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *===================================================================
*/

#include <string.h>
#include "types.h"
#include "cmd.h"
#include "captures.h"

#include "utils.h"
#include "hal.h"
#include "mixer.h"
#include "quad_main.h"

char cmdBuffer[CMD_MAX_LEN];
unsigned char cmdLen = 0;

//-------------------------------------------------------------------
// input: the string contain the command and its length
//-------------------------------------------------------------------
void process_cmd(char* cmd_str, unsigned char cmd_length)
{
   if (strcmp(cmd_str,"get_pulse") == 0)
   {
   	show_info();      
   }
    
   else if (strcmp(cmd_str, "cal_on")==0)
   {
      mixer_flags |= MIXER_CALIBRATE_ON;	//calibrate_on= 1;
   }
   else if (strcmp(cmd_str,"cal_off")==0)
   {
      mixer_flags &= ~MIXER_CALIBRATE_ON;	//calibrate_on= 0;      
   }
   
   else if (strcmp(cmd_str,"dis_input_on")==0)
   {
      mixer_flags |= MIXER_DISC_INPUT_ON; //disc_input = 1;
   }
   else if (strcmp(cmd_str,"dis_input_off")==0)
   {
      mixer_flags &= ~MIXER_DISC_INPUT_ON;	//disc_input = 0;
   }
   
   else if (strcmp(cmd_str,"gyro_test_on")==0)
   {
      mixer_flags |= MIXER_GYRO_TEST_ON;
   }
   else if (strcmp(cmd_str,"gyro_test_off")==0)
   {
      mixer_flags &= ~MIXER_GYRO_TEST_ON;	
   }

   else if (strcmp(cmd_str,"gyro_filter_on")==0)
   {
      mixer_flags |= MIXER_GYRO_FILTER_ON;
   }
   else if (strcmp(cmd_str,"gyro_filter_off")==0)
   {
      mixer_flags &= ~MIXER_GYRO_FILTER_ON;	
   }
   
   else if (strcmp(cmd_str,"debug_on")==0)
   {
      mixer_flags |= MIXER_PRINT_DEBUG_ON;	
   }
   else if (strcmp(cmd_str,"debug_off")==0)
   {
      mixer_flags &= ~MIXER_PRINT_DEBUG_ON;	
   }

   else if (strstr(cmd_str,"gyro_gain")!=0)
   {
      copter_config_data.gyro_gain = parse_value(cmd_str, cmd_length);
      // debug
      printU16(copter_config_data.gyro_gain);      
      tx_string("\n\r",2);                  
   }
   
   else if (strstr(cmd_str,"x_bias")!=0)
   {
      copter_config_data.gain_x_bias = parse_value(cmd_str, cmd_length);
      // debug
      printU16(copter_config_data.gain_x_bias);      
      tx_string("\n\r",2);                  
   }
   else if (strstr(cmd_str,"y_bias")!=0)
   {
      copter_config_data.gain_y_bias = parse_value(cmd_str, cmd_length);
      // debug
      printU16(copter_config_data.gain_y_bias);      
      tx_string("\n\r",2);      
   }
   else if (strstr(cmd_str,"z_bias")!=0)
   {
      copter_config_data.gain_z_bias = parse_value(cmd_str, cmd_length);
      // debug
      printU16(copter_config_data.gain_z_bias);      
      tx_string("\n\r",2);            
   }   
   else if (strcmp(cmd_str,"flip_roll")==0)
   {
      copter_config_data.gyro_dir ^= ROLL_GYRO_DIR;
   }
   else if (strcmp(cmd_str,"flip_pitch")==0)
   {
      copter_config_data.gyro_dir ^= PITCH_GYRO_DIR;
   }
   else if (strcmp(cmd_str,"flip_yaw")==0)
   {
      copter_config_data.gyro_dir ^= YAW_GYRO_DIR;
   }
   else if (strcmp(cmd_str,"save")==0)
   {
   	if (sizeof(copter_config_data)>64) /* size of flash segment */
   	{
   		tx_string("config too big!\n\r",17);
   		return;
   	}
      write_flash(FLASH_CONFIG_ADDR, (char*)&copter_config_data, sizeof(copter_config_data));      
   }
   else if (strcmp(cmd_str,"read")==0)
   {
      read_config_from_flash();
   }
   else if (strcmp(cmd_str,"clear")==0)
   {
   	clear_stats();
   }
   else
   {
   	tx_string("error!\n\r",8);
   }
}


/*------------------------------------------------------------------------------
// show various data: rx pulses, motor outputs, stats
------------------------------------------------------------------------------*/
void show_info()
{
   tx_string("\n\r-----------",13);
   tx_string("\n\rail pulse = ",15);
   printU16(ail_pulse);      

   tx_string("\n\rpit pulse = ",15);
   printU16(pit_pulse);      

   tx_string("\n\rthr pulse = ",15);
   printU16(thr_pulse);      

   tx_string("\n\rrud pulse = ",15);
   printU16(rud_pulse);      

   tx_string("\n\rgain pulse= ",15);
   printU16(tx_gain_pulse);      

   tx_string("\n\r-----------",13);

   tx_string("\n\radc ail = ",12);
   printU16(adc_roll_gyro_val);      

   tx_string("\n\radc pit = ",12);
   printU16(adc_pitch_gyro_val);      

   tx_string("\n\radc yaw = ",12);
   printU16(adc_yaw_gyro_val);      

   tx_string("\n\rail gyro = ",13);
   printU16(neutral_roll_gyro_val);      

   tx_string("\n\rpit gyro = ",13);
   printU16(neutral_pitch_gyro_val);      

   tx_string("\n\ryaw gyro = ",13);
   printU16(neutral_yaw_gyro_val);      

   tx_string("\n\r--",4);

   tx_string("\n\rFr Motor = ",13);
   printU16(front_motor);      

   tx_string("\n\rBk Motor = ",13);
   printU16(back_motor);      

   tx_string("\n\rRt Motor = ",13);
   printU16(right_motor);      

   tx_string("\n\rLt Motor = ",13);
   printU16(left_motor);      
   
   tx_string("\n\r---",5);                   
   tx_string("\n\rx gyro: ",10); printU16(copter_config_data.min_x_gyro_rate);                   
   tx_string(" - ",3);           printU16(copter_config_data.max_x_gyro_rate);                   

   tx_string("\n\ry gyro: ",10); printU16(copter_config_data.min_y_gyro_rate);                   
   tx_string(" - ",3);           printU16(copter_config_data.max_y_gyro_rate);                   

   tx_string("\n\rz gyro: ",10); printU16(copter_config_data.min_z_gyro_rate);                   
   tx_string(" - ",3);           printU16(copter_config_data.max_z_gyro_rate);                   

   tx_string("\n\rm1: ",6); printU16(copter_config_data.min_m1_pulse);                   
   tx_string(" - ",3);      printU16(copter_config_data.max_m1_pulse);
                      
   tx_string("\n\rm2: ",6); printU16(copter_config_data.min_m2_pulse);                   
   tx_string(" - ",3);      printU16(copter_config_data.max_m2_pulse);
                      
   tx_string("\n\rm3: ",6); printU16(copter_config_data.min_m3_pulse);                   
   tx_string(" - ",3);      printU16(copter_config_data.max_m3_pulse);                   
   tx_string("\n\rm4: ",6); printU16(copter_config_data.min_m4_pulse);                   
   tx_string(" - ",3);      printU16(copter_config_data.max_m4_pulse);                   
   tx_string("\n\r---",5);                   
   tx_string("\n\rmixer flag = ",15); printU16(mixer_flags);
   tx_string("\n\rgyro gain  = ",15); printU16(tx_gain_pulse - TX_GAIN_OFFSET);
         
   tx_string("\n\r",2);                   
}

/*------------------------------------------------------------------------------
// clear gyro & motor stats
------------------------------------------------------------------------------*/
void clear_stats()
{
   copter_config_data.min_x_gyro_rate = 9999;
   copter_config_data.min_y_gyro_rate = 9999;
   copter_config_data.min_z_gyro_rate = 9999;

   copter_config_data.max_x_gyro_rate = 0;
   copter_config_data.max_y_gyro_rate = 0;
   copter_config_data.max_z_gyro_rate = 0;
   
   copter_config_data.min_m1_pulse = 9999;
   copter_config_data.max_m1_pulse = 0;
   copter_config_data.min_m2_pulse = 9999;
   copter_config_data.max_m2_pulse = 0;
   copter_config_data.min_m3_pulse = 9999;
   copter_config_data.max_m3_pulse = 0;
   copter_config_data.min_m4_pulse = 9999;
   copter_config_data.max_m4_pulse = 0;
}

